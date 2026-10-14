// 1D tracking with a Kalman filter: a cart on a track, a noisy position
// sensor, and a constant-velocity filter. Pure JS, no DOM, so behavior can be
// checked under node --test.

export const TRUE_Q = 1; // m²/s³, acceleration noise of the "Random accel" truth
export const TRUTH_DT = 1 / 200;
export const HISTORY_SECONDS = 12;
export const DROPOUT = { period: 10, start: 5, length: 3 }; // seconds
export const P0 = [[4, 0], [0, 4]]; // starting uncertainty: 2 m and 2 m/s
export const WARMUP_S = 12; // longer than the 10 s chart window
// Outliers: about one position reading in 15 is way off, the kind of thing a
// reflection or a glitch gives you.
export const OUTLIER_RATE = 1 / 15;
// Chi-square 99% points: a reading that fits the filter's own uncertainty
// lands past these only 1% of the time. One number for position alone, two
// with the velocity sensor fused.
export const GATE = { 1: 6.63, 2: 9.21 };

// Weak pull toward the middle of the track for "Random accel". Pure white-noise
// acceleration drifts off without bound, faster every minute. These terms are
// small next to the noise, so the filter's model still fits closely.
const PULL = { k: 0.05, c: 0.2 };

// "You steer": the cart is pulled toward wherever you are dragging, critically
// damped so it sets off and settles smoothly rather than teleporting. omega of
// 5 rad/s settles in about half a second, direct enough to feel like steering
// and slow enough that the constant-velocity model has something to chase.
export const DRIVE = { w: 5, reach: 40 };

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

export const PROFILES = {
  random: {
    name: "Random accel",
    blurb: "The acceleration is white noise with q = 1 m²/s³, plus a weak pull toward the middle so the cart stays in view. This is the filter's own model, so well tuned is close to optimal.",
  },
  sine: {
    name: "Sine",
    blurb: "The cart swings 4 m each way every 8 s. The motion is smooth but not random, so the model is an approximation.",
  },
  stopgo: {
    name: "Stop and go",
    blurb: "Constant speed with sudden changes every few seconds. The model expects smooth acceleration, so each jump makes the estimate lag until it catches up.",
  },
  drive: {
    name: "You steer",
    blurb: "Drag along the track to pull the cart where you want it, or steer with the arrow keys. The filter is never told what you did, so your maneuver only reaches it through noisy position readings, a sample late. Raise q if you want it to keep up, and watch what that costs.",
  },
};

export const DEFAULTS = {
  profile: "random",
  rate: 10, // Hz
  sigmaZ: 0.5, // m, true position noise
  fuseVel: false,
  sigmaV: 0.2, // m/s, velocity noise; the filter is told the true value
  dropout: false,
  outliers: false,
  gate: false, // reject readings whose d² is past GATE
  driveTarget: 0, // m, where "You steer" is pulling the cart
  filterSigmaZ: 0.5, // m, what the filter assumes; R = filterSigmaZ²
  filterQ: TRUE_Q,
};

/** Filter settings for each preset, given the true sensor noise. */
export const PRESETS = {
  tuned: { name: "Well tuned", apply: (p) => ({ filterSigmaZ: p.sigmaZ, filterQ: TRUE_Q }) },
  sensor: { name: "Trusts sensor too much", apply: (p) => ({ filterSigmaZ: p.sigmaZ / 10, filterQ: TRUE_Q }) },
  model: { name: "Trusts model too much", apply: (p) => ({ filterSigmaZ: p.sigmaZ, filterQ: TRUE_Q / 1000 }) },
  // R and Q both scaled by 9 give the same gain as well tuned, with a band
  // three times too wide. Only the stated confidence is wrong.
  cautious: { name: "Overcautious", apply: (p) => ({ filterSigmaZ: p.sigmaZ * 3, filterQ: TRUE_Q * 9 }) },
};

// ---- Small matrix helpers. Matrices are arrays of rows; vectors are columns. ----
export const transpose = (A) => A[0].map((_, j) => A.map((r) => r[j]));
export const mul = (A, B) => A.map((r) => B[0].map((_, j) => r.reduce((s, a, k) => s + a * B[k][j], 0)));
export const add = (A, B) => A.map((r, i) => r.map((a, j) => a + B[i][j]));
export const sub = (A, B) => A.map((r, i) => r.map((a, j) => a - B[i][j]));
export const eye = (n) => Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
export function inv(A) {
  if (A.length === 1) return [[1 / A[0][0]]];
  const [[a, b], [c, d]] = A;
  const det = a * d - b * c;
  return [[d / det, -b / det], [-c / det, a / det]];
}
// Rounding makes P drift off symmetric over thousands of steps; average it back.
function symmetrize(P) {
  const o = (P[0][1] + P[1][0]) / 2;
  return [[P[0][0], o], [o, P[1][1]]];
}

export const transition = (dt) => [[1, dt], [0, 1]];

/** Q for continuous white-noise acceleration with spectral density q. */
export const processNoise = (q, dt) => [
  [(q * dt ** 3) / 3, (q * dt ** 2) / 2],
  [(q * dt ** 2) / 2, q * dt],
];

/** Small seedable RNG so runs repeat exactly. */
export function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export class KalmanFilter {
  constructor() {
    this.reset([0, 0], P0);
  }

  reset(x, P) {
    this.x = [[x[0]], [x[1]]];
    this.P = P.map((r) => [...r]);
  }

  predict(dt, q) {
    const F = transition(dt);
    const Q = processNoise(q, dt);
    this.x = mul(F, this.x);
    this.P = symmetrize(add(mul(mul(F, this.P), transpose(F)), Q));
    return { F, Q };
  }

  /**
   * The innovation y (how far the reading is from the prediction), its
   * covariance S, and d² = yᵀS⁻¹y: how surprising the reading is, in squared
   * standard deviations.
   */
  innovation(z, H, R) {
    const y = sub(z, mul(H, this.x));
    const S = add(mul(mul(H, this.P), transpose(H)), R);
    const d2 = mul(mul(transpose(y), inv(S)), y)[0][0];
    return { y, S, d2 };
  }

  update(z, H, R) {
    const xPrior = this.x, PPrior = this.P;
    const y = sub(z, mul(H, xPrior));
    const S = add(mul(mul(H, PPrior), transpose(H)), R);
    const K = mul(mul(PPrior, transpose(H)), inv(S));
    this.x = add(xPrior, mul(K, y));
    this.P = symmetrize(mul(sub(eye(2), mul(K, H)), PPrior));
    return { y, S, K };
  }
}

export const inDropout = (t) => {
  const m = t % DROPOUT.period;
  return m >= DROPOUT.start && m < DROPOUT.start + DROPOUT.length;
};

export class Simulation {
  constructor(params = {}, seed = 1) {
    this.params = { ...DEFAULTS, ...params };
    this.seed = seed;
    this.filter = new KalmanFilter();
    this.reset();
  }

  /** Change settings. A new motion profile restarts the run. */
  set(changes) {
    const prev = { ...this.params };
    Object.assign(this.params, changes);
    if (this.params.profile !== prev.profile) {
      this.reset();
    } else if (this.params.rate !== prev.rate) {
      this.nextFilter = Math.max(this.tFilter + 1 / this.params.rate, this.t);
    }
  }

  reset() {
    // Outliers draw from their own stream, so turning them on doesn't change
    // the cart's motion or the ordinary noise.
    this.outlierRng = mulberry32(this.seed ^ 0x9e3779b9);
    const rng = mulberry32(this.seed);
    this.gauss = () => {
      const u = 1 - rng();
      return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rng());
    };
    this.rng = rng;
    this.t = 0;
    this.truth = this.params.profile === "sine" ? sineAt(0) : { p: 0, v: 0, segEnd: 1.5 };
    if (this.params.profile === "stopgo") this.truth.v = 2;
    // Start under your hand, so the warm-up settles where you left the cart.
    if (this.params.profile === "drive") this.truth.p = this.driveTarget();
    this.filter.reset([0, 0], P0);
    this.tFilter = 0;
    this.nextFilter = 1 / this.params.rate;
    this.history = [];
    this.meas = [];
    this.gains = [];
    this.last = null;
    this.totals = { n: 0, se: 0, inside: 0, measN: 0, measSe: 0 };
    this.record();
  }

  /** A reading that's way off: several meters, well past any sensible σ. */
  outlierOffset() {
    const r = this.outlierRng;
    const mag = (2 + 8 * this.params.sigmaZ) * (0.75 + 0.5 * r());
    return (r() < 0.5 ? -1 : 1) * mag;
  }

  /** Where "You steer" is pulling the cart, kept on the track. */
  driveTarget() {
    const v = this.params.driveTarget;
    return Number.isFinite(v) ? clamp(v, -DRIVE.reach, DRIVE.reach) : 0;
  }

  advanceTruth(h) {
    const s = this.truth;
    const prof = this.params.profile;
    if (prof === "sine") {
      Object.assign(s, sineAt(this.t + h));
    } else if (prof === "drive") {
      const target = this.driveTarget();
      s.v += (DRIVE.w ** 2 * (target - s.p) - 2 * DRIVE.w * s.v) * h;
      s.p += s.v * h;
    } else if (prof === "random") {
      // Semi-implicit Euler at 5 ms: close enough to the continuous model.
      s.v += (-PULL.k * s.p - PULL.c * s.v) * h + Math.sqrt(TRUE_Q * h) * this.gauss();
      s.p += s.v * h;
    } else {
      let left = h, t = this.t;
      while (left > 0) {
        const d = Math.min(left, s.segEnd - t);
        s.p += s.v * d;
        t += d;
        left -= d;
        if (t >= s.segEnd - 1e-12) this.newSegment(t);
      }
    }
  }

  // Stop and go: hold a speed for 1.5 to 4 s, then jump. Far from the middle
  // it heads back, so the cart stays near the track center.
  newSegment(t) {
    const s = this.truth;
    const r = this.rng();
    if (r < 0.3) s.v = 0;
    else {
      const dir = Math.abs(s.p) > 5 ? -Math.sign(s.p) : this.rng() < 0.5 ? -1 : 1;
      s.v = dir * (1 + 2 * this.rng());
    }
    s.segEnd = t + 1.5 + 2.5 * this.rng();
  }

  /** Advance the truth one small step, running the filter when a sample is due. */
  step() {
    const h = Math.min(TRUTH_DT, this.nextFilter - this.t);
    if (h > 0) this.advanceTruth(h);
    this.t += Math.max(0, h);
    if (this.t >= this.nextFilter - 1e-9) {
      this.t = this.nextFilter;
      this.filterStep();
    }
    this.record();
  }

  advance(seconds) {
    const end = this.t + seconds;
    let n = 0;
    while (this.t < end - 1e-9 && n++ < 4000) this.step();
  }

  filterStep() {
    const p = this.params;
    const dt = this.t - this.tFilter;
    this.tFilter = this.t;
    this.nextFilter = this.t + 1 / p.rate;
    const f = this.filter;
    const xPrev = f.x, PPrev = f.P;
    const { F, Q } = f.predict(dt, p.filterQ);
    const xPrior = f.x, PPrior = f.P;
    const dropped = p.dropout && inDropout(this.t);

    let H = null, R = null, z = null, upd = null, gate = null;
    if (!dropped) {
      let zp = this.truth.p + p.sigmaZ * this.gauss();
      // Always draw, so the outlier pattern doesn't shift when they're toggled.
      const roll = this.outlierRng();
      const outlier = p.outliers && roll < OUTLIER_RATE;
      if (outlier) zp += this.outlierOffset();
      const rz = p.filterSigmaZ ** 2;
      if (p.fuseVel) {
        const zv = this.truth.v + p.sigmaV * this.gauss();
        H = [[1, 0], [0, 1]];
        R = [[rz, 0], [0, p.sigmaV ** 2]];
        z = [[zp], [zv]];
      } else {
        H = [[1, 0]];
        R = [[rz]];
        z = [[zp]];
      }
      const inn = f.innovation(z, H, R);
      const limit = GATE[z.length];
      gate = { y: inn.y, S: inn.S, d2: inn.d2, limit, rejected: p.gate && inn.d2 > limit, outlier };
      // A rejected reading never reaches the update: the filter keeps its
      // prediction, and P keeps growing until a reading fits again.
      if (!gate.rejected) upd = f.update(z, H, R);
      this.meas.push({ t: this.t, z: zp, truth: this.truth.p, outlier, rejected: gate.rejected });
      this.totals.measN++;
      this.totals.measSe += (zp - this.truth.p) ** 2;
    }
    // Column 0 of K: how far a position innovation moves p and v.
    this.gains.push({ t: this.t, kp: upd ? upd.K[0][0] : null, kv: upd ? upd.K[1][0] : null });
    this.last = { t: this.t, dt, q: p.filterQ, F, Q, xPrev, PPrev, xPrior, PPrior, H, R, z, ...upd, x: f.x, P: f.P, dropped, gate };
  }

  /**
   * One history sample. Between filter steps the estimate is the prediction
   * carried forward to now, with P grown by the same model, so the band on
   * the charts shows the filter's belief at every instant.
   */
  record() {
    const s = this.t - this.tFilter;
    const x = this.filter.x, P = this.filter.P, q = this.params.filterQ;
    const pe = x[0][0] + x[1][0] * s;
    const p00 = P[0][0] + 2 * s * P[0][1] + s * s * P[1][1] + (q * s ** 3) / 3;
    const p11 = P[1][1] + q * s;
    const sp = Math.sqrt(p00);
    const e = pe - this.truth.p;
    this.history.push({
      t: this.t, p: this.truth.p, v: this.truth.v, pe, ve: x[1][0], sp, sv: Math.sqrt(p11),
      drop: this.params.dropout && inDropout(this.t),
    });
    const tot = this.totals;
    tot.n++;
    tot.se += e * e;
    if (Math.abs(e) <= 2 * sp) tot.inside++;

    const cutoff = this.t - HISTORY_SECONDS;
    for (const list of [this.history, this.meas, this.gains]) {
      let drop = 0;
      while (drop < list.length && list[drop].t < cutoff) drop++;
      if (drop > 64) list.splice(0, drop);
    }
  }

  /**
   * Run quietly before anything is shown. From a cold start the filter spends
   * its first second or two converging: the band balloons, the gain spikes
   * and the charts rescale. Running past the 10 s chart window first means the
   * page opens on a filter already at steady state with full charts.
   */
  warmUp(seconds = WARMUP_S) {
    this.advance(seconds);
    this.totals = { n: 0, se: 0, inside: 0, measN: 0, measSe: 0 };
  }

  /** Error statistics over the last `window` seconds. */
  metrics(window = 10) {
    const t0 = this.t - window;
    let n = 0, se = 0, inside = 0;
    for (const h of this.history) {
      if (h.t < t0) continue;
      const e = h.pe - h.p;
      n++;
      se += e * e;
      if (Math.abs(e) <= 2 * h.sp) inside++;
    }
    let mn = 0, mse = 0;
    for (const m of this.meas) {
      if (m.t < t0) continue;
      mn++;
      mse += (m.z - m.truth) ** 2;
    }
    let outliers = 0, rejected = 0, rejectedOutliers = 0;
    for (const m of this.meas) {
      if (m.t < t0) continue;
      if (m.outlier) outliers++;
      if (m.rejected) rejected++;
      if (m.outlier && m.rejected) rejectedOutliers++;
    }
    const last = this.history[this.history.length - 1];
    return {
      outliers, rejected, rejectedOutliers,
      estRmse: n ? Math.sqrt(se / n) : null,
      measRmse: mn ? Math.sqrt(mse / mn) : null,
      coverage: n ? inside / n : null,
      sigma: last.sp,
      span: Math.min(window, this.t),
    };
  }

  /** The same statistics over the whole run, for tests. */
  totalMetrics() {
    const t = this.totals;
    return {
      estRmse: Math.sqrt(t.se / t.n),
      measRmse: Math.sqrt(t.measSe / t.measN),
      coverage: t.inside / t.n,
    };
  }
}

function sineAt(t) {
  const A = 4, w = (2 * Math.PI) / 8;
  return { p: A * Math.sin(w * t), v: A * w * Math.cos(w * t) };
}
