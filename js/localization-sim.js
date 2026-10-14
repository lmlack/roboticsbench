// Robot localization against known landmarks, with walls that block the view
// and the robot. Five estimators run side by side on the same data: dead
// reckoning, a particle filter (Monte Carlo localization), an EKF, a UKF and
// a grid (histogram) filter. Pure JS, no DOM, and every random draw comes
// from a seeded generator so the behavior can be checked under node --test.

import { gridSearch, run } from "./planning-sim.js";

export const WORLD = { w: 10, h: 7 };
export const DT = 0.05; // seconds per motion and sensor step
export const HISTORY_SECONDS = 20;
export const TRAIL_SECONDS = 8;
export const GATE = 13.82; // chi-square, 2 dof, 99.9%: EKF rejects anything further out
export const ELLIPSE_K2 = 5.991; // chi-square, 2 dof, 95%
export const RECOVERED = 0.3; // meters, what counts as "found the robot"

export const DEFAULT_LANDMARKS = [
  { x: 1.2, y: 0.9 }, { x: 5, y: 0.7 }, { x: 8.8, y: 1 },
  { x: 9.2, y: 6 }, { x: 5, y: 6.3 }, { x: 0.8, y: 6 }, { x: 5, y: 3.5 },
];

// A loop around the central landmark. The autopilot plans between these with
// A*, so walls may cross the loop.
export const WAYPOINTS = [
  [2, 2], [5, 1.8], [8, 2], [8.4, 3.5], [8, 5], [5, 5.2], [2, 5], [1.6, 3.5],
];
const START = { x: 2, y: 2, th: 0 };

/**
 * Velocity motion model noise (Probabilistic Robotics, table 5.3): the
 * executed v and ω are the commanded ones plus zero-mean Gaussian noise with
 *   var(v) = α1 v² + α2 ω²,  var(ω) = α3 v² + α4 ω².
 * One slider scales all four. At ×1 a 20 s loop drifts about 10° in heading
 * and half a meter in position by dead reckoning alone: plenty for the
 * filters to visibly correct, small enough that both track it with landmarks.
 */
export const ALPHA = [0.04, 0.01, 0.02, 0.06];
// Filters add a little process noise on top of the model so the particle
// cloud and the EKF covariance never shrink to a point while the robot is
// stopped and the model predicts zero noise.
export const FLOOR = { v: 0.05, w: 0.05 };
// Bearing noise follows the range noise slider at this ratio (rad per m), so
// 0.1 m of range noise comes with about 2.9° of bearing noise.
const BEARING_PER_RANGE = 0.5;

// Augmented MCL (Probabilistic Robotics, table 8.3). The averages update once
// per sensor step at 20 Hz, so these are time constants of about 50 s and 0.5 s.
export const A_SLOW = 0.001;
export const A_FAST = 0.1;
// Thrun's rule can ask for 100% random particles, which throws away the one
// good guess the moment it appears. Capping it keeps most of the set.
export const MAX_INJECT = 0.25;
// The scan-to-scan fit is noisy, so the short-term average often dips 20 to
// 30% below the long-term one with nothing wrong. Injection starts only once
// it falls below this fraction of the long-term average.
export const INJECT_RATIO = 0.5;
// Resampling also runs when injection is called for, so a kidnapped filter
// whose weights happen to stay even still gets fresh particles.
const INJECT_RESAMPLE = 0.05;

export const wrap = (a) => a - 2 * Math.PI * Math.floor((a + Math.PI) / (2 * Math.PI));

/** Small, fast, seedable PRNG returning floats in [0, 1). */
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

/** Box-Muller from a given uniform source, keeping the second value. */
export function makeGauss(rng) {
  let spare = null;
  return () => {
    if (spare !== null) {
      const s = spare;
      spare = null;
      return s;
    }
    const u = 1 - rng();
    const r = Math.sqrt(-2 * Math.log(u));
    const a = 2 * Math.PI * rng();
    spare = r * Math.sin(a);
    return r * Math.cos(a);
  };
}

/** Variances of the executed v and ω, for an odometry noise scale k. */
export function noiseVar(v, w, k) {
  return [k * (ALPHA[0] * v * v + ALPHA[1] * w * w), k * (ALPHA[2] * v * v + ALPHA[3] * w * w)];
}

/** Expected range and bearing from a pose to a landmark: h(x). */
export function expectedMeasurement(pose, lm) {
  const dx = lm.x - pose.x, dy = lm.y - pose.y;
  return { r: Math.hypot(dx, dy), b: wrap(Math.atan2(dy, dx) - pose.th) };
}

/**
 * Low-variance (systematic) resampling: one random offset u0 in [0, 1/n),
 * then n evenly spaced pointers into the cumulative weights. Returns the
 * index each new particle copies. Weights must sum to 1.
 */
export function systematicResample(w, n, u0) {
  const idx = new Int32Array(n);
  let c = w[0];
  let i = 0;
  for (let m = 0; m < n; m++) {
    const u = u0 + m / n;
    while (u > c && i < w.length - 1) c += w[++i];
    idx[m] = i;
  }
  return idx;
}

export class ParticleFilter {
  constructor(n, rng, gauss) {
    this.rng = rng;
    this.gauss = gauss;
    this.alloc(n);
  }

  alloc(n) {
    this.n = n;
    this.x = new Float64Array(n);
    this.y = new Float64Array(n);
    this.th = new Float64Array(n);
    // Log-weights are the source of truth. Many landmarks at small noise give
    // likelihoods like e^-400, which plain weights would round to zero.
    this.lw = new Float64Array(n).fill(-Math.log(n));
    this.w = new Float64Array(n).fill(1 / n);
    this.ll = new Float64Array(n);
    this.neff = n;
    this.inject = 0;
    this.injected = 0;
    this.resampled = false;
    this.wSlow = 0;
    this.wFast = 0;
    this.wAvg = 0;
    this.mean = null;
  }

  resetWeights() {
    this.lw.fill(-Math.log(this.n));
    this.w.fill(1 / this.n);
    this.neff = this.n;
  }

  initGaussian(pose, sxy, sth) {
    for (let i = 0; i < this.n; i++) {
      this.x[i] = pose.x + sxy * this.gauss();
      this.y[i] = pose.y + sxy * this.gauss();
      this.th[i] = wrap(pose.th + sth * this.gauss());
    }
    this.resetAverages();
  }

  /**
   * Both averages start at zero, as in the book. The fast one climbs first,
   * so nothing is injected while a fresh filter is still settling.
   */
  resetAverages() {
    this.resetWeights();
    this.wSlow = this.wFast = 0;
    this.inject = 0;
    this.mean = null;
  }

  initUniform() {
    for (let i = 0; i < this.n; i++) this.randomize(i, this.x, this.y, this.th);
    this.resetAverages();
  }

  randomize(i, x, y, th) {
    x[i] = this.rng() * WORLD.w;
    y[i] = this.rng() * WORLD.h;
    th[i] = (this.rng() * 2 - 1) * Math.PI;
  }

  /** Change the particle count by resampling to the new size. */
  resize(n) {
    if (n === this.n) return;
    const idx = systematicResample(this.w, n, this.rng() / n);
    const ox = this.x, oy = this.y, oth = this.th;
    const { wSlow, wFast } = this;
    this.alloc(n);
    for (let m = 0; m < n; m++) {
      this.x[m] = ox[idx[m]];
      this.y[m] = oy[idx[m]];
      this.th[m] = oth[idx[m]];
    }
    this.wSlow = wSlow;
    this.wFast = wFast;
  }

  /** Motion update: every particle samples its own noisy copy of the command. */
  predict(v, w, dt, k) {
    const [vv, vw] = noiseVar(v, w, k);
    const sv = Math.sqrt(vv + FLOOR.v * FLOOR.v);
    const sw = Math.sqrt(vw + FLOOR.w * FLOOR.w);
    const { x, y, th, gauss } = this;
    this.mean = null;
    for (let i = 0; i < this.n; i++) {
      const vn = v + sv * gauss();
      const wn = w + sw * gauss();
      const tm = th[i] + (wn * dt) / 2;
      x[i] += vn * dt * Math.cos(tm);
      y[i] += vn * dt * Math.sin(tm);
      th[i] = wrap(th[i] + wn * dt);
    }
  }

  /**
   * Measurement update. z is a list of { j, r, b }: landmark index, range,
   * bearing. Weights are multiplied by p(z | x), done as a sum of logs.
   */
  update(z, landmarks, sr, sb) {
    this.resampled = false;
    this.injected = 0;
    if (!z.length) return;
    const { x, y, th, lw, w, ll, n } = this;
    for (let i = 0; i < n; i++) {
      let s = 0;
      for (const m of z) {
        const lm = landmarks[m.j];
        const dx = lm.x - x[i], dy = lm.y - y[i];
        const er = (m.r - Math.hypot(dx, dy)) / sr;
        const eb = wrap(m.b - Math.atan2(dy, dx) + th[i]) / sb;
        s += er * er + eb * eb;
      }
      ll[i] = -0.5 * s;
    }

    // How well the set as a whole explains this scan. The per-landmark
    // geometric mean keeps the score from jumping when a landmark enters or
    // leaves range, which would otherwise read as a kidnapping.
    let avg = 0;
    for (let i = 0; i < n; i++) avg += w[i] * Math.exp(ll[i] / z.length);
    this.wAvg = avg;
    this.wSlow += A_SLOW * (avg - this.wSlow);
    this.wFast += A_FAST * (avg - this.wFast);
    const ratio = this.wSlow > 0 ? this.wFast / (INJECT_RATIO * this.wSlow) : 1;
    this.inject = Math.min(MAX_INJECT, Math.max(0, 1 - ratio));

    // Normalize in log space: subtract the max before exp, then the log of
    // the sum, so the largest weight is always exactly representable.
    let max = -Infinity;
    for (let i = 0; i < n; i++) {
      lw[i] += ll[i];
      if (lw[i] > max) max = lw[i];
    }
    if (!Number.isFinite(max)) {
      this.resetWeights();
      return;
    }
    let sum = 0;
    for (let i = 0; i < n; i++) sum += Math.exp(lw[i] - max);
    const logZ = max + Math.log(sum);
    let sq = 0;
    for (let i = 0; i < n; i++) {
      lw[i] -= logZ;
      w[i] = Math.exp(lw[i]);
      sq += w[i] * w[i];
    }
    const neff = 1 / sq;
    // The estimate comes from this weighted set. After resampling, the
    // injected random particles would drag a plain mean toward the map center.
    this.mean = this.weightedMean();
    if (neff < n / 2 || this.inject > INJECT_RESAMPLE) this.resample();
    // Report the value the decision was made on, not the N that resampling
    // leaves behind.
    this.neff = neff;
  }

  /** Systematic resampling, with a share of uniform random particles mixed in. */
  resample() {
    const n = this.n;
    const idx = systematicResample(this.w, n, this.rng() / n);
    const nx = new Float64Array(n), ny = new Float64Array(n), nth = new Float64Array(n);
    let injected = 0;
    for (let m = 0; m < n; m++) {
      if (this.rng() < this.inject) {
        this.randomize(m, nx, ny, nth);
        injected++;
      } else {
        nx[m] = this.x[idx[m]];
        ny[m] = this.y[idx[m]];
        nth[m] = this.th[idx[m]];
      }
    }
    this.x = nx;
    this.y = ny;
    this.th = nth;
    this.resetWeights();
    this.resampled = true;
    this.injected = injected;
  }

  estimate() {
    return this.mean ?? this.weightedMean();
  }

  /** Weighted mean pose. Heading is averaged as a unit vector. */
  weightedMean() {
    let x = 0, y = 0, c = 0, s = 0;
    for (let i = 0; i < this.n; i++) {
      const wi = this.w[i];
      x += wi * this.x[i];
      y += wi * this.y[i];
      c += wi * Math.cos(this.th[i]);
      s += wi * Math.sin(this.th[i]);
    }
    return { x, y, th: Math.atan2(s, c) };
  }
}

// ---- Tiny dense matrix helpers, arrays of rows ----
export const matMul = (A, B) => A.map((row) => B[0].map((_, j) => row.reduce((s, a, k) => s + a * B[k][j], 0)));
export const transpose = (A) => A[0].map((_, j) => A.map((row) => row[j]));
const matAdd = (A, B) => A.map((row, i) => row.map((a, j) => a + B[i][j]));
const eye3 = () => [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
const inv2 = ([[a, b], [c, d]]) => {
  const det = a * d - b * c;
  return [[d / det, -b / det], [-c / det, a / det]];
};

export class EKF {
  constructor() {
    this.init(START, 0.1, 0.1, 0.05);
  }

  init(pose, sx, sy, sth) {
    this.mu = [pose.x, pose.y, pose.th];
    this.P = [[sx * sx, 0, 0], [0, sy * sy, 0], [0, 0, sth * sth]];
    this.G = eye3();
    this.Pbar = this.P;
    this.last = null;
    this.rejected = 0;
  }

  /**
   * Predict with the same midpoint motion model the particles use.
   * G = ∂g/∂x carries the old uncertainty forward, V = ∂g/∂u maps the
   * command noise M into state space.
   */
  predict(v, w, dt, k) {
    const [x, y, th] = this.mu;
    const tm = th + (w * dt) / 2;
    const c = Math.cos(tm), s = Math.sin(tm);
    this.mu = [x + v * dt * c, y + v * dt * s, wrap(th + w * dt)];
    const G = [[1, 0, -v * dt * s], [0, 1, v * dt * c], [0, 0, 1]];
    const V = [[dt * c, (-v * dt * dt * s) / 2], [dt * s, (v * dt * dt * c) / 2], [0, dt]];
    const [vv, vw] = noiseVar(v, w, k);
    const M = [[vv + FLOOR.v * FLOOR.v, 0], [0, vw + FLOOR.w * FLOOR.w]];
    this.G = G;
    this.P = symmetrize(matAdd(matMul(matMul(G, this.P), transpose(G)), matMul(matMul(V, M), transpose(V))));
    this.Pbar = this.P;
  }

  /** Sequential update, one landmark at a time, with a Mahalanobis gate. */
  update(z, landmarks, sr, sb) {
    const R = [[sr * sr, 0], [0, sb * sb]];
    this.rejected = 0;
    for (const m of z) {
      const lm = landmarks[m.j];
      const [x, y, th] = this.mu;
      const dx = lm.x - x, dy = lm.y - y;
      const q = dx * dx + dy * dy;
      if (q < 1e-6) continue;
      const sq = Math.sqrt(q);
      const zhat = [sq, wrap(Math.atan2(dy, dx) - th)];
      const H = [[-dx / sq, -dy / sq, 0], [dy / q, -dx / q, -1]];
      const nu = [m.r - zhat[0], wrap(m.b - zhat[1])];
      const PHt = matMul(this.P, transpose(H));
      const S = matAdd(matMul(H, PHt), R);
      const Si = inv2(S);
      const d2 = nu[0] * (Si[0][0] * nu[0] + Si[0][1] * nu[1]) + nu[1] * (Si[1][0] * nu[0] + Si[1][1] * nu[1]);
      const accepted = d2 <= GATE;
      this.last = { j: m.j, z: [m.r, m.b], zhat, nu, H, S, d2, accepted, K: null };
      if (!accepted) {
        this.rejected++;
        continue;
      }
      const K = matMul(PHt, Si);
      this.mu = [x + K[0][0] * nu[0] + K[0][1] * nu[1], y + K[1][0] * nu[0] + K[1][1] * nu[1], wrap(th + K[2][0] * nu[0] + K[2][1] * nu[1])];
      // Joseph form: same result as (I - KH)P in exact arithmetic, but it
      // stays symmetric positive definite under rounding.
      const IKH = matAdd(eye3(), matMul(K, H).map((row) => row.map((v) => -v)));
      this.P = symmetrize(matAdd(matMul(matMul(IKH, this.P), transpose(IKH)), matMul(matMul(K, R), transpose(K))));
      this.last.K = K;
    }
  }

  estimate() {
    return { x: this.mu[0], y: this.mu[1], th: this.mu[2] };
  }

  /** Semi-axes and angle of the 95% position ellipse. */
  ellipse() {
    return ellipseOf(this.P);
  }
}

function symmetrize(P) {
  return P.map((row, i) => row.map((v, j) => (i === j ? v : (v + P[j][i]) / 2)));
}

/** Semi-axes and angle of the 95% ellipse of the x, y block of a covariance. */
export function ellipseOf(P) {
  const a = P[0][0], b = P[0][1], d = P[1][1];
  const mid = (a + d) / 2;
  const rad = Math.hypot((a - d) / 2, b);
  const l1 = mid + rad, l2 = Math.max(0, mid - rad);
  return { rx: Math.sqrt(ELLIPSE_K2 * l1), ry: Math.sqrt(ELLIPSE_K2 * l2), angle: 0.5 * Math.atan2(2 * b, a - d) };
}

/** Cholesky of a symmetric 3×3, or null if it is not positive definite. */
export function chol3(P) {
  const L = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j <= i; j++) {
      let s = P[i][j];
      for (let k = 0; k < j; k++) s -= L[i][k] * L[j][k];
      if (i === j) {
        if (!(s > 0)) return null;
        L[i][i] = Math.sqrt(s);
      } else {
        L[i][j] = s / L[j][j];
      }
    }
  }
  return L;
}

// ---- EKF-SLAM ----

/**
 * EKF-SLAM: an EKF whose state is the robot's pose plus every landmark it has
 * mapped so far, [x, y, θ, m1x, m1y, m2x, m2y, ...], with one covariance over
 * all of it. It is never given the map. Landmarks are identified (the
 * measurement says which one it is), so the hard part left is the
 * estimation: a landmark seen for the first time is placed where the
 * measurement says, inheriting the robot's uncertainty, and every later
 * sighting refines the robot and the whole map together through the
 * correlations in P. That is why seeing an old landmark again, a loop
 * closure, tightens landmarks that are not even in view.
 */
export class EKFSLAM {
  constructor() {
    this.init(START, 0.1, 0.1, 0.05);
  }

  /** Start at a pose. keepMap holds on to the landmarks and their own covariance. */
  init(pose, sx, sy, sth, keepMap = false) {
    const robotP = [[sx * sx, 0, 0], [0, sy * sy, 0], [0, 0, sth * sth]];
    if (keepMap && this.ids?.length) {
      this.mu = [pose.x, pose.y, pose.th, ...this.mu.slice(3)];
      const n = this.mu.length;
      const P = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => {
        if (i < 3 && j < 3) return robotP[i][j];
        if (i < 3 || j < 3) return 0; // the new pose guess knows nothing about the map
        return this.P[i][j];
      }));
      this.P = P;
    } else {
      this.mu = [pose.x, pose.y, pose.th];
      this.P = robotP;
      this.ids = []; // landmark index in the world, per map slot
    }
    this.last = null;
    this.rejected = 0;
    this.added = 0;
  }

  /** Same motion model as the EKF. Only the robot block moves; P_rm turns with it. */
  predict(v, w, dt, k) {
    const [x, y, th] = this.mu;
    const tm = th + (w * dt) / 2;
    const c = Math.cos(tm), s = Math.sin(tm);
    this.mu[0] = x + v * dt * c;
    this.mu[1] = y + v * dt * s;
    this.mu[2] = wrap(th + w * dt);
    const G = [[1, 0, -v * dt * s], [0, 1, v * dt * c], [0, 0, 1]];
    const V = [[dt * c, (-v * dt * dt * s) / 2], [dt * s, (v * dt * dt * c) / 2], [0, dt]];
    const [vv, vw] = noiseVar(v, w, k);
    const M = [[vv + FLOOR.v * FLOOR.v, 0], [0, vw + FLOOR.w * FLOOR.w]];
    const P = this.P, n = this.mu.length;
    const Prr = [0, 1, 2].map((i) => [0, 1, 2].map((j) => P[i][j]));
    const newRR = matAdd(matMul(matMul(G, Prr), transpose(G)), matMul(matMul(V, M), transpose(V)));
    // Robot-map cross terms: rows 0-2 become G times themselves.
    for (let j = 3; j < n; j++) {
      const col = [P[0][j], P[1][j], P[2][j]];
      for (let i = 0; i < 3; i++) {
        const v2 = G[i][0] * col[0] + G[i][1] * col[1] + G[i][2] * col[2];
        P[i][j] = v2;
        P[j][i] = v2;
      }
    }
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) P[i][j] = i === j ? newRR[i][j] : (newRR[i][j] + newRR[j][i]) / 2;
  }

  /** Map slot of world landmark j, or -1 if it has not been mapped. */
  slot(j) {
    return this.ids.indexOf(j);
  }

  /** Put a newly seen landmark into the state, where this measurement says it is. */
  addLandmark(j, r, b, R) {
    const [x, y, th] = this.mu;
    const a = th + b, c = Math.cos(a), s = Math.sin(a);
    const P = this.P, n = this.mu.length;
    // m = robot position + r in direction θ + b. Gr = ∂m/∂robot, Gz = ∂m/∂(r, b).
    const Gr = [[1, 0, -r * s], [0, 1, r * c]];
    const Gz = [[c, -r * s], [s, r * c]];
    // Covariance of the new landmark with everything already in the state.
    const cross = Array.from({ length: n }, (_, k) => [0, 1].map((i) => Gr[i][0] * P[0][k] + Gr[i][1] * P[1][k] + Gr[i][2] * P[2][k]));
    const Prr = [0, 1, 2].map((i) => [0, 1, 2].map((jj) => P[i][jj]));
    const Pmm = matAdd(matMul(matMul(Gr, Prr), transpose(Gr)), matMul(matMul(Gz, R), transpose(Gz)));
    for (let k = 0; k < n; k++) P[k].push(cross[k][0], cross[k][1]);
    P.push([...cross.map((cv) => cv[0]), Pmm[0][0], Pmm[0][1]]);
    P.push([...cross.map((cv) => cv[1]), Pmm[1][0], Pmm[1][1]]);
    this.mu.push(x + r * c, y + r * s);
    this.ids.push(j);
    this.added++;
  }

  /** Measurements carry the landmark's index; no map is passed in. */
  update(z, sr, sb) {
    const R = [[sr * sr, 0], [0, sb * sb]];
    this.rejected = 0;
    this.added = 0;
    for (const m of z) {
      let k = this.slot(m.j);
      if (k < 0) {
        this.addLandmark(m.j, m.r, m.b, R);
        continue;
      }
      const P = this.P, mu = this.mu, n = mu.length;
      const li = 3 + 2 * k;
      const dx = mu[li] - mu[0], dy = mu[li + 1] - mu[1];
      const q = dx * dx + dy * dy;
      if (q < 1e-6) continue;
      const sq = Math.sqrt(q);
      const zhat = [sq, wrap(Math.atan2(dy, dx) - mu[2])];
      // H is zero except for the robot block and this landmark's two columns.
      const cols = [0, 1, 2, li, li + 1];
      const Hs = [
        [-dx / sq, -dy / sq, 0, dx / sq, dy / sq],
        [dy / q, -dx / q, -1, -dy / q, dx / q],
      ];
      const PHt = Array.from({ length: n }, (_, i) => [0, 1].map((r) => cols.reduce((acc, c, t) => acc + P[i][c] * Hs[r][t], 0)));
      const S = [0, 1].map((r) => [0, 1].map((c2) => cols.reduce((acc, c, t) => acc + Hs[r][t] * PHt[c][c2], 0) + R[r][c2]));
      const Si = inv2(S);
      const nu = [m.r - zhat[0], wrap(m.b - zhat[1])];
      const d2 = nu[0] * (Si[0][0] * nu[0] + Si[0][1] * nu[1]) + nu[1] * (Si[1][0] * nu[0] + Si[1][1] * nu[1]);
      const accepted = d2 <= GATE;
      this.last = { j: m.j, nu, d2, accepted };
      if (!accepted) {
        this.rejected++;
        continue;
      }
      const K = PHt.map(([a, b]) => [a * Si[0][0] + b * Si[1][0], a * Si[0][1] + b * Si[1][1]]);
      for (let i = 0; i < n; i++) mu[i] += K[i][0] * nu[0] + K[i][1] * nu[1];
      mu[2] = wrap(mu[2]);
      // P ← P − K S Kᵀ, kept symmetric.
      const KS = K.map(([a, b]) => [a * S[0][0] + b * S[1][0], a * S[0][1] + b * S[1][1]]);
      for (let i = 0; i < n; i++) {
        for (let j2 = i; j2 < n; j2++) {
          const v2 = P[i][j2] - (KS[i][0] * K[j2][0] + KS[i][1] * K[j2][1]);
          P[i][j2] = v2;
          P[j2][i] = v2;
        }
      }
    }
  }

  /** Drop world landmark j from the map, and shift the indices above it down. */
  forget(j) {
    const k = this.slot(j);
    if (k >= 0) {
      const li = 3 + 2 * k;
      this.mu.splice(li, 2);
      this.P.splice(li, 2);
      for (const row of this.P) row.splice(li, 2);
      this.ids.splice(k, 1);
    }
    this.ids = this.ids.map((i) => (i > j ? i - 1 : i));
    if (this.last && this.last.j === j) this.last = null;
  }

  estimate() {
    return { x: this.mu[0], y: this.mu[1], th: this.mu[2] };
  }

  ellipse() {
    return ellipseOf(this.P);
  }

  /** The map as it stands: each landmark's estimate and 95% ellipse. */
  map() {
    return this.ids.map((j, k) => {
      const li = 3 + 2 * k;
      const Pm = [[this.P[li][li], this.P[li][li + 1]], [this.P[li + 1][li], this.P[li + 1][li + 1]]];
      return { j, x: this.mu[li], y: this.mu[li + 1], ...ellipseOf(Pm) };
    });
  }
}

// ---- Dead reckoning ----

/** Integrates the commanded motion and nothing else. */
export class DeadReckoning {
  constructor() {
    this.init(START);
  }

  init(pose) {
    this.pose = { ...pose };
  }

  predict(v, w, dt) {
    const p = this.pose;
    const tm = p.th + (w * dt) / 2;
    p.x += v * dt * Math.cos(tm);
    p.y += v * dt * Math.sin(tm);
    p.th = wrap(p.th + w * dt);
  }

  estimate() {
    return { ...this.pose };
  }
}

// ---- Unscented Kalman filter ----

// Scaled sigma points with α = 1, β = 2, κ = 1 for n = 3, so λ = 1. All
// seven weights are positive, which keeps the covariance positive definite
// without the corrections a negative center weight needs.
export const UKF_PARAMS = { alpha: 1, beta: 2, kappa: 1 };

export class UKF {
  constructor() {
    const { alpha, beta, kappa } = UKF_PARAMS;
    const n = 3;
    this.lambda = alpha * alpha * (n + kappa) - n;
    this.gamma = Math.sqrt(n + this.lambda);
    const wi = 1 / (2 * (n + this.lambda));
    this.wm = [this.lambda / (n + this.lambda), ...Array(2 * n).fill(wi)];
    this.wc = [this.wm[0] + 1 - alpha * alpha + beta, ...Array(2 * n).fill(wi)];
    this.init(START, 0.1, 0.1, 0.05);
  }

  init(pose, sx, sy, sth) {
    this.mu = [pose.x, pose.y, pose.th];
    this.P = [[sx * sx, 0, 0], [0, sy * sy, 0], [0, 0, sth * sth]];
    this.chi = this.sigmaPoints();
    this.last = null;
    this.rejected = 0;
  }

  /** μ, then μ ± γ times each column of the Cholesky factor of P. */
  sigmaPoints() {
    let L = chol3(this.P);
    if (!L) {
      // Rounding can leave P a hair from positive definite; nudge it back.
      this.P = this.P.map((row, i) => row.map((v, j) => (i === j ? Math.max(v, 0) + 1e-9 : v)));
      L = chol3(this.P) ?? [[1e-4, 0, 0], [0, 1e-4, 0], [0, 0, 1e-4]];
    }
    const pts = [[...this.mu]];
    for (const sign of [1, -1]) {
      for (let c = 0; c < 3; c++) {
        pts.push([0, 1, 2].map((r) => this.mu[r] + sign * this.gamma * L[r][c]));
      }
    }
    for (const p of pts) p[2] = wrap(p[2]);
    return pts;
  }

  /** Weighted mean with the heading averaged as a unit vector. */
  meanOf(pts) {
    let x = 0, y = 0, c = 0, s = 0;
    pts.forEach((p, i) => {
      x += this.wm[i] * p[0];
      y += this.wm[i] * p[1];
      c += this.wm[i] * Math.cos(p[2]);
      s += this.wm[i] * Math.sin(p[2]);
    });
    return [x, y, Math.atan2(s, c)];
  }

  /**
   * Each sigma point goes through the motion model exactly, no Jacobian.
   * Process noise is added the same way the EKF adds it, so the two differ
   * only in how they carry the existing uncertainty through g.
   */
  predict(v, w, dt, k) {
    const chi = this.sigmaPoints().map(([x, y, th]) => {
      const tm = th + (w * dt) / 2;
      return [x + v * dt * Math.cos(tm), y + v * dt * Math.sin(tm), wrap(th + w * dt)];
    });
    const mu = this.meanOf(chi);
    const P = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    chi.forEach((p, i) => {
      const d = [p[0] - mu[0], p[1] - mu[1], wrap(p[2] - mu[2])];
      for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) P[r][c] += this.wc[i] * d[r] * d[c];
    });
    const tm = mu[2] - (w * dt) / 2;
    const V = [[dt * Math.cos(tm), (-v * dt * dt * Math.sin(tm)) / 2], [dt * Math.sin(tm), (v * dt * dt * Math.cos(tm)) / 2], [0, dt]];
    const [vv, vw] = noiseVar(v, w, k);
    const M = [[vv + FLOOR.v * FLOOR.v, 0], [0, vw + FLOOR.w * FLOOR.w]];
    const Q = matMul(matMul(V, M), transpose(V));
    this.mu = mu;
    this.P = symmetrize(P.map((row, r) => row.map((x, c) => x + Q[r][c])));
    this.chi = chi;
  }

  /** Sequential update per landmark, gated like the EKF. */
  update(z, landmarks, sr, sb) {
    const R = [[sr * sr, 0], [0, sb * sb]];
    this.rejected = 0;
    for (const m of z) {
      const lm = landmarks[m.j];
      const chi = this.sigmaPoints();
      const Z = chi.map(([x, y, th]) => [Math.hypot(lm.x - x, lm.y - y), wrap(Math.atan2(lm.y - y, lm.x - x) - th)]);
      if (Z.some((zz) => zz[0] < 1e-3)) continue;
      let r = 0, c = 0, s = 0;
      Z.forEach((zz, i) => {
        r += this.wm[i] * zz[0];
        c += this.wm[i] * Math.cos(zz[1]);
        s += this.wm[i] * Math.sin(zz[1]);
      });
      const zhat = [r, Math.atan2(s, c)];
      const S = [[R[0][0], 0], [0, R[1][1]]];
      const Pxz = [[0, 0], [0, 0], [0, 0]];
      Z.forEach((zz, i) => {
        const dz = [zz[0] - zhat[0], wrap(zz[1] - zhat[1])];
        const dx = [chi[i][0] - this.mu[0], chi[i][1] - this.mu[1], wrap(chi[i][2] - this.mu[2])];
        for (let a = 0; a < 2; a++) for (let b = 0; b < 2; b++) S[a][b] += this.wc[i] * dz[a] * dz[b];
        for (let a = 0; a < 3; a++) for (let b = 0; b < 2; b++) Pxz[a][b] += this.wc[i] * dx[a] * dz[b];
      });
      const nu = [m.r - zhat[0], wrap(m.b - zhat[1])];
      const Si = inv2(S);
      const d2 = nu[0] * (Si[0][0] * nu[0] + Si[0][1] * nu[1]) + nu[1] * (Si[1][0] * nu[0] + Si[1][1] * nu[1]);
      const accepted = d2 <= GATE;
      this.last = { j: m.j, zhat, nu, S, d2, accepted, K: null };
      if (!accepted) {
        this.rejected++;
        continue;
      }
      const K = matMul(Pxz, Si);
      this.mu = [
        this.mu[0] + K[0][0] * nu[0] + K[0][1] * nu[1],
        this.mu[1] + K[1][0] * nu[0] + K[1][1] * nu[1],
        wrap(this.mu[2] + K[2][0] * nu[0] + K[2][1] * nu[1]),
      ];
      const KSKt = matMul(matMul(K, S), transpose(K));
      this.P = symmetrize(this.P.map((row, a) => row.map((x, b) => x - KSKt[a][b])));
      this.last.K = K;
    }
  }

  estimate() {
    return { x: this.mu[0], y: this.mu[1], th: this.mu[2] };
  }

  ellipse() {
    return ellipseOf(this.P);
  }
}

// ---- Grid (histogram) localization ----

export const GRID = {
  cell: 0.25, // m
  nt: 16, // heading bins, 22.5° each
  every: 4, // sensor steps per grid update: 5 Hz
  floor: 1e-3, // share of belief spread evenly after each update
};

/**
 * A Bayes filter over a 40 × 28 × 16 grid of poses. Odometry is summed over
 * four sensor steps, then the whole belief is shifted by that motion and
 * blurred by the motion noise. Shifts are split between neighboring cells in
 * proportion to the overlap, so a 6 cm move is not rounded away to nothing.
 */
export class GridFilter {
  constructor() {
    this.nx = Math.round(WORLD.w / GRID.cell);
    this.ny = Math.round(WORLD.h / GRID.cell);
    this.nt = GRID.nt;
    this.dth = (2 * Math.PI) / this.nt;
    this.N = this.nx * this.ny * this.nt;
    this.bel = new Float64Array(this.N);
    this.tmp = new Float64Array(this.N);
    this.ll = new Float64Array(this.N);
    this.wall = new Uint8Array(this.nx * this.ny); // 1 where the cell center is inside a wall
    this.heat = new Float32Array(this.nx * this.ny); // max over heading, for drawing
    this.free = this.nx * this.ny;
    this.updates = 0;
    this.blur = { xy: 0, th: 0 };
    this.lastMove = { x: 0, y: 0, th: 0 };
    this.clearAcc();
    this.initAt(START);
  }

  idx(i, j, k) {
    return (k * this.ny + j) * this.nx + i;
  }

  center(i, j) {
    return [(i + 0.5) * GRID.cell, (j + 0.5) * GRID.cell];
  }

  clearAcc() {
    this.acc = { x: 0, y: 0, th: 0, varPos: 0, varTh: 0, steps: 0 };
  }

  /** Cells whose center lies inside a wall can never hold the robot. */
  setWalls(walls) {
    let free = 0;
    for (let j = 0; j < this.ny; j++) {
      for (let i = 0; i < this.nx; i++) {
        const [cx, cy] = this.center(i, j);
        const inside = walls.some((w) => distToSegment(cx, cy, w) < WALL_T / 2);
        this.wall[j * this.nx + i] = inside ? 1 : 0;
        if (!inside) free++;
      }
    }
    this.free = free;
    this.applyMask();
    this.normalize();
  }

  applyMask() {
    const plane = this.nx * this.ny;
    for (let c = 0; c < plane; c++) {
      if (!this.wall[c]) continue;
      for (let k = 0; k < this.nt; k++) this.bel[k * plane + c] = 0;
    }
  }

  normalize() {
    let sum = 0;
    for (let n = 0; n < this.N; n++) sum += this.bel[n];
    if (!(sum > 0) || !Number.isFinite(sum)) {
      this.initUniform();
      return;
    }
    for (let n = 0; n < this.N; n++) this.bel[n] /= sum;
  }

  /** Mix in a little uniform belief so a lost filter can find the robot again. */
  addFloor() {
    const plane = this.nx * this.ny;
    const f = GRID.floor / (this.free * this.nt);
    for (let k = 0; k < this.nt; k++) {
      for (let c = 0; c < plane; c++) {
        const n = k * plane + c;
        this.bel[n] = this.wall[c] ? 0 : (1 - GRID.floor) * this.bel[n] + f;
      }
    }
  }

  initAt(pose) {
    this.bel.fill(0);
    this.splat(this.bel, pose.x / GRID.cell - 0.5, pose.y / GRID.cell - 0.5, wrap(pose.th) / this.dth, 1);
    this.applyMask();
    this.normalize();
    this.addFloor();
    this.clearAcc();
    this.finish();
  }

  initUniform() {
    this.bel.fill(1);
    this.applyMask();
    let sum = 0;
    for (let n = 0; n < this.N; n++) sum += this.bel[n];
    for (let n = 0; n < this.N; n++) this.bel[n] /= sum;
    this.clearAcc();
    this.finish();
  }

  /**
   * Adds mass m at fractional cell (fi, fj) and heading bin fk, split over
   * the 8 surrounding cells by overlap. Off-map or in-wall targets keep the
   * mass at the given fallback cell instead.
   */
  splat(arr, fi, fj, fk, m, fallback = -1) {
    const i0 = Math.floor(fi), j0 = Math.floor(fj), k0 = Math.floor(fk);
    const ax = fi - i0, ay = fj - j0, at = fk - k0;
    const plane = this.nx * this.ny;
    for (let dk = 0; dk < 2; dk++) {
      const wk = dk ? at : 1 - at;
      if (wk === 0) continue;
      const k = (((k0 + dk) % this.nt) + this.nt) % this.nt;
      for (let dj = 0; dj < 2; dj++) {
        const wj = dj ? ay : 1 - ay;
        if (wj === 0) continue;
        const j = j0 + dj;
        for (let di = 0; di < 2; di++) {
          const wi = di ? ax : 1 - ax;
          if (wi === 0) continue;
          const i = i0 + di;
          const part = m * wi * wj * wk;
          if (i < 0 || j < 0 || i >= this.nx || j >= this.ny || this.wall[j * this.nx + i]) {
            if (fallback >= 0) arr[fallback] += part;
          } else {
            arr[k * plane + j * this.nx + i] += part;
          }
        }
      }
    }
  }

  /** Sum one sensor step of commanded motion into the pending odometry. */
  accumulate(v, w, dt, kNoise) {
    const a = this.acc;
    const tm = a.th + (w * dt) / 2;
    a.x += v * dt * Math.cos(tm);
    a.y += v * dt * Math.sin(tm);
    a.th += w * dt;
    const [vv, vw] = noiseVar(v, w, kNoise);
    a.varPos += (vv + FLOOR.v * FLOOR.v) * dt * dt;
    a.varTh += (vw + FLOOR.w * FLOOR.w) * dt * dt;
    a.steps++;
  }

  /** Shift every cell by the pending odometry, as seen from its own heading, then blur. */
  predict() {
    const a = this.acc;
    const { nx, ny, nt } = this;
    const plane = nx * ny;
    const src = this.bel, dst = this.tmp;
    dst.fill(0);
    for (let k = 0; k < nt; k++) {
      const th = k * this.dth;
      const c = Math.cos(th), s = Math.sin(th);
      const sx = (c * a.x - s * a.y) / GRID.cell;
      const sy = (s * a.x + c * a.y) / GRID.cell;
      const fk = k + a.th / this.dth;
      for (let j = 0; j < ny; j++) {
        for (let i = 0; i < nx; i++) {
          const n = k * plane + j * nx + i;
          const m = src[n];
          if (m === 0) continue;
          // Blocked motion stays put: the wheels spin but the robot does not.
          this.splat(dst, i + sx, j + sy, fk, m, n);
        }
      }
    }
    // Noise blur: a [b, 1 - 2b, b] kernel has a variance of 2b cells².
    const bxy = Math.min(0.25, Math.max(0.02, a.varPos / (2 * GRID.cell * GRID.cell)));
    const bth = Math.min(0.25, Math.max(0.02, a.varTh / (2 * this.dth * this.dth)));
    this.blur = { xy: bxy, th: bth };
    this.blurAxis(dst, src, 1, nx, bxy, false);
    this.blurAxis(src, dst, nx, ny, bxy, false);
    this.blurAxis(dst, src, plane, nt, bth, true);
    this.lastMove = { x: a.x, y: a.y, th: a.th };
    this.clearAcc();
    this.applyMask();
    this.normalize();
  }

  /** One pass of the 3-tap blur along an axis with the given stride and length. */
  blurAxis(from, to, stride, len, b, circular) {
    const N = this.N;
    for (let n = 0; n < N; n++) {
      const pos = Math.floor(n / stride) % len;
      const prev = pos > 0 ? n - stride : circular ? n + (len - 1) * stride : -1;
      const next = pos < len - 1 ? n + stride : circular ? n - (len - 1) * stride : -1;
      // At a closed edge the mass that would leave stays in the cell.
      to[n] = from[n] * (1 - 2 * b) + b * ((prev >= 0 ? from[prev] : from[n]) + (next >= 0 ? from[next] : from[n]));
    }
  }

  /**
   * Same range and bearing likelihood as the particle filter, with the noise
   * widened by half a cell and half a heading bin: a cell center can be that
   * far from the true pose, and without the widening the cell that holds the
   * robot can score worse than its neighbors.
   */
  update(z, landmarks, sr, sb) {
    if (!z.length) return;
    const { nx, ny, nt, ll, bel } = this;
    const plane = nx * ny;
    const srg2 = sr * sr + (GRID.cell / 2) ** 2;
    const sbg2 = sb * sb + (this.dth / 2) ** 2;
    let max = -Infinity;
    for (let j = 0; j < ny; j++) {
      for (let i = 0; i < nx; i++) {
        const c = j * nx + i;
        if (this.wall[c]) continue;
        const [cx, cy] = this.center(i, j);
        for (let k = 0; k < nt; k++) ll[k * plane + c] = 0;
        for (const m of z) {
          const lm = landmarks[m.j];
          const dx = lm.x - cx, dy = lm.y - cy;
          const er = m.r - Math.hypot(dx, dy);
          const base = (er * er) / srg2;
          const ang = Math.atan2(dy, dx);
          for (let k = 0; k < nt; k++) {
            const eb = wrap(m.b - ang + k * this.dth);
            ll[k * plane + c] += base + (eb * eb) / sbg2;
          }
        }
        for (let k = 0; k < nt; k++) {
          const n = k * plane + c;
          ll[n] *= -0.5;
          if (ll[n] > max) max = ll[n];
        }
      }
    }
    // Subtract the best log-likelihood before exp. The floor keeps every free
    // cell above zero, so the best-scoring cell always keeps some mass.
    for (let k = 0; k < nt; k++) {
      for (let c = 0; c < plane; c++) {
        const n = k * plane + c;
        bel[n] = this.wall[c] ? 0 : bel[n] * Math.exp(ll[n] - max);
      }
    }
    this.normalize();
    this.addFloor();
    this.updates++;
  }

  /** Called once per sensor step; runs the grid every GRID.every steps. */
  step(v, w, dt, kNoise, z, landmarks, sr, sb) {
    this.accumulate(v, w, dt, kNoise);
    this.pendingZ = z;
    if (this.acc.steps < GRID.every) return false;
    this.predict();
    this.update(z, landmarks, sr, sb);
    this.finish();
    return true;
  }

  /** Mode, the estimate around it, and the heatmap. */
  finish() {
    const { nx, ny, nt, bel } = this;
    const plane = nx * ny;
    let best = 0;
    for (let n = 1; n < this.N; n++) if (bel[n] > bel[best]) best = n;
    const k0 = Math.floor(best / plane), c0 = best % plane;
    const i0 = c0 % nx, j0 = Math.floor(c0 / nx);
    // Weighted mean over the 5 × 5 × 5 block around the mode. The global mean
    // would land between two hypotheses when the belief has more than one.
    let sw = 0, sx = 0, sy = 0, sc = 0, ss = 0;
    for (let dk = -2; dk <= 2; dk++) {
      const k = (((k0 + dk) % nt) + nt) % nt;
      for (let j = Math.max(0, j0 - 2); j <= Math.min(ny - 1, j0 + 2); j++) {
        for (let i = Math.max(0, i0 - 2); i <= Math.min(nx - 1, i0 + 2); i++) {
          const m = bel[k * plane + j * nx + i];
          const [cx, cy] = this.center(i, j);
          sw += m;
          sx += m * cx;
          sy += m * cy;
          sc += m * Math.cos(k * this.dth);
          ss += m * Math.sin(k * this.dth);
        }
      }
    }
    this.mode = { i: i0, j: j0, k: k0, p: bel[best] };
    this.modeMass = sw;
    this.est = sw > 0 ? { x: sx / sw, y: sy / sw, th: Math.atan2(ss, sc) } : { x: WORLD.w / 2, y: WORLD.h / 2, th: 0 };
    let hmax = 0;
    for (let c = 0; c < plane; c++) {
      let m = 0;
      for (let k = 0; k < nt; k++) m = Math.max(m, bel[k * plane + c]);
      this.heat[c] = m;
      if (m > hmax) hmax = m;
    }
    this.heatMax = hmax;
  }

  estimate() {
    return { ...this.est };
  }
}

// ---- Walls ----

export const WALL_T = 0.15; // m, wall thickness
export const ROBOT_R = 0.18; // m
// Two short walls and one long one. From the left side of the loop the wall
// at x = 3.4 hides the center landmark, the long one on the right hides the
// lower right corner from the top of the loop and makes the planner route
// around its west end, and the top wall hides landmark 5 from the right.
export const DEFAULT_WALLS = [
  { ax: 3.4, ay: 2.6, bx: 3.4, by: 4.4 },
  { ax: 7.0, ay: 3.0, bx: 9.6, by: 3.0 },
  { ax: 6.2, ay: 5.7, bx: 6.2, by: 7.0 },
];

export function distToSegment(px, py, w) {
  const dx = w.bx - w.ax, dy = w.by - w.ay;
  const L2 = dx * dx + dy * dy;
  const t = L2 > 0 ? Math.max(0, Math.min(1, ((px - w.ax) * dx + (py - w.ay) * dy) / L2)) : 0;
  return Math.hypot(px - (w.ax + t * dx), py - (w.ay + t * dy));
}

/** Shortest distance between segment p0-p1 and a wall's center line. */
export function segmentToWall(x0, y0, x1, y1, w) {
  const cross = (ax, ay, bx, by) => ax * by - ay * bx;
  const rx = x1 - x0, ry = y1 - y0, sx = w.bx - w.ax, sy = w.by - w.ay;
  const den = cross(rx, ry, sx, sy);
  if (den !== 0) {
    const t = cross(w.ax - x0, w.ay - y0, sx, sy) / den;
    const u = cross(w.ax - x0, w.ay - y0, rx, ry) / den;
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return 0;
  }
  const seg = { ax: x0, ay: y0, bx: x1, by: y1 };
  return Math.min(distToSegment(x0, y0, w), distToSegment(x1, y1, w), distToSegment(w.ax, w.ay, seg), distToSegment(w.bx, w.by, seg));
}

// ---- Autopilot planning ----

export const PLAN_CELL = 0.25; // m
// Cells closer than this to a wall are off limits to the planner: the robot's
// radius, half the wall, and a little room for the noise in its steering.
const PLAN_CLEAR = ROBOT_R + WALL_T / 2 + 0.07;
const LOOKAHEAD = 0.45; // m, pure pursuit
const WAYPOINT_REACHED = 0.4; // m
const STALL_SECONDS = 5;

/** Occupancy grid for the planner, walls inflated by the robot. */
export function planGrid(walls) {
  const w = Math.round(WORLD.w / PLAN_CELL), h = Math.round(WORLD.h / PLAN_CELL);
  const cells = new Uint8Array(w * h);
  for (let j = 0; j < h; j++) {
    for (let i = 0; i < w; i++) {
      const x = (i + 0.5) * PLAN_CELL, y = (j + 0.5) * PLAN_CELL;
      const edge = Math.min(x, y, WORLD.w - x, WORLD.h - y) < ROBOT_R + 0.05;
      if (edge || walls.some((wl) => distToSegment(x, y, wl) < PLAN_CLEAR)) cells[j * w + i] = 1;
    }
  }
  return { w, h, cells };
}

/** The free cell nearest a point, searching outward up to maxRing rings. */
function nearestFree(grid, x, y, maxRing = 6) {
  const ci = Math.floor(x / PLAN_CELL), cj = Math.floor(y / PLAN_CELL);
  let best = null, bestD = Infinity;
  for (let ring = 0; ring <= maxRing; ring++) {
    for (let j = cj - ring; j <= cj + ring; j++) {
      for (let i = ci - ring; i <= ci + ring; i++) {
        if (Math.max(Math.abs(i - ci), Math.abs(j - cj)) !== ring) continue;
        if (i < 0 || j < 0 || i >= grid.w || j >= grid.h || grid.cells[j * grid.w + i]) continue;
        const d = Math.hypot((i + 0.5) * PLAN_CELL - x, (j + 0.5) * PLAN_CELL - y);
        if (d < bestD) {
          bestD = d;
          best = { x: i, y: j };
        }
      }
    }
    if (best) return best;
  }
  return null;
}

/** A* between two world points; returns world waypoints or null. */
export function planPath(grid, from, to) {
  const s = nearestFree(grid, from.x, from.y);
  const g = nearestFree(grid, to.x, to.y, 2);
  if (!s || !g) return null;
  const res = run(gridSearch(grid, s, g, { algo: "astar", conn: 8 }));
  if (!res.found) return null;
  const path = res.path.map((p) => ({ x: p.x * PLAN_CELL, y: p.y * PLAN_CELL }));
  path.push({ x: to.x, y: to.y });
  return path;
}

// ---- The whole simulation ----

export const ALGO_KEYS = ["dr", "pf", "ekf", "ukf", "grid", "slam"];

export class Localization {
  constructor({ seed = 1, particles = 1000, walls = DEFAULT_WALLS } = {}) {
    this.rng = mulberry32(seed);
    this.gauss = makeGauss(this.rng);
    this.params = { odomNoise: 1, sensorNoise: 0.1, maxRange: 6, wind: 0, windDir: 0, gusty: false };
    this.landmarks = DEFAULT_LANDMARKS.map((l) => ({ ...l }));
    this.walls = walls.map((w) => ({ ...w }));
    this.autopilot = true;
    this.manual = { v: 0, w: 0 };
    this.dr = new DeadReckoning();
    this.pf = new ParticleFilter(particles, this.rng, this.gauss);
    this.ekf = new EKF();
    this.ukf = new UKF();
    this.grid = new GridFilter();
    this.slam = new EKFSLAM();
    this.wallsChanged();
    this.reset();
  }

  /** Back to the start pose, every filter told exactly where it is. */
  reset() {
    this.t = 0;
    this.acc = 0;
    this.gust = 1;
    this.truth = { ...START };
    this.u = { v: 0, w: 0 };
    this.dr.init(START);
    this.pf.initGaussian(START, 0.1, 0.05);
    this.ekf.init(START, 0.1, 0.1, 0.05);
    this.ukf.init(START, 0.1, 0.1, 0.05);
    this.grid.initAt(START);
    // SLAM starts with an empty map. Its known start pose is what ties the
    // map it builds to the world's coordinates.
    this.slam.init(START, 0.1, 0.1, 0.05);
    this.history = [];
    this.trail = [];
    this.breakTrail = true;
    this.bumpT = -Infinity;
    this.bumps = 0;
    this.wp = 1;
    this.visits = 0;
    this.path = null;
    // A reading for display only, so a paused start shows the sensor rays
    // and landmark count. The filters already know the start pose.
    this.z = this.sense();
  }

  get sigma() {
    const r = this.params.sensorNoise;
    return { r, b: r * BEARING_PER_RANGE };
  }

  /**
   * Forget everything. PF and grid spread over the whole map; EKF and UKF
   * start from a broad Gaussian at the map center; dead reckoning restarts
   * from that same center guess, since it has nothing better.
   */
  resetFilters() {
    const c = { x: WORLD.w / 2, y: WORLD.h / 2, th: 0 };
    this.dr.init(c);
    this.pf.initUniform();
    this.ekf.init(c, 2.5, 2, Math.PI / 2);
    this.ukf.init(c, 2.5, 2, Math.PI / 2);
    this.grid.initUniform();
    // SLAM keeps the map it built and forgets only where it is.
    this.slam.init(c, 2.5, 2, Math.PI / 2, true);
  }

  /**
   * The wind, in meters per second, as a vector. Nothing tells the filters
   * about it: the odometry they are given is still the commanded motion, so
   * the push shows up only as the robot ending up somewhere else.
   */
  windVector() {
    const g = this.params.gusty ? this.gust : 1;
    const v = this.params.wind * g;
    return { x: v * Math.cos(this.params.windDir), y: v * Math.sin(this.params.windDir), gust: g };
  }

  /**
   * Gusts: a slow pull back toward the average with a random kick, so the
   * strength wanders between a third and about twice the setting. Seeded from
   * the same generator as everything else, so runs still repeat.
   */
  stepGust() {
    if (!this.params.gusty) {
      this.gust = 1;
      return;
    }
    this.gust += (1 - this.gust) * 0.4 * DT + 1.1 * Math.sqrt(DT) * this.gauss();
    this.gust = Math.min(2.2, Math.max(0.3, this.gust));
  }

  /** Free for the robot's body: inside the map and clear of every wall. */
  poseFree(x, y, margin = 0) {
    const r = ROBOT_R + margin;
    if (x < r || y < r || x > WORLD.w - r || y > WORLD.h - r) return false;
    return this.walls.every((w) => distToSegment(x, y, w) >= r + WALL_T / 2);
  }

  /** Teleport the robot somewhere clear, at least 3 m away. No filter is told. */
  kidnap() {
    const old = this.truth;
    let x = old.x, y = old.y;
    for (let tries = 0; tries < 500; tries++) {
      const cx = 1 + this.rng() * (WORLD.w - 2);
      const cy = 1 + this.rng() * (WORLD.h - 2);
      if (!this.poseFree(cx, cy, 0.1)) continue;
      x = cx;
      y = cy;
      if (Math.hypot(cx - old.x, cy - old.y) >= 3) break;
    }
    this.truth = { x, y, th: (this.rng() * 2 - 1) * Math.PI };
    this.breakTrail = true;
    // Pick up the loop at the closest waypoint instead of crossing the map.
    let best = 0;
    WAYPOINTS.forEach(([wx, wy], i) => {
      if (Math.hypot(wx - x, wy - y) < Math.hypot(WAYPOINTS[best][0] - x, WAYPOINTS[best][1] - y)) best = i;
    });
    this.wp = best;
    this.path = null;
  }

  /** Returns false when the spot is inside a wall. */
  addLandmark(x, y) {
    if (this.walls.some((w) => distToSegment(x, y, w) < WALL_T / 2 + 0.08)) return false;
    this.landmarks.push({ x, y });
    this.resense();
    return true;
  }

  removeLandmark(j) {
    this.landmarks.splice(j, 1);
    this.slam.forget(j);
    this.resense();
    this.z = [];
    for (const f of [this.ekf, this.ukf]) if (f.last && f.last.j >= this.landmarks.length) f.last = null;
  }

  /** Returns false if the wall would run through the robot. */
  addWall(ax, ay, bx, by) {
    const w = { ax, ay, bx, by };
    if (distToSegment(this.truth.x, this.truth.y, w) < ROBOT_R + WALL_T / 2 + 0.02) return false;
    this.walls.push(w);
    this.wallsChanged();
    return true;
  }

  removeWall(i) {
    this.walls.splice(i, 1);
    this.wallsChanged();
  }

  clearWalls() {
    this.walls = [];
    this.wallsChanged();
  }

  /** Put the default walls back, for Start over. */
  restoreWalls() {
    this.walls = DEFAULT_WALLS.map((w) => ({ ...w }));
    this.wallsChanged();
  }

  wallsChanged() {
    this.planGrid = planGrid(this.walls);
    this.grid.setWalls(this.walls);
    this.path = null;
    this.resense();
  }

  /**
   * Retake the display reading after the map changes, so what is in view and
   * what is blocked is right even while paused. The filters are not updated;
   * the next step does that with a fresh reading anyway.
   */
  resense() {
    if (this.truth) this.z = this.sense();
  }

  /** Plan from the robot to the current waypoint, skipping any it cannot reach. */
  plan() {
    for (let tries = 0; tries < WAYPOINTS.length; tries++) {
      const [x, y] = WAYPOINTS[this.wp];
      const path = planPath(this.planGrid, this.truth, { x, y });
      if (path) {
        this.path = path;
        this.pathIdx = 0;
        this.bestDist = Infinity;
        this.bestT = this.t;
        return true;
      }
      this.wp = (this.wp + 1) % WAYPOINTS.length;
    }
    this.path = null;
    return false;
  }

  nextWaypoint() {
    this.wp = (this.wp + 1) % WAYPOINTS.length;
    this.path = null;
  }

  /**
   * The autopilot is a scripted driver, not part of localization, so it
   * steers by the true pose. It follows an A* path with a pure pursuit style
   * controller: aim at the first path point at least LOOKAHEAD away.
   */
  pilot() {
    const tr = this.truth;
    const [gx, gy] = WAYPOINTS[this.wp];
    if (Math.hypot(gx - tr.x, gy - tr.y) < WAYPOINT_REACHED) {
      this.visits++;
      this.nextWaypoint();
    }
    if (!this.path && !this.plan()) return { v: 0, w: 0 };

    // Watchdog: if the waypoint gets no closer for a while, give up on it.
    const d = Math.hypot(WAYPOINTS[this.wp][0] - tr.x, WAYPOINTS[this.wp][1] - tr.y);
    if (d < this.bestDist - 0.1) {
      this.bestDist = d;
      this.bestT = this.t;
    } else if (this.t - this.bestT > STALL_SECONDS) {
      this.nextWaypoint();
      if (!this.plan()) return { v: 0, w: 0 };
    }

    const path = this.path;
    // Replan if slip or a bump pushed the robot well off the path.
    let near = Infinity;
    for (let i = this.pathIdx; i < Math.min(path.length, this.pathIdx + 6); i++) {
      near = Math.min(near, Math.hypot(path[i].x - tr.x, path[i].y - tr.y));
    }
    if (near > 0.8 && !this.plan()) return { v: 0, w: 0 };
    while (this.pathIdx < this.path.length - 1 && Math.hypot(this.path[this.pathIdx].x - tr.x, this.path[this.pathIdx].y - tr.y) < LOOKAHEAD) {
      this.pathIdx++;
    }
    const target = this.path[this.pathIdx];
    const e = wrap(Math.atan2(target.y - tr.y, target.x - tr.x) - tr.th);
    // Turn in place when facing well away, so it never sweeps into a wall.
    return { v: 0.7 * Math.max(0, Math.cos(e)) ** 2, w: Math.max(-1.5, Math.min(1.5, 2.5 * e)) };
  }

  /**
   * Moves the true robot. If the full move would hit a wall it tries each
   * axis alone, so it slides along the wall, and otherwise stays put. The
   * filters still get the commanded motion: to them a bump is wheel slip.
   */
  moveTruth(dx, dy) {
    const tr = this.truth;
    if (this.poseFree(tr.x + dx, tr.y + dy)) {
      tr.x += dx;
      tr.y += dy;
      return false;
    }
    if (this.poseFree(tr.x + dx, tr.y)) tr.x += dx;
    else if (this.poseFree(tr.x, tr.y + dy)) tr.y += dy;
    return true;
  }

  step() {
    const u = this.autopilot ? this.pilot() : { ...this.manual };
    this.u = u;
    const k = this.params.odomNoise;

    // The wheels execute the command with slip; the filters only get the command.
    const [vv, vw] = noiseVar(u.v, u.w, k);
    const vn = u.v + Math.sqrt(vv) * this.gauss();
    const wn = u.w + Math.sqrt(vw) * this.gauss();
    const tr = this.truth;
    const tm = tr.th + (wn * DT) / 2;
    if (this.moveTruth(vn * DT * Math.cos(tm), vn * DT * Math.sin(tm)) && Math.abs(u.v) > 1e-6) {
      this.bumpT = this.t;
      this.bumps++;
    }
    tr.th = wrap(tr.th + wn * DT);

    // Wind pushes the body after the wheels have had their say. Blowing into
    // a wall is not a bump: the counter is for the robot driving into things.
    this.stepGust();
    const air = this.windVector();
    if (air.x || air.y) this.moveTruth(air.x * DT, air.y * DT);

    this.dr.predict(u.v, u.w, DT);
    this.pf.predict(u.v, u.w, DT, k);
    this.ekf.predict(u.v, u.w, DT, k);
    this.ukf.predict(u.v, u.w, DT, k);
    this.slam.predict(u.v, u.w, DT, k);

    const { r: sr, b: sb } = this.sigma;
    const z = this.sense();
    this.z = z;
    this.pf.update(z, this.landmarks, sr, sb);
    this.ekf.update(z, this.landmarks, sr, sb);
    this.ukf.update(z, this.landmarks, sr, sb);
    this.slam.update(z, sr, sb);
    this.grid.step(u.v, u.w, DT, k, z, this.landmarks, sr, sb);

    this.t += DT;
    this.record();
  }

  /** True when the straight line between two points crosses no wall. */
  lineOfSight(x0, y0, x1, y1) {
    return this.walls.every((w) => segmentToWall(x0, y0, x1, y1, w) >= WALL_T / 2);
  }

  /**
   * Noisy range and bearing to every landmark in range of the true pose and
   * not hidden behind a wall. Landmarks in range but hidden go in blocked.
   */
  sense() {
    const { r: sr, b: sb } = this.sigma;
    const z = [];
    this.blocked = [];
    this.inRange = 0;
    const tr = this.truth;
    this.landmarks.forEach((lm, j) => {
      const h = expectedMeasurement(tr, lm);
      if (h.r > this.params.maxRange) return;
      this.inRange++;
      if (!this.lineOfSight(tr.x, tr.y, lm.x, lm.y)) {
        this.blocked.push(j);
        return;
      }
      z.push({ j, r: Math.max(0.01, h.r + sr * this.gauss()), b: wrap(h.b + sb * this.gauss()) });
    });
    return z;
  }

  estimates() {
    return {
      dr: this.dr.estimate(), pf: this.pf.estimate(), ekf: this.ekf.estimate(),
      ukf: this.ukf.estimate(), grid: this.grid.estimate(), slam: this.slam.estimate(),
    };
  }

  record() {
    const est = this.estimates();
    const tr = this.truth;
    const h = { t: this.t };
    const p = { t: this.t, x: tr.x, y: tr.y, brk: this.breakTrail };
    for (const key of ALGO_KEYS) {
      const e = est[key];
      h[key] = Math.hypot(e.x - tr.x, e.y - tr.y);
      p[key] = [e.x, e.y];
    }
    this.history.push(h);
    this.trail.push(p);
    this.breakTrail = false;
    prune(this.history, this.t - HISTORY_SECONDS);
    prune(this.trail, this.t - TRAIL_SECONDS);
  }

  /** Run as many fixed steps as fit, carrying the remainder to the next call. */
  advance(seconds) {
    this.acc = Math.min(this.acc + seconds, 0.5);
    while (this.acc >= DT) {
      this.acc -= DT;
      this.step();
    }
  }

  errors() {
    const tr = this.truth;
    const est = this.estimates();
    const out = {};
    for (const key of ALGO_KEYS) {
      const e = est[key];
      out[key] = { pos: Math.hypot(e.x - tr.x, e.y - tr.y), head: Math.abs(wrap(e.th - tr.th)) };
    }
    return out;
  }
}

function prune(list, cutoff) {
  let drop = 0;
  while (drop < list.length && list[drop].t < cutoff) drop++;
  if (drop > 64) list.splice(0, drop);
}
