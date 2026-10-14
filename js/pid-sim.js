// PID controller and three plants. Pure JS, no DOM, so presets can be checked
// under node --test.

const g = 9.81;
const DEG = Math.PI / 180;

/**
 * Each system: physics in SI units, plus what the UI needs to label it.
 * `display` converts the measured value to the unit shown on screen.
 */
export const SYSTEMS = {
  drone: {
    name: "Drone altitude",
    short: "Drone",
    blurb: "Thrust lifts a 1 kg drone against gravity. Motors can push up but not pull down, and hovering needs steady thrust, so without Ki it sags below the target.",
    blurbOpt: "Thrust lifts a 1 kg drone against gravity. Motors can push up but not pull down. The model says hovering takes 9.81 N, so it holds the target without an integral term.",
    unit: "m",
    display: (v) => v,
    fromDisplay: (v) => v,
    setpoint: { min: 0.5, max: 9, default: 5, step: 0.1 },
    start: 1,
    uLimit: [0, 25],
    uUnit: "N",
    noise: 0.03,
    band: 0.05,
    gains: { kp: [0, 40], ki: [0, 20], kd: [0, 20] },
    presets: {
      tuned: { kp: 12, ki: 6, kd: 5 },
      overshoot: { kp: 20, ki: 6, kd: 2.5 },
      oscillating: { kp: 30, ki: 2, kd: 0.3 },
      sluggish: { kp: 3, ki: 0.4, kd: 4 },
      "no Ki": { kp: 12, ki: 0, kd: 6 },
    },
    init(y) {
      return { y, v: 0 };
    },
    measure: (s) => s.y,
    measureKey: "y",
    deriv(s, u) {
      const m = 1;
      return { y: s.v, v: u / m - g - 0.15 * s.v };
    },
    constrain(s) {
      if (s.y <= 0) {
        s.y = 0;
        if (s.v < 0) s.v = 0;
      }
      if (s.y > 11) {
        s.y = 11;
        if (s.v > 0) s.v = 0;
      }
    },
    kick(s) {
      s.v -= 3;
    },
  },

  arm: {
    name: "Robot arm",
    short: "Robot arm",
    blurb: "A motor swings a 1 m link with a 1 kg load. Gravity pulls hardest when the arm is level, so holding an angle off vertical takes constant torque.",
    unit: "°",
    display: (v) => v / DEG,
    fromDisplay: (v) => v * DEG,
    setpoint: { min: -150, max: 150, default: 75, step: 1 },
    start: 0,
    uLimit: [-14, 14],
    uUnit: "N·m",
    noise: 0.6 * DEG,
    band: 1 * DEG,
    gains: { kp: [0, 120], ki: [0, 80], kd: [0, 30] },
    presets: {
      tuned: { kp: 45, ki: 25, kd: 9 },
      overshoot: { kp: 20, ki: 60, kd: 4 },
      oscillating: { kp: 110, ki: 10, kd: 0.4 },
      sluggish: { kp: 12, ki: 3, kd: 10 },
      "no Ki": { kp: 45, ki: 0, kd: 9 },
    },
    init(th) {
      return { th, w: 0 };
    },
    measure: (s) => s.th,
    measureKey: "th",
    // Torque gravity applies about the pivot, same sign convention as the
    // motor (positive turns toward larger angles). Shared with the drawing.
    gravityTorque: (s) => -1 * g * 1 * Math.sin(s.th),
    deriv(s, u) {
      const inertia = 1 * 1 * 1 + 0.05, b = 0.25;
      return { th: s.w, w: (u + SYSTEMS.arm.gravityTorque(s) - b * s.w) / inertia };
    },
    constrain() {},
    kick(s) {
      s.w += 4;
    },
  },

  cartpole: {
    name: "Inverted pendulum",
    short: "Pendulum",
    blurb: "Push a cart left and right to keep a pole balanced upright. The target is fixed at 0°. Kp has to beat gravity (about 12 N per radian here) before anything else works.",
    blurbOpt: "Push a cart left and right to get a pole upright and keep it there. The target is fixed at 0°, straight up.",
    unit: "°",
    display: (v) => v / DEG,
    fromDisplay: (v) => v * DEG,
    setpoint: { min: 0, max: 0, default: 0, step: 1 },
    start: 6 * DEG,
    // MPC and iLQR get a free pivot and start hanging straight down, so they
    // have to swing the pole up before they can balance it.
    startFree: Math.PI,
    uLimit: [-40, 40],
    uUnit: "N",
    noise: 0.3 * DEG,
    band: 1 * DEG,
    // The force has to push toward the lean, so the controller output is
    // negated: error is setpoint - angle, and a positive lean needs +F.
    sign: -1,
    gains: { kp: [0, 200], ki: [0, 60], kd: [0, 40] },
    presets: {
      tuned: { kp: 70, ki: 5, kd: 12 },
      overshoot: { kp: 40, ki: 20, kd: 3 },
      oscillating: { kp: 140, ki: 0, kd: 0.2 },
      sluggish: { kp: 18, ki: 0, kd: 14 },
      "too weak": { kp: 10, ki: 0, kd: 5 },
    },
    init(th) {
      return { x: 0, v: 0, th, w: 0 };
    },
    measure: (s) => s.th,
    measureKey: "th",
    deriv(s, u) {
      // Point mass on a massless rod, frictionless cart.
      const M = 1, m = 0.2, l = 1;
      const sin = Math.sin(s.th), cos = Math.cos(s.th);
      const temp = (u + m * l * s.w * s.w * sin) / (M + m);
      const alpha = (g * sin - cos * temp) / (l * (1 - (m * cos * cos) / (M + m)));
      const acc = temp - (m * l * alpha * cos) / (M + m);
      return { x: s.v, v: acc - 0.1 * s.v, th: s.w, w: alpha - 0.02 * s.w };
    },
    constrain(s, free = false) {
      // The pole lies on the floor once it falls past horizontal, unless the
      // pivot is free (MPC and iLQR), when it swings all the way round.
      if (free) return;
      const lim = 95 * DEG;
      if (Math.abs(s.th) > lim) {
        s.th = Math.sign(s.th) * lim;
        s.w = 0;
      }
    },
    kick(s) {
      s.w += 0.9;
    },
  },
};

export class PID {
  constructor({ kp = 0, ki = 0, kd = 0 } = {}) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.antiWindup = true;
    this.derivOnMeasurement = true;
    this.dFilter = 0.02; // seconds, low-pass on the D term so noise does not saturate it
    this.reset();
  }

  reset() {
    this.integral = 0;
    this.prevError = null;
    this.prevMeas = null;
    this.dState = 0;
  }

  update(sp, meas, dt, [lo, hi], sign = 1) {
    const e = sp - meas;
    const p = this.kp * e;

    // Derivative on measurement skips the spike a setpoint step would cause.
    let raw = 0;
    if (this.derivOnMeasurement) {
      if (this.prevMeas !== null) raw = -(meas - this.prevMeas) / dt;
    } else if (this.prevError !== null) {
      raw = (e - this.prevError) / dt;
    }
    const a = dt / (this.dFilter + dt);
    this.dState += a * (raw - this.dState);
    const d = this.kd * this.dState;

    const trialIntegral = this.integral + e * dt;
    let i = this.ki * trialIntegral;
    let out = sign * (p + i + d);
    const saturated = out > hi || out < lo;

    // Conditional integration: hold the integral while the actuator is
    // pinned and the error would push it further in.
    const pushingIn = (out > hi && sign * e > 0) || (out < lo && sign * e < 0);
    if (this.antiWindup && saturated && pushingIn) {
      i = this.ki * this.integral;
      out = sign * (p + i + d);
    } else {
      this.integral = trialIntegral;
    }

    this.prevError = e;
    this.prevMeas = meas;
    const u = Math.min(hi, Math.max(lo, out));
    return { u, raw: out, p, i, d, e, saturated: u !== out };
  }
}

// Box-Muller, for sensor noise.
function gauss() {
  const u = 1 - Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * Math.random());
}

export const CONTROL_DT = 1 / 200;
const SUBSTEPS = 4;
export const HISTORY_SECONDS = 12;

export class Simulation {
  constructor(key, gains) {
    this.pid = new PID(gains);
    this.noise = false;
    this.mode = "pid";
    this.opt = null; // an LQR, MPC or iLQR controller from optctl.js when mode is not "pid"
    this.load(key);
  }

  /** Switch between PID and an optimizing controller built for this system. */
  setMode(mode, controller = null) {
    this.mode = mode;
    this.opt = controller;
    this.reset();
  }

  /** MPC and iLQR get a free pivot and swing up; PID and LQR start near the goal. */
  get free() {
    return this.mode === "mpc" || this.mode === "ilqr";
  }

  load(key) {
    this.key = key;
    this.sys = SYSTEMS[key];
    this.reset();
  }

  reset() {
    this.t = 0;
    this.sp = this.sys.fromDisplay(this.sys.setpoint.default);
    this.state = this.sys.init(this.free && this.sys.startFree !== undefined ? this.sys.startFree : this.sys.start);
    this.pid.reset();
    this.opt?.reset();
    this.history = [];
    this.last = { u: 0, e: 0, p: 0, i: 0, d: 0, saturated: false };
    this.markEvent();
  }

  setSetpoint(sp) {
    if (sp === this.sp) return;
    this.sp = sp;
    this.markEvent();
  }

  kick() {
    this.sys.kick(this.state);
    this.opt?.disturbed?.();
    this.markEvent();
  }

  /** Start a new measurement window for the response metrics. */
  markEvent() {
    const y = this.sys.measure(this.state);
    this.event = { t0: this.t, y0: y, r: this.sp, peak: y, lastOutside: this.t, t10: null, t90: null };
  }

  /** Advance by one controller period. */
  step() {
    const sys = this.sys;
    const noise = this.noise ? gauss() * sys.noise : 0;
    if (this.mode === "pid") {
      const meas = sys.measure(this.state) + noise;
      this.last = this.pid.update(this.sp, meas, CONTROL_DT, sys.uLimit, sys.sign ?? 1);
    } else {
      // The optimizers use the whole state; noise lands on the measured part.
      const seen = { ...this.state, [sys.measureKey]: this.state[sys.measureKey] + noise };
      const { u, raw } = this.opt.update(this.opt.vec(seen), this.sp, CONTROL_DT);
      this.last = { u, raw, e: this.sp - sys.measure(this.state), p: 0, i: 0, d: 0, saturated: u !== raw };
    }

    const h = CONTROL_DT / SUBSTEPS;
    for (let k = 0; k < SUBSTEPS; k++) {
      // Semi-implicit Euler: velocities first, then positions with the new
      // velocities. Stable for these stiffnesses at this step size.
      const d = sys.deriv(this.state, this.last.u);
      for (const key of Object.keys(d)) {
        if (key === "v" || key === "w") this.state[key] += d[key] * h;
      }
      const d2 = sys.deriv(this.state, this.last.u);
      for (const key of Object.keys(d2)) {
        if (key !== "v" && key !== "w") this.state[key] += d2[key] * h;
      }
      sys.constrain(this.state, this.free);
    }
    this.t += CONTROL_DT;

    const y = sys.measure(this.state);
    // Angles on a free pivot are shown wrapped, so a pole that has swung up
    // through 180° reads as near 0°, not 360°.
    const yShown = this.free && sys.measureKey === "th" ? this.sp + Math.atan2(Math.sin(y - this.sp), Math.cos(y - this.sp)) : y;
    this.trackMetrics(yShown);
    this.history.push({ t: this.t, sp: this.sp, y: yShown, e: this.sp - yShown, u: this.last.u, sat: this.last.saturated });
    const cutoff = this.t - HISTORY_SECONDS;
    let drop = 0;
    while (drop < this.history.length && this.history[drop].t < cutoff) drop++;
    if (drop > 64) this.history.splice(0, drop);
  }

  /** Run as many controller steps as fit in `seconds` of simulated time. */
  advance(seconds) {
    const n = Math.min(400, Math.round(seconds / CONTROL_DT));
    for (let k = 0; k < n; k++) this.step();
  }

  trackMetrics(y) {
    const ev = this.event;
    const step = ev.r - ev.y0;
    const dir = Math.sign(step) || 1;
    if ((y - ev.peak) * dir > 0) ev.peak = y;
    const frac = step === 0 ? null : (y - ev.y0) / step;
    if (frac !== null) {
      if (ev.t10 === null && frac >= 0.1) ev.t10 = this.t;
      if (ev.t90 === null && frac >= 0.9) ev.t90 = this.t;
    }
    const band = Math.max(this.sys.band, Math.abs(step) * 0.02);
    if (Math.abs(ev.r - y) > band) ev.lastOutside = this.t;
  }

  /** Response metrics since the last setpoint change or kick. */
  metrics() {
    const ev = this.event;
    const step = ev.r - ev.y0;
    const y = this.sys.measure(this.state);
    const settledFor = this.t - ev.lastOutside;
    const settled = settledFor >= 1;
    const big = Math.abs(step) > this.sys.band * 2;
    return {
      riseTime: big && ev.t10 !== null && ev.t90 !== null ? ev.t90 - ev.t10 : null,
      overshoot: big ? Math.max(0, ((ev.peak - ev.r) * Math.sign(step)) / Math.abs(step)) * 100 : null,
      settlingTime: settled ? ev.lastOutside - ev.t0 : null,
      steadyError: settled ? ev.r - y : null,
      elapsed: this.t - ev.t0,
    };
  }
}

/**
 * Closed-loop characteristic polynomial, linearized about the setpoint and
 * ignoring actuator limits, the D filter and sampling. Coefficients are in
 * descending powers of s. With Ki = 0 the shared factor of s is dropped.
 *
 *   drone:     m s³ + (c + Kd) s² + Kp s + Ki
 *   arm:       J s³ + (b + Kd) s² + (Kp + m g L cos θ₀) s + Ki
 *   pendulum:  M l s³ + (Kd + 0.02 M l) s² + (Kp − (M + m) g) s + Ki
 */
export function charPoly(key, { kp, ki, kd }, sp) {
  let c;
  if (key === "drone") c = [1, 0.15 + kd, kp, ki];
  else if (key === "arm") c = [1.05, 0.25 + kd, kp + g * Math.cos(sp), ki];
  else c = [1, kd + 0.02, kp - 1.2 * g, ki];
  return ki === 0 ? c.slice(0, 3) : c;
}

/** Roots of a real polynomial (descending coefficients), Durand-Kerner. */
export function polyRoots(coeffs) {
  const n = coeffs.length - 1;
  const a = coeffs.map((v) => v / coeffs[0]);
  const mul = (x, y) => [x[0] * y[0] - x[1] * y[1], x[0] * y[1] + x[1] * y[0]];
  const div = (x, y) => {
    const d = y[0] * y[0] + y[1] * y[1] || 1e-300;
    return [(x[0] * y[0] + x[1] * y[1]) / d, (x[1] * y[0] - x[0] * y[1]) / d];
  };
  const evalP = (z) => {
    let r = [1, 0];
    for (let k = 1; k <= n; k++) r = [mul(r, z)[0] + a[k], mul(r, z)[1]];
    return r;
  };
  let z = Array.from({ length: n }, (_, k) => {
    const ang = (2 * Math.PI * k) / n + 0.4;
    return [0.9 * Math.cos(ang) * 2, 0.9 * Math.sin(ang) * 2];
  });
  for (let it = 0; it < 500; it++) {
    let moved = 0;
    z = z.map((zi, i) => {
      let den = [1, 0];
      z.forEach((zj, j) => {
        if (j !== i) den = mul(den, [zi[0] - zj[0], zi[1] - zj[1]]);
      });
      const step = div(evalP(zi), den);
      moved = Math.max(moved, Math.hypot(...step));
      return [zi[0] - step[0], zi[1] - step[1]];
    });
    if (moved < 1e-12) break;
  }
  // Snap tiny imaginary parts so real roots print as real.
  return z.map(([re, im]) => [re, Math.abs(im) < 1e-7 * (1 + Math.abs(re)) ? 0 : im]);
}

/**
 * Stability, plus two different "worst" poles: the least damped one sets how
 * much it rings, the slowest one (largest real part) sets how long it takes.
 * They are often not the same pole: a small Ki adds a slow real pole that
 * drags settling out while a fast, lightly damped pair does the ringing.
 */
export function analyzePoles(roots) {
  const damping = ([re, im]) => {
    const mag = Math.hypot(re, im);
    return mag > 0 ? -re / mag : 1;
  };
  const stable = roots.every((r) => r[0] < -1e-9);
  const slowest = roots.reduce((best, r) => (r[0] > best[0] ? r : best), roots[0]);
  const ringing = roots.reduce((best, r) => (damping(r) < damping(best) ? r : best), roots[0]);
  return { stable, slowest, ringing, zeta: damping(ringing), wn: Math.hypot(...ringing) };
}
