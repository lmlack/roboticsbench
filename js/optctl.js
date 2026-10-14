// Optimal control for the three PID plants: iLQR, and MPC built on it.
// Pure JS, no DOM, so it can be checked under node --test.
//
// Every plant here has one input (thrust, torque or force), which keeps the
// algebra small: the "Quu" matrix of iLQR is a single number.

import { SYSTEMS } from "./pid-sim.js";

const g = 9.81;
const wrap = (a) => a - 2 * Math.PI * Math.floor((a + Math.PI) / (2 * Math.PI));

/**
 * Each plant as a vector model the optimizers can plan with: the state as an
 * array, which entries are angles (their error wraps), the goal state and
 * the input that holds it there, and the cost weights.
 */
export const MODELS = {
  drone: {
    keys: ["y", "v"],
    angles: [],
    goal: (sp) => ({ x: [sp, 0], u: g * 1 }),
    Q: [40, 4],
    R: 0.02,
    Qf: [400, 40],
  },
  arm: {
    keys: ["th", "w"],
    // Not wrapped: a real joint has cables and stops, so the planner should
    // reach 75° by turning 75°, not by going once round to 435°.
    angles: [],
    // Holding still at sp takes exactly the torque gravity applies there.
    goal: (sp) => ({ x: [sp, 0], u: -SYSTEMS.arm.gravityTorque({ th: sp }) }),
    Q: [60, 2],
    R: 0.05,
    Qf: [600, 20],
  },
  cartpole: {
    keys: ["x", "v", "th", "w"],
    angles: [2],
    goal: () => ({ x: [0, 0, 0, 0], u: 0 }),
    Q: [1, 0.5, 40, 2],
    R: 0.01,
    Qf: [20, 10, 800, 60],
  },
};

export const toVec = (key, s) => MODELS[key].keys.map((k) => s[k]);
export const fromVec = (key, x) => Object.fromEntries(MODELS[key].keys.map((k, i) => [k, x[i]]));

/** x_{t+1} = f(x_t, u_t): RK4 over dt, ignoring the floor and the walls. */
export function step(key, x, u, dt) {
  const sys = SYSTEMS[key];
  const keys = MODELS[key].keys;
  const d = (xv) => {
    const r = sys.deriv(fromVec(key, xv), u);
    return keys.map((k) => r[k]);
  };
  const n = Math.max(1, Math.ceil(dt / 0.02));
  const h = dt / n;
  let s = x.slice();
  for (let i = 0; i < n; i++) {
    const k1 = d(s);
    const k2 = d(s.map((v, j) => v + (h / 2) * k1[j]));
    const k3 = d(s.map((v, j) => v + (h / 2) * k2[j]));
    const k4 = d(s.map((v, j) => v + h * k3[j]));
    s = s.map((v, j) => v + (h / 6) * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]));
  }
  return s;
}

/** State error to the goal, with angles wrapped so 350° is 10° short of 0°. */
export function stateError(key, x, xg) {
  const ang = MODELS[key].angles;
  return x.map((v, i) => (ang.includes(i) ? wrap(v - xg[i]) : v - xg[i]));
}

/** Cost of a whole trajectory: Σ ½eᵀQe + ½R(u−u*)² plus ½eᵀQf e at the end. */
export function trajectoryCost(key, X, U, sp, w = MODELS[key]) {
  const { x: xg, u: ug } = MODELS[key].goal(sp);
  let J = 0;
  for (let t = 0; t < U.length; t++) {
    const e = stateError(key, X[t], xg);
    J += 0.5 * e.reduce((s, ei, i) => s + w.Q[i] * ei * ei, 0) + 0.5 * w.R * (U[t] - ug) ** 2;
  }
  const e = stateError(key, X[X.length - 1], xg);
  return J + 0.5 * e.reduce((s, ei, i) => s + w.Qf[i] * ei * ei, 0);
}

/**
 * A and B of the linearized step, by forward differences: one extra step per
 * state and one for the input. Accurate enough for iLQR, which only needs a
 * good descent direction, and half the cost of central differences.
 */
function linearize(key, x, u, dt, f0 = step(key, x, u, dt)) {
  const n = x.length, eps = 1e-6;
  const A = Array.from({ length: n }, () => new Array(n).fill(0));
  for (let j = 0; j < n; j++) {
    const xp = x.slice();
    xp[j] += eps;
    const fp = step(key, xp, u, dt);
    for (let i = 0; i < n; i++) A[i][j] = (fp[i] - f0[i]) / eps;
  }
  const fp = step(key, x, u + eps, dt);
  const B = fp.map((v, i) => (v - f0[i]) / eps);
  return { A, B };
}

const rollout = (key, x0, U, dt) => {
  const X = [x0.slice()];
  for (const u of U) X.push(step(key, X[X.length - 1], u, dt));
  return X;
};

/**
 * iLQR with box limits on u. Each iteration linearizes the dynamics around
 * the current trajectory, solves the resulting LQR problem backward in time
 * for a correction k and feedback gains K, then rolls forward with a line
 * search. Where the correction would push u past a limit, it is clamped there
 * and the feedback on that step is switched off.
 *
 * @returns {{X: number[][], U: number[], K: number[][], costs: number[], iters: number}}
 */
export function ilqr(key, x0, sp, { N, dt, U0, maxIter = 60, tol = 1e-4, weights } = {}) {
  const m = MODELS[key];
  const w = weights ?? m;
  const [lo, hi] = SYSTEMS[key].uLimit;
  const { x: xg, u: ug } = m.goal(sp);
  const n = m.keys.length;
  const clamp = (u) => Math.min(hi, Math.max(lo, u));
  let U = (U0 ?? new Array(N).fill(ug)).map(clamp);
  let X = rollout(key, x0, U, dt);
  let J = trajectoryCost(key, X, U, sp, w);
  const costs = [J];
  let K = Array.from({ length: N }, () => new Array(n).fill(0));
  let mu = 1e-6;
  let iters = 0;

  for (; iters < maxIter; iters++) {
    // Backward pass.
    const eN = stateError(key, X[N], xg);
    let Vx = eN.map((e, i) => w.Qf[i] * e);
    let Vxx = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? w.Qf[i] : 0)));
    const kff = new Array(N), Kfb = new Array(N);
    let ok = true;
    for (let t = N - 1; t >= 0; t--) {
      const { A, B } = linearize(key, X[t], U[t], dt, X[t + 1]);
      const e = stateError(key, X[t], xg);
      const lx = e.map((ei, i) => w.Q[i] * ei);
      const lu = w.R * (U[t] - ug);
      // Q-function terms. With one input, Quu is a scalar.
      const VB = Vxx.map((row) => row.reduce((s, v, j) => s + v * B[j], 0));       // Vxx·B
      const Qx = lx.map((v, i) => v + A.reduce((s, row, r) => s + row[i] * Vx[r], 0)); // lx + AᵀVx
      const Qu = lu + B.reduce((s, b, r) => s + b * Vx[r], 0);
      const Quu = w.R + B.reduce((s, b, r) => s + b * VB[r], 0) + mu;
      const Qux = Array.from({ length: n }, (_, j) => A.reduce((s, row, r) => s + VB[r] * row[j], 0));
      const AVA = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => {
        let s = 0;
        for (let r = 0; r < n; r++) for (let c = 0; c < n; c++) s += A[r][i] * Vxx[r][c] * A[c][j];
        return s;
      }));
      const Qxx = AVA.map((row, i) => row.map((v, j) => v + (i === j ? w.Q[i] : 0)));
      if (!(Quu > 0)) { ok = false; break; }
      let k = -Qu / Quu;
      let Kt = Qux.map((v) => -v / Quu);
      // Box limits: clamp the correction at the bound and drop the feedback.
      if (U[t] + k > hi) { k = hi - U[t]; Kt = Kt.map(() => 0); }
      else if (U[t] + k < lo) { k = lo - U[t]; Kt = Kt.map(() => 0); }
      kff[t] = k;
      Kfb[t] = Kt;
      Vx = Qx.map((v, i) => v + Kt[i] * Quu * k + Kt[i] * Qu + Qux[i] * k);
      Vxx = Qxx.map((row, i) => row.map((v, j) => v + Kt[i] * Quu * Kt[j] + Kt[i] * Qux[j] + Qux[i] * Kt[j]));
      Vxx = Vxx.map((row, i) => row.map((v, j) => 0.5 * (v + Vxx[j][i])));
    }
    if (!ok) { mu = Math.max(mu * 10, 1e-4); continue; }

    // Forward pass with a backtracking line search.
    let accepted = false;
    for (let alpha = 1; alpha > 1e-3; alpha *= 0.5) {
      const Xn = [x0.slice()], Un = [];
      for (let t = 0; t < N; t++) {
        const dx = stateError(key, Xn[t], X[t]);
        const u = clamp(U[t] + alpha * kff[t] + Kfb[t].reduce((s, kv, i) => s + kv * dx[i], 0));
        Un.push(u);
        Xn.push(step(key, Xn[t], u, dt));
      }
      const Jn = trajectoryCost(key, Xn, Un, sp, w);
      if (Jn < J) {
        const drop = (J - Jn) / Math.max(1, Math.abs(J));
        X = Xn; U = Un; K = Kfb; J = Jn;
        costs.push(J);
        accepted = true;
        mu = Math.max(1e-6, mu / 4);
        if (drop < tol) { iters++; return { X, U, K, costs, iters }; }
        break;
      }
    }
    if (!accepted) {
      mu *= 10;
      if (mu > 1e6) break;
    }
  }
  return { X, U, K, costs, iters };
}

/** Steady-state LQR gain at the goal, by running the Riccati recursion out. */
export function lqrGain(key, sp, dt) {
  const m = MODELS[key];
  const { x: xg, u: ug } = m.goal(sp);
  const { A, B } = linearize(key, xg, ug, dt);
  const n = xg.length;
  let P = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? m.Qf[i] : 0)));
  let K = new Array(n).fill(0);
  for (let it = 0; it < 2000; it++) {
    const PB = P.map((row) => row.reduce((s, v, j) => s + v * B[j], 0));
    const den = m.R + B.reduce((s, b, r) => s + b * PB[r], 0);
    const BPA = Array.from({ length: n }, (_, j) => A.reduce((s, row, r) => s + PB[r] * row[j], 0));
    K = BPA.map((v) => -v / den);
    const next = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => {
      let s = 0;
      for (let r = 0; r < n; r++) for (let c = 0; c < n; c++) s += A[r][i] * P[r][c] * A[c][j];
      return s + (i === j ? m.Q[i] : 0) - BPA[i] * BPA[j] / den;
    }));
    const diff = Math.max(...next.flat().map((v, i) => Math.abs(v - P.flat()[i])));
    P = next;
    if (diff < 1e-9) break;
  }
  return K;
}

/**
 * MPC: every `period` seconds, plan `N` steps of `dt` ahead from the current
 * state with a few iLQR iterations, warm-started from the last plan shifted
 * by one step. Between replans, follow the plan's first step with its
 * feedback gain, so the input still reacts at the full control rate.
 */
export class MPC {
  constructor(key, { N = 40, dt = 0.05, iters = 2 } = {}) {
    this.key = key;
    this.N = N;
    this.dt = dt;
    this.iters = iters;
    this.reset();
  }

  vec(state) {
    return toVec(this.key, state);
  }

  reset() {
    this.plan = null;
    this.since = Infinity;
    this.solves = 0;
    this.lastIters = 0;
    this.lastCost = null;
  }

  update(x, sp, ctrlDt) {
    if (!this.plan || this.since >= this.dt - 1e-9) {
      const warm = this.plan ? [...this.plan.U.slice(1), this.plan.U[this.plan.U.length - 1]] : null;
      const r = ilqr(this.key, x, sp, { N: this.N, dt: this.dt, U0: warm, maxIter: this.plan ? this.iters : 30 });
      this.plan = { ...r, t: 0 };
      this.since = 0;
      this.solves++;
      this.lastIters = r.iters;
      this.lastCost = r.costs[r.costs.length - 1];
    }
    const { X, U, K } = this.plan;
    // Where the plan expected us to be by now, between its first two points.
    const f = Math.min(1, this.since / this.dt);
    const xp = X[0].map((v, i) => v + (X[1][i] - v) * f);
    const dx = stateError(this.key, x, xp);
    const [lo, hi] = SYSTEMS[this.key].uLimit;
    const raw = U[0] + K[0].reduce((s, k, i) => s + k * dx[i], 0);
    this.since += ctrlDt;
    return { u: Math.min(hi, Math.max(lo, raw)), raw };
  }

  /** The forecast, as states from now on, for drawing. */
  forecast() {
    return this.plan ? this.plan.X : [];
  }
}

/**
 * iLQR as a controller: optimize one long plan from where the system is, then
 * follow it with the time-varying feedback gains the solver produced. When the
 * plan runs out, hold the goal with a steady-state LQR. Re-plan after a kick
 * or a new target.
 */
export class ILQRController {
  constructor(key, { horizon = 3, dt = 0.02 } = {}) {
    this.key = key;
    this.horizon = horizon;
    this.dt = dt;
    this.reset();
  }

  vec(state) {
    return toVec(this.key, state);
  }

  /** A kick throws the system off the plan: plan again from where it landed. */
  disturbed() {
    this.plan = null;
  }

  reset() {
    this.plan = null;
    this.t = 0;
    this.hold = null;
    this.sp = null;
  }

  replan(x, sp) {
    const N = Math.round(this.horizon / this.dt);
    this.plan = ilqr(this.key, x, sp, { N, dt: this.dt, maxIter: 150, tol: 1e-6 });
    this.t = 0;
    this.sp = sp;
    this.hold = lqrGain(this.key, sp, this.dt);
  }

  update(x, sp, ctrlDt) {
    if (!this.plan || sp !== this.sp) this.replan(x, sp);
    const { X, U, K } = this.plan;
    const [lo, hi] = SYSTEMS[this.key].uLimit;
    const j = Math.floor(this.t / this.dt + 1e-9);
    let raw;
    if (j < U.length) {
      const dx = stateError(this.key, x, X[j]);
      raw = U[j] + K[j].reduce((s, k, i) => s + k * dx[i], 0);
    } else {
      const { x: xg, u: ug } = MODELS[this.key].goal(sp);
      const dx = stateError(this.key, x, xg);
      raw = ug + this.hold.reduce((s, k, i) => s + k * dx[i], 0);
    }
    this.t += ctrlDt;
    return { u: Math.min(hi, Math.max(lo, raw)), raw };
  }

  /** The part of the plan still ahead, for drawing. */
  forecast() {
    if (!this.plan) return [];
    const j = Math.min(this.plan.X.length - 1, Math.floor(this.t / this.dt));
    return this.plan.X.slice(j);
  }
}
