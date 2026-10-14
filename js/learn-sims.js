// The two simulations behind Learn lessons 7 and 8, kept free of the DOM so
// they can be tested: a weak-motor swing-up planned with iLQR (and run with
// MPC), and a robot driving a loop while EKF-SLAM maps the landmarks.

import { ilqr, lqrGain, step } from "./optctl.js";
import { DT, DeadReckoning, EKFSLAM, makeGauss, mulberry32, noiseVar } from "./localization-sim.js";

const wrap = (a) => a - 2 * Math.PI * Math.floor((a + Math.PI) / (2 * Math.PI));

/* ---------- Lesson 7: swing-up ---------- */

// Charge only lightly for being away from the top along the way, and a lot
// for not being there at the end. A heavy running cost makes swinging away
// first look expensive, and the planner then settles for hanging partway up.
export const SWING_WEIGHTS = { Q: [2, 0.1], R: 0.02, Qf: [2000, 200] };
export const MGL = 9.81; // N·m: what holding the arm out level takes

export class SwingUp {
  constructor({ limit = 3.5, horizon = 5, dt = 0.03, coldIters = 25, warmIters = 5 } = {}) {
    this.limit = limit;
    // MPC's first plan after a kick is kept short so the page never stalls;
    // the next re-plans, warm-started, finish the job.
    this.coldIters = coldIters;
    this.warmIters = warmIters;
    this.retryHorizon = 1.5;
    this.horizon = horizon;
    this.dt = dt;
    this.mode = "follow"; // or "mpc"
    this.reset();
  }

  reset() {
    this.x = [0, 0]; // hanging straight down, still
    this.t = 0;
    this.u = 0;
    this.plan = null;
    this.iterations = [];
    this.mpc = null;
    this.solves = 0;
    this.deadline = this.horizon;
    this.hold = lqrGain("arm", Math.PI, 0.01);
  }

  /** Optimize one whole plan from where the arm is now, keeping every iteration. */
  replan() {
    this.iterations = [];
    const N = Math.round(this.horizon / this.dt);
    this.plan = ilqr("arm", this.x, Math.PI, {
      N, dt: this.dt, maxIter: 300, tol: 1e-8, limits: [-this.limit, this.limit],
      weights: SWING_WEIGHTS, onIter: (X) => this.iterations.push(X),
    });
    this.t = 0;
    this.mpc = null;
  }

  kick(dw = 2) {
    this.x[1] += dw;
    // MPC aims for a fresh finish time. Following the plan just carries on.
    this.deadline = this.t + this.horizon;
    this.mpc = null;
  }

  /** The input right now: the plan's, MPC's, or a steady LQR once at the top. */
  control() {
    const lim = this.limit;
    const clamp = (u) => Math.max(-lim, Math.min(lim, u));
    // Close enough and slow enough that the weak motor can catch it.
    const nearTop = Math.abs(wrap(this.x[0] - Math.PI)) < 0.15 && Math.abs(this.x[1]) < 0.6;
    if (this.mode === "mpc") {
      // Shrinking-horizon MPC: every 100 ms it re-plans from the measured
      // state toward a fixed finish time. With a sliding window the finish
      // would always be 5 s away and it would keep putting the swing off.
      const left = this.deadline - this.t;
      if (nearTop || left < 0.3) {
        if (!nearTop) {
          // Missed it: try again. If it is already close to the top, a short
          // second try is enough; otherwise it needs a whole new swing.
          const close = Math.abs(wrap(this.x[0] - Math.PI)) < 0.6;
          this.deadline = this.t + (close ? this.retryHorizon : this.horizon);
        }
        this.mpc = null;
        return clamp(this.hold[0] * wrap(this.x[0] - Math.PI) + this.hold[1] * this.x[1]);
      }
      if (!this.mpc || this.mpc.since >= 0.1 - 1e-9) {
        const dt = 0.05, N = Math.max(4, Math.round(left / dt));
        // Warm start: the last plan, shifted by the 100 ms that passed.
        const warm = this.mpc ? this.mpc.U.slice(2, 2 + N) : null;
        while (warm && warm.length < N) warm.push(warm[warm.length - 1] ?? 0);
        const r = ilqr("arm", this.x, Math.PI, {
          // Near the finish the plan is short and an iteration is cheap, so it
          // can afford more of them, which is when arriving gently matters.
          N, dt, U0: warm, maxIter: this.mpc ? Math.min(40, Math.max(this.warmIters, Math.round(500 / N))) : this.coldIters,
          tol: 1e-8, limits: [-lim, lim], weights: SWING_WEIGHTS,
        });
        this.mpc = { ...r, dt, since: 0 };
        this.solves++;
      }
      return this.mpc.U[0];
    }
    // Following the plan: open loop, exactly as computed. A kick partway
    // through leaves the rest of the plan aimed at a state the arm is not in.
    const j = Math.floor(this.t / this.dt + 1e-9);
    if (this.plan && j < this.plan.U.length) return this.plan.U[j];
    return nearTop ? clamp(this.hold[0] * wrap(this.x[0] - Math.PI) + this.hold[1] * this.x[1]) : 0;
  }

  advance(seconds) {
    const h = 0.005;
    for (let k = 0; k < Math.round(seconds / h); k++) {
      this.u = this.control();
      this.x = step("arm", this.x, this.u, h);
      this.t += h;
      if (this.mpc) this.mpc.since += h;
    }
  }

  /** Where MPC currently expects the arm to go, as states from now. */
  forecast() {
    return this.mode === "mpc" && this.mpc ? this.mpc.X : null;
  }

  get upright() {
    return Math.abs(wrap(this.x[0] - Math.PI)) < 5 * (Math.PI / 180) && Math.abs(this.x[1]) < 0.3;
  }
}

/* ---------- Lesson 8: SLAM around a loop ---------- */

export const LOOP = { cx: 5, cy: 3.5, rx: 3.4, ry: 2.2, speed: 0.5 };
// Landmarks scattered around the loop, a little inside and outside it.
export const LOOP_LANDMARKS = [
  { x: 1.0, y: 3.4 }, { x: 2.0, y: 5.9 }, { x: 4.4, y: 6.4 }, { x: 6.9, y: 6.1 },
  { x: 8.9, y: 4.6 }, { x: 8.7, y: 1.9 }, { x: 6.4, y: 0.6 }, { x: 3.6, y: 0.7 },
  { x: 1.6, y: 1.3 }, { x: 5.0, y: 3.6 },
];

export class SlamLoop {
  constructor({ seed = 5, range = 2, odomNoise = 3, sigmaR = 0.05, sigmaB = 0.02 } = {}) {
    this.seed = seed;
    this.range = range;
    this.odomNoise = odomNoise;
    this.sigmaR = sigmaR;
    this.sigmaB = sigmaB;
    this.landmarks = LOOP_LANDMARKS;
    this.reset();
  }

  reset() {
    this.rng = mulberry32(this.seed);
    this.gauss = makeGauss(this.rng);
    this.s = 0; // arc parameter along the loop
    const start = this.pointAt(0);
    this.truth = { x: start.x, y: start.y, th: start.th };
    this.slam = new EKFSLAM();
    this.slam.init(this.truth, 0.02, 0.02, 0.01);
    this.dr = new DeadReckoning();
    this.dr.init({ ...this.truth });
    this.t = 0;
    this.acc = 0;
    this.z = [];
    this.trail = [];
    this.firstSeen = new Map();
    this.lastSeen = new Map();
    this.closures = [];
    this.history = [];
  }

  /** A point on the elliptical loop and the heading along it. */
  pointAt(s) {
    const { cx, cy, rx, ry } = LOOP;
    return { x: cx + rx * Math.cos(s), y: cy + ry * Math.sin(s), th: Math.atan2(ry * Math.cos(s), -rx * Math.sin(s)) };
  }

  /** Drive along the loop: commands from the true pose, noisy wheels, noisy sensing. */
  step() {
    // Steer toward a point a little further along the loop.
    const ahead = this.pointAt(this.s + 0.25);
    const tr = this.truth;
    const want = Math.atan2(ahead.y - tr.y, ahead.x - tr.x);
    const v = LOOP.speed;
    const w = Math.max(-1.2, Math.min(1.2, 2.5 * wrap(want - tr.th)));
    const near = (s) => Math.hypot(this.pointAt(s).x - tr.x, this.pointAt(s).y - tr.y);
    for (let i = 0; i < 20 && near(this.s + 0.02) < near(this.s); i++) this.s += 0.02;

    const [vv, vw] = noiseVar(v, w, this.odomNoise);
    const vn = v + Math.sqrt(vv) * this.gauss();
    const wn = w + Math.sqrt(vw) * this.gauss();
    const tm = tr.th + (wn * DT) / 2;
    tr.x += vn * DT * Math.cos(tm);
    tr.y += vn * DT * Math.sin(tm);
    tr.th = wrap(tr.th + wn * DT);

    this.slam.predict(v, w, DT, this.odomNoise);
    this.dr.predict(v, w, DT);

    const z = [];
    this.landmarks.forEach((lm, j) => {
      const dx = lm.x - tr.x, dy = lm.y - tr.y;
      const r = Math.hypot(dx, dy);
      if (r > this.range) return;
      z.push({ j, r: r + this.sigmaR * this.gauss(), b: wrap(Math.atan2(dy, dx) - tr.th + this.sigmaB * this.gauss()) });
    });
    this.z = z;
    // A loop closure: a landmark mapped long ago, out of sight for a while,
    // seen again.
    for (const m of z) {
      const first = this.firstSeen.get(m.j);
      const last = this.lastSeen.get(m.j);
      if (first === undefined) this.firstSeen.set(m.j, this.t);
      else if (this.t - last > 8) this.closures.push({ t: this.t, j: m.j, before: this.mapSpread() });
      this.lastSeen.set(m.j, this.t);
    }
    this.slam.update(z, this.sigmaR, this.sigmaB);
    for (const c of this.closures) if (c.after === undefined && this.t - c.t >= 0) c.after = this.mapSpread();

    this.t += DT;
    const e = this.slam.estimate();
    this.trail.push({ t: this.t, truth: [tr.x, tr.y], slam: [e.x, e.y], dr: [this.dr.pose.x, this.dr.pose.y] });
    if (this.trail.length > 1200) this.trail.shift();
    this.history.push({ t: this.t, spread: this.mapSpread(), slam: this.error("slam"), dr: this.error("dr") });
    if (this.history.length > 1200) this.history.shift();
  }

  advance(seconds) {
    this.acc = Math.min(this.acc + seconds, 0.5);
    while (this.acc >= DT) {
      this.acc -= DT;
      this.step();
    }
  }

  /** Mean 95% semi-axis of the mapped landmarks: how uncertain the map is. */
  mapSpread() {
    const map = this.slam.map();
    return map.length ? map.reduce((s, m) => s + m.rx, 0) / map.length : null;
  }

  error(which) {
    const e = which === "slam" ? this.slam.estimate() : this.dr.pose;
    return Math.hypot(e.x - this.truth.x, e.y - this.truth.y);
  }
}
