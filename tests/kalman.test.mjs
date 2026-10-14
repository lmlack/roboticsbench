import { test } from "node:test";
import assert from "node:assert/strict";
import { DEFAULTS, DRIVE, KalmanFilter, PRESETS, Simulation, TRUE_Q, inDropout } from "../js/kalman-sim.js";

// advance() caps its work per call for the UI, so long runs go a second at a time.
function run(params, seconds, seed = 3, onStep) {
  const sim = new Simulation(params, seed);
  for (let t = 0; t < seconds; t++) {
    if (onStep) {
      const end = sim.t + 1;
      while (sim.t < end - 1e-9) {
        sim.step();
        onStep(sim);
      }
    } else {
      sim.advance(1);
    }
  }
  return sim;
}

const tuned = PRESETS.tuned.apply(DEFAULTS);

test("well tuned on Random accel beats the raw sensor and is consistent", () => {
  for (const seed of [1, 2, 3]) {
    const m = run({ profile: "random", ...tuned }, 600, seed).totalMetrics();
    assert.ok(m.estRmse < 0.75 * m.measRmse, `seed ${seed}: estimate ${m.estRmse} vs sensor ${m.measRmse}`);
    assert.ok(m.coverage > 0.9 && m.coverage < 0.99, `seed ${seed}: coverage ${m.coverage}`);
  }
});

test("P stays symmetric positive definite", () => {
  const settings = [
    { ...tuned },
    { ...tuned, fuseVel: true, dropout: true },
    { ...PRESETS.sensor.apply(DEFAULTS), rate: 50 },
    { ...PRESETS.model.apply(DEFAULTS), rate: 2, dropout: true },
  ];
  for (const s of settings) {
    let lastT = -1;
    run(s, 60, 5, (sim) => {
      if (!sim.last || sim.last.t === lastT) return;
      lastT = sim.last.t;
      for (const P of [sim.last.PPrior, sim.last.P]) {
        assert.equal(P[0][1], P[1][0], "symmetric");
        assert.ok(P[0][0] > 0, `P00 ${P[0][0]}`);
        assert.ok(P[0][0] * P[1][1] - P[0][1] ** 2 > 0, `det ${JSON.stringify(P)}`);
      }
    });
  }
});

test("steady-state gain matches the discrete Riccati equation", () => {
  for (const rate of [5, 10, 40]) {
    const sim = run({ ...tuned, rate }, 60);
    const dt = 1 / rate, q = TRUE_Q, r = DEFAULTS.sigmaZ ** 2;
    // Iterate predict and update by hand, written out element by element.
    let a = 4, b = 0, c = 4; // P = [[a, b], [b, c]]
    let k0 = 0;
    for (let i = 0; i < 5000; i++) {
      const pa = a + 2 * dt * b + dt * dt * c + (q * dt ** 3) / 3;
      const pb = b + dt * c + (q * dt ** 2) / 2;
      const pc = c + q * dt;
      const s = pa + r;
      k0 = pa / s;
      const k1 = pb / s;
      a = (1 - k0) * pa;
      b = (1 - k0) * pb;
      c = pc - k1 * pb;
    }
    const K = sim.last.K;
    assert.ok(Math.abs(K[0][0] - k0) < 1e-6, `rate ${rate}: sim ${K[0][0]} vs Riccati ${k0}`);
  }
});

test("trusting the sensor too much lets the noise through", () => {
  const good = run({ ...tuned }, 300).totalMetrics();
  const bad = run({ ...PRESETS.sensor.apply(DEFAULTS) }, 300).totalMetrics();
  assert.ok(bad.estRmse > good.estRmse * 1.2, `R too small ${bad.estRmse} vs tuned ${good.estRmse}`);
  assert.ok(bad.coverage < 0.6, `R too small should be overconfident, coverage ${bad.coverage}`);
});

test("overcautious has the same gain as well tuned but a band that is too wide", () => {
  const a = run({ ...tuned }, 60);
  const b = run({ ...PRESETS.cautious.apply(DEFAULTS) }, 60);
  assert.ok(Math.abs(a.last.K[0][0] - b.last.K[0][0]) < 1e-9);
  assert.ok(b.totalMetrics().coverage > 0.995);
});

test("during dropout P[0][0] grows every step, then shrinks once data returns", () => {
  const sim = new Simulation({ ...tuned, dropout: true }, 4);
  while (sim.t < 14) sim.advance(1);
  const inGap = [];
  let after = null;
  let lastT = -1;
  while (sim.t < 19) {
    sim.step();
    if (sim.last.t === lastT) continue;
    lastT = sim.last.t;
    if (inDropout(sim.last.t)) {
      assert.equal(sim.last.dropped, true);
      inGap.push(sim.last.P[0][0]);
    } else if (inGap.length && after === null) {
      after = sim.last.P[0][0];
    }
  }
  assert.ok(inGap.length >= 25, `steps in gap ${inGap.length}`);
  for (let i = 1; i < inGap.length; i++) assert.ok(inGap[i] > inGap[i - 1], `step ${i}: ${inGap[i]} <= ${inGap[i - 1]}`);
  assert.ok(after < inGap[inGap.length - 1] / 4, `snaps back: ${after} vs ${inGap[inGap.length - 1]}`);
});

test("runs repeat exactly for a given seed", () => {
  const a = run({ profile: "stopgo" }, 20, 9).totalMetrics();
  const b = run({ profile: "stopgo" }, 20, 9).totalMetrics();
  assert.deepEqual(a, b);
});

test("a warmed-up run opens at steady state with full charts", async () => {
  const { Simulation, WARMUP_S } = await import("../js/kalman-sim.js");
  const sim = new Simulation({}, 7);
  sim.warmUp();
  const window = sim.history.filter((h) => h.t >= sim.t - 10);
  assert.ok(sim.t >= WARMUP_S - 1e-6);
  assert.ok(window[0].t <= sim.t - 9.9, "the 10 s chart window is already full");
  // The cold-start transient is gone: the band no longer moves much.
  const sp = window.map((h) => h.sp);
  assert.ok(Math.max(...sp) < 1.5 * Math.min(...sp), `band varies ${Math.min(...sp)} to ${Math.max(...sp)}`);
  assert.ok(sp[0] < 1, "band is not the 2 m cold-start prior");
});

test("You steer: the cart reaches the target smoothly and stops there", () => {
  const sim = new Simulation({ profile: "drive", driveTarget: 3 }, 3);
  let peak = -Infinity;
  while (sim.t < 3) {
    sim.step();
    peak = Math.max(peak, sim.truth.p);
  }
  assert.ok(Math.abs(sim.truth.p - 3) < 1e-3, `settles at the target, got ${sim.truth.p}`);
  assert.ok(Math.abs(sim.truth.v) < 1e-3, `and stops, v = ${sim.truth.v}`);
  // Critically damped, so it must not sail past the target and come back.
  assert.ok(peak <= 3 + 1e-6, `no overshoot, peak ${peak}`);
});

test("You steer: the target is kept on the track", () => {
  for (const [target, want] of [[1000, DRIVE.reach], [-1000, -DRIVE.reach], [NaN, 0]]) {
    const sim = new Simulation({ profile: "drive", driveTarget: target }, 3);
    sim.advance(20);
    assert.ok(Math.abs(sim.truth.p - want) < 1e-3, `target ${target} should hold at ${want}, got ${sim.truth.p}`);
  }
});

test("You steer: a sudden maneuver throws the estimate off, then it catches up", () => {
  const sim = new Simulation({ profile: "drive", ...tuned }, 3);
  sim.warmUp();
  const before = Math.abs(sim.history[sim.history.length - 1].pe - sim.history[sim.history.length - 1].p);
  assert.ok(before < 0.4, `settled before the yank, error ${before}`);
  sim.set({ driveTarget: 4 });
  let worst = 0;
  const t0 = sim.t;
  while (sim.t < t0 + 0.6) {
    sim.step();
    const h = sim.history[sim.history.length - 1];
    worst = Math.max(worst, Math.abs(h.pe - h.p));
  }
  assert.ok(worst > 0.8, `the constant-velocity model should lag a 4 m yank, worst ${worst}`);
  sim.advance(3);
  const h = sim.history[sim.history.length - 1];
  assert.ok(Math.abs(h.p - 4) < 1e-3, "the cart is parked at the target");
  assert.ok(Math.abs(h.pe - h.p) < 0.25, `the filter caught up, error ${Math.abs(h.pe - h.p)}`);
});

test("You steer: changing the target does not restart the run", () => {
  const sim = new Simulation({ profile: "drive" }, 3);
  sim.advance(5);
  const t = sim.t;
  sim.set({ driveTarget: 2 });
  assert.equal(sim.t, t, "the clock keeps running");
  assert.ok(sim.history.length > 1, "and the history is kept");
});

test("outliers drag the estimate around, and gating on d² throws them out", () => {
  const run = (gate) => {
    const sim = new Simulation({ outliers: true, gate }, 3);
    sim.warmUp();
    let outliers = 0, rejected = 0, goodRejected = 0, readings = 0, worstJump = 0;
    let prev = sim.filter.x[0][0];
    for (let i = 0; i < 1200; i++) {
      sim.advance(0.1);
      const g = sim.last.gate;
      if (g) {
        readings++;
        if (g.outlier) outliers++;
        if (g.rejected) rejected++;
        if (g.rejected && !g.outlier) goodRejected++;
      }
      worstJump = Math.max(worstJump, Math.abs(sim.filter.x[0][0] - prev));
      prev = sim.filter.x[0][0];
    }
    return { ...sim.totalMetrics(), outliers, rejected, goodRejected, readings, worstJump };
  };
  const open = run(false), gated = run(true);
  assert.ok(open.outliers > 40, `${open.outliers} outliers`);
  assert.equal(open.rejected, 0);
  // Every outlier is meters off, so the gate catches nearly all of them, and
  // a good reading is past the 99% point only about 1% of the time.
  assert.ok(gated.rejected - gated.goodRejected >= 0.95 * gated.outliers, JSON.stringify(gated));
  assert.ok(gated.goodRejected <= 0.03 * gated.readings, JSON.stringify(gated));
  assert.ok(gated.estRmse < 0.7 * open.estRmse, `rmse ${gated.estRmse} vs ${open.estRmse}`);
  assert.ok(open.worstJump > 2 * gated.worstJump, `jumps ${open.worstJump} vs ${gated.worstJump}`);
});

test("d² is y²/S for a single reading", () => {
  const f = new KalmanFilter();
  f.reset([1, 0], [[0.5, 0], [0, 1]]);
  const { y, S, d2 } = f.innovation([[3]], [[1, 0]], [[0.25]]);
  assert.ok(Math.abs(y[0][0] - 2) < 1e-12 && Math.abs(S[0][0] - 0.75) < 1e-12);
  assert.ok(Math.abs(d2 - 4 / 0.75) < 1e-12);
});
