import { test } from "node:test";
import assert from "node:assert/strict";
import { MGL, SlamLoop, SwingUp } from "../js/learn-sims.js";

test("the motors in the swing-up lesson are too weak to lift the arm directly", () => {
  assert.ok(new SwingUp().limit < MGL);
});

for (const limit of [3, 3.5, 5]) {
  test(`iLQR plans a swing-up with a ${limit} N·m motor, and following it gets there`, () => {
    const s = new SwingUp({ limit });
    s.replan();
    const c = s.plan.costs;
    for (let i = 1; i < c.length; i++) assert.ok(c[i] < c[i - 1]);
    assert.ok(s.plan.U.every((u) => Math.abs(u) <= limit + 1e-9), "the plan asks for more than the motor has");
    s.advance(7);
    assert.ok(s.upright, `ended at ${(s.x[0] * 180 / Math.PI).toFixed(1)}°`);
  });
}

test("a kick partway through breaks a plan that is followed blindly", () => {
  const s = new SwingUp();
  s.replan();
  s.advance(2);
  s.kick(2.5);
  s.advance(8);
  assert.equal(s.upright, false);
});

test("MPC swings up on its own and gets back up after kicks", () => {
  const s = new SwingUp();
  s.mode = "mpc";
  s.advance(8);
  assert.ok(s.upright, "did not swing up");
  for (const k of [2, -4]) {
    s.kick(k);
    let t = 0;
    while (t < 16 && !(t > 0.5 && s.upright)) { s.advance(0.1); t += 0.1; }
    assert.ok(s.upright, `did not recover from a kick of ${k} rad/s`);
  }
});

test("SLAM's map grows less certain over the first lap, then tightens after closing the loop", () => {
  const L = new SlamLoop();
  let peak = 0;
  for (let i = 0; i < 70 / 0.05; i++) {
    L.step();
    peak = Math.max(peak, L.mapSpread() ?? 0);
  }
  assert.ok(L.closures.length > 0, "never closed the loop");
  assert.ok(L.mapSpread() < peak * 0.6, `map spread ${L.mapSpread().toFixed(3)} vs peak ${peak.toFixed(3)}`);
  assert.ok(L.error("slam") < L.error("dr") / 2, "SLAM no better than dead reckoning");
});
