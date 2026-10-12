import { test } from "node:test";
import assert from "node:assert/strict";
import { SYSTEMS, Simulation } from "../js/pid-sim.js";

const DEG = Math.PI / 180;

// Runs a preset from its start state and summarizes the response. Peaks are
// counted as sign changes of the error, which is what "oscillating" means.
function run(key, preset, seconds = 15) {
  const sim = new Simulation(key, SYSTEMS[key].presets[preset]);
  let crossings = 0;
  let prev = null;
  let maxAbsLate = 0;
  sim.markEvent();
  for (let t = 0; t < seconds; t += 1 / 200) {
    sim.step();
    const e = sim.sp - sim.sys.measure(sim.state);
    if (prev !== null && Math.sign(e) !== Math.sign(prev) && Math.abs(e) > sim.sys.band) crossings++;
    if (Math.abs(e) > sim.sys.band) prev = e;
    if (t > seconds - 3) maxAbsLate = Math.max(maxAbsLate, Math.abs(e));
  }
  return { ...sim.metrics(), crossings, maxAbsLate, sim };
}

const table = [];
for (const key of Object.keys(SYSTEMS)) {
  for (const preset of Object.keys(SYSTEMS[key].presets)) {
    const r = run(key, preset);
    table.push({
      system: key,
      preset,
      overshoot: r.overshoot?.toFixed(1),
      settle: r.settlingTime?.toFixed(2),
      crossings: r.crossings,
      lateErr: SYSTEMS[key].display(r.maxAbsLate).toFixed(3),
    });
  }
}
console.table(table);

const get = (s, p) => table.find((r) => r.system === s && r.preset === p);

test("tuned presets settle with little overshoot", () => {
  for (const key of ["drone", "arm"]) {
    const r = get(key, "tuned");
    assert.ok(r.settle !== undefined && Number(r.settle) < 5, `${key} tuned settle ${r.settle}`);
    assert.ok(Number(r.overshoot) < 12, `${key} tuned overshoot ${r.overshoot}`);
  }
  assert.ok(Number(get("cartpole", "tuned").lateErr) < 1, "cartpole tuned stays upright");
});

test("overshoot presets overshoot but settle", () => {
  for (const key of ["drone", "arm"]) {
    const r = get(key, "overshoot");
    assert.ok(Number(r.overshoot) > 20, `${key} overshoot ${r.overshoot}`);
    assert.ok(r.settle !== undefined, `${key} overshoot preset should settle`);
  }
});

test("oscillating presets keep oscillating", () => {
  for (const key of Object.keys(SYSTEMS)) {
    const r = get(key, "oscillating");
    assert.ok(r.crossings >= 8, `${key} oscillating crossings ${r.crossings}`);
  }
});

test("sluggish presets have no overshoot and settle slowly", () => {
  for (const key of ["drone", "arm"]) {
    const r = get(key, "sluggish");
    assert.ok(Number(r.overshoot) < 3, `${key} sluggish overshoot ${r.overshoot}`);
    assert.ok(r.settle === undefined || Number(r.settle) > 4, `${key} sluggish settle ${r.settle}`);
  }
});

test("without Ki the drone and arm sag", () => {
  for (const key of ["drone", "arm"]) {
    const r = run(key, "no Ki");
    const e = r.sim.sp - r.sim.sys.measure(r.sim.state);
    assert.ok(e > SYSTEMS[key].band * 2, `${key} steady error ${e}`);
  }
});

test("too weak Kp lets the pendulum fall", () => {
  const r = run("cartpole", "too weak");
  assert.ok(Math.abs(r.sim.state.th) > 80 * DEG);
});

test("anti-windup shortens recovery after saturation", () => {
  const settle = (aw) => {
    const sim = new Simulation("drone", { kp: 12, ki: 6, kd: 6 });
    sim.pid.antiWindup = aw;
    sim.setSetpoint(9);
    for (let t = 0; t < 25; t += 1 / 200) sim.step();
    return sim.metrics();
  };
  const on = settle(true), off = settle(false);
  assert.ok(on.overshoot < off.overshoot, `aw on ${on.overshoot} vs off ${off.overshoot}`);
});

import { analyzePoles, charPoly, polyRoots } from "../js/pid-sim.js";

test("polynomial roots", () => {
  // (s+1)(s+2)(s+3) = s³ + 6s² + 11s + 6
  const r = polyRoots([1, 6, 11, 6]).map((z) => z[0]).sort((a, b) => a - b);
  [-3, -2, -1].forEach((v, i) => assert.ok(Math.abs(r[i] - v) < 1e-9, `${r}`));
  // s² + 2s + 5 → -1 ± 2j
  const c = polyRoots([1, 2, 5]);
  assert.ok(c.every((z) => Math.abs(z[0] + 1) < 1e-9 && Math.abs(Math.abs(z[1]) - 2) < 1e-9));
});

test("linear prediction agrees with the simulation", () => {
  const DEG = Math.PI / 180;
  const verdict = (key, preset) => {
    const sys = SYSTEMS[key];
    return analyzePoles(polyRoots(charPoly(key, sys.presets[preset], sys.fromDisplay(sys.setpoint.default))));
  };
  assert.equal(verdict("cartpole", "too weak").stable, false, "Kp below (M+m)g falls");
  assert.equal(verdict("cartpole", "tuned").stable, true);
  for (const key of ["drone", "arm"]) {
    assert.equal(verdict(key, "tuned").stable, true, key);
    assert.ok(verdict(key, "oscillating").zeta < verdict(key, "tuned").zeta, `${key} oscillating is less damped`);
  }
  // The arm above horizontal behaves like an inverted pendulum: gravity
  // subtracts from Kp, so weak gains that hold it low cannot hold it high.
  const weak = { kp: 5, ki: 0, kd: 2 };
  assert.equal(analyzePoles(polyRoots(charPoly("arm", weak, 30 * DEG))).stable, true);
  assert.equal(analyzePoles(polyRoots(charPoly("arm", weak, 150 * DEG))).stable, false);
});
