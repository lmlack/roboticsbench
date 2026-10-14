import { test } from "node:test";
import assert from "node:assert/strict";
import { SYSTEMS, Simulation } from "../js/pid-sim.js";
import { ILQRController, LQRController, MODELS, MPC, ilqr, lqrGain, step, stateError, toVec } from "../js/optctl.js";

const DEG = Math.PI / 180;
const wrapErr = (a) => Math.atan2(Math.sin(a), Math.cos(a));

function run(key, Ctrl, seconds, { sp } = {}) {
  const sim = new Simulation(key, SYSTEMS[key].presets.tuned);
  sim.setMode(Ctrl === MPC ? "mpc" : "ilqr", new Ctrl(key));
  if (sp !== undefined) sim.setSetpoint(SYSTEMS[key].fromDisplay(sp));
  let maxU = -Infinity, minU = Infinity;
  for (let i = 0; i < seconds * 200; i++) {
    sim.step();
    maxU = Math.max(maxU, sim.last.u);
    minU = Math.min(minU, sim.last.u);
  }
  return { sim, maxU, minU };
}

test("the model step matches the plant's own derivative", () => {
  for (const key of Object.keys(MODELS)) {
    const s = SYSTEMS[key].init(0.3);
    const x = toVec(key, s);
    const d = SYSTEMS[key].deriv(s, 1);
    const x2 = step(key, x, 1, 1e-4);
    MODELS[key].keys.forEach((k, i) => {
      assert.ok(Math.abs((x2[i] - x[i]) / 1e-4 - d[k]) < 1e-3, `${key}.${k}`);
    });
  }
});

test("angle errors wrap the short way round", () => {
  const e = stateError("cartpole", [0, 0, 2 * Math.PI - 0.1, 0], [0, 0, 0, 0]);
  assert.ok(Math.abs(e[2] + 0.1) < 1e-12);
});

test("iLQR only ever lowers the cost, and respects the input limits", () => {
  const r = ilqr("cartpole", [0, 0, Math.PI, 0], 0, { N: 150, dt: 0.02, maxIter: 80 });
  for (let i = 1; i < r.costs.length; i++) assert.ok(r.costs[i] < r.costs[i - 1]);
  const [lo, hi] = SYSTEMS.cartpole.uLimit;
  assert.ok(r.U.every((u) => u >= lo - 1e-9 && u <= hi + 1e-9));
  assert.ok(r.costs[r.costs.length - 1] < r.costs[0] / 3, "barely improved");
});

test("iLQR swings the pendulum up from hanging and holds it", () => {
  const { sim } = run("cartpole", ILQRController, 6);
  assert.ok(Math.abs(wrapErr(sim.state.th)) < 3 * DEG, `pole at ${(wrapErr(sim.state.th) / DEG).toFixed(1)}°`);
  assert.ok(Math.abs(sim.state.w) < 0.2);
});

test("MPC swings the pendulum up from hanging and holds it", () => {
  const { sim } = run("cartpole", MPC, 8);
  assert.ok(Math.abs(wrapErr(sim.state.th)) < 3 * DEG, `pole at ${(wrapErr(sim.state.th) / DEG).toFixed(1)}°`);
});

for (const Ctrl of [MPC, ILQRController]) {
  test(`${Ctrl.name} brings the drone and the arm to their targets without overshoot past 10%`, () => {
    const d = run("drone", Ctrl, 6, { sp: 6 });
    assert.ok(Math.abs(d.sim.state.y - 6) < 0.05, `drone at ${d.sim.state.y}`);
    assert.ok(d.sim.metrics().overshoot < 10);
    assert.ok(d.minU >= 0 && d.maxU <= 25, "thrust left its limits");
    const a = run("arm", Ctrl, 6, { sp: 120 });
    assert.ok(Math.abs(a.sim.state.th - 120 * DEG) < 1 * DEG, `arm at ${a.sim.state.th / DEG}°, not 120° the short way`);
    assert.ok(a.minU >= -14 && a.maxU <= 14, "torque left its limits");
  });
}

test("a kick makes iLQR re-plan, and it recovers", () => {
  const { sim } = run("drone", ILQRController, 4, { sp: 5 });
  sim.kick();
  for (let i = 0; i < 4 * 200; i++) sim.step();
  assert.ok(Math.abs(sim.state.y - 5) < 0.05);
});

test("the holding LQR gain stabilizes the upright pendulum", () => {
  const K = lqrGain("cartpole", 0, 0.02);
  let x = [0, 0, 0.1, 0];
  for (let i = 0; i < 300; i++) {
    const u = Math.max(-40, Math.min(40, K.reduce((s, k, j) => s + k * x[j], 0)));
    x = step("cartpole", x, u, 0.02);
  }
  assert.ok(Math.abs(x[2]) < 0.01);
});

test("PID mode is unchanged: the pole still lies on the floor when it falls", () => {
  const sim = new Simulation("cartpole", { kp: 0, ki: 0, kd: 0 });
  for (let i = 0; i < 600; i++) sim.step();
  assert.ok(Math.abs(sim.state.th) <= 95 * DEG + 1e-9);
});

test("LQR's gain doesn't depend on the time step much, and the cart gain isn't lost", () => {
  // The plain Riccati loop used to blow up on the pendulum after converging,
  // which zeroed the cart-position gain.
  const a = lqrGain("cartpole", 0, 0.01), b = lqrGain("cartpole", 0, 0.03);
  assert.ok(a[0] > 5 && b[0] > 5, `cart gains ${a[0]}, ${b[0]}`);
  for (let i = 0; i < 4; i++) assert.ok(Math.abs(a[i] - b[i]) / Math.abs(a[i]) < 0.25, `entry ${i}`);
});

test("LQR holds every system at its target and rides out a kick", () => {
  const targets = { drone: 7, arm: 75, cartpole: 0 };
  for (const key of Object.keys(targets)) {
    const sys = SYSTEMS[key];
    const sim = new Simulation(key, sys.presets.tuned);
    sim.setMode("lqr", new LQRController(key));
    assert.equal(sim.free, false, "LQR starts near the goal, not swung down");
    sim.setSetpoint(sys.fromDisplay(targets[key]));
    for (let i = 0; i < 8 * 200; i++) sim.step();
    assert.ok(Math.abs(sys.measure(sim.state) - sim.sp) < sys.band, `${key} settles`);
    sim.kick();
    for (let i = 0; i < 6 * 200; i++) sim.step();
    assert.ok(Math.abs(sys.measure(sim.state) - sim.sp) < sys.band, `${key} recovers`);
    if (key === "cartpole") assert.ok(Math.abs(sim.state.x) < 0.2, "the cart comes back to the middle");
  }
});
