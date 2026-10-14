import { test } from "node:test";
import assert from "node:assert/strict";
import { EKFSLAM, Localization } from "../js/localization-sim.js";

function drive(seconds, opts = {}) {
  const sim = new Localization({ seed: 7, walls: [], ...opts });
  for (let i = 0; i < seconds / 0.05; i++) sim.step();
  return sim;
}

test("SLAM maps every landmark it has seen, close to where it really is", () => {
  const sim = drive(90);
  const map = sim.slam.map();
  assert.ok(map.length >= sim.landmarks.length - 1, `mapped ${map.length} of ${sim.landmarks.length}`);
  for (const m of map) {
    const t = sim.landmarks[m.j];
    const err = Math.hypot(m.x - t.x, m.y - t.y);
    assert.ok(err < 0.35, `landmark ${m.j} is ${err.toFixed(2)} m off`);
  }
});

test("SLAM tracks the robot about as well as the EKF that is given the map", () => {
  const sim = drive(90);
  const e = sim.errors();
  assert.ok(e.slam.pos < 0.3, `SLAM ${e.slam.pos.toFixed(3)} m off`);
  assert.ok(e.slam.pos < Math.max(0.15, e.ekf.pos * 4), `SLAM ${e.slam.pos.toFixed(3)} vs EKF ${e.ekf.pos.toFixed(3)}`);
});

test("SLAM does far better than dead reckoning on the same odometry", () => {
  const sim = drive(120, { walls: undefined });
  const e = sim.errors();
  assert.ok(e.slam.pos < e.dr.pos / 2, `SLAM ${e.slam.pos.toFixed(2)} vs DR ${e.dr.pos.toFixed(2)}`);
});

test("the covariance stays symmetric with a positive diagonal", () => {
  const P = drive(60).slam.P;
  for (let i = 0; i < P.length; i++) {
    assert.ok(P[i][i] > 0);
    for (let j = 0; j < P.length; j++) assert.ok(Math.abs(P[i][j] - P[j][i]) < 1e-9);
  }
});

test("a landmark added on the fly shows up in the map once seen", () => {
  const sim = drive(10);
  const before = sim.slam.ids.length;
  sim.addLandmark(sim.truth.x + 1, sim.truth.y);
  for (let i = 0; i < 20; i++) sim.step();
  assert.ok(sim.slam.ids.includes(sim.landmarks.length - 1));
  assert.ok(sim.slam.ids.length > before);
});

test("removing a landmark drops it from the map and keeps the others matched", () => {
  const sim = drive(60);
  const keep = sim.slam.map().filter((m) => m.j !== 0).map((m) => ({ ...sim.landmarks[m.j], est: [m.x, m.y] }));
  sim.removeLandmark(0);
  const map = sim.slam.map();
  assert.ok(!map.some((m) => m.j >= sim.landmarks.length));
  for (const k of keep) {
    const m = map.find((mm) => sim.landmarks[mm.j].x === k.x && sim.landmarks[mm.j].y === k.y);
    assert.ok(m, "a landmark lost its place in the map");
    assert.ok(Math.hypot(m.x - k.est[0], m.y - k.est[1]) < 1e-9);
  }
});

test("seeing a mapped landmark again shrinks the map's uncertainty", () => {
  // A hand-built check: two landmarks, the second seen once, then the first
  // re-observed. The second landmark's variance must drop even though it is
  // not in view, through its correlation with the robot.
  const s = new EKFSLAM();
  s.init({ x: 0, y: 0, th: 0 }, 0.01, 0.01, 0.01);
  s.update([{ j: 0, r: 2, b: 0 }], 0.05, 0.01);
  for (let i = 0; i < 20; i++) s.predict(0.5, 0, 0.1, 3);
  s.update([{ j: 1, r: 2, b: Math.PI / 2 }], 0.05, 0.01);
  const before = s.map()[1].rx;
  s.update([{ j: 0, r: 1, b: 0 }], 0.05, 0.01); // landmark 0 is 1 m dead ahead now
  assert.ok(s.map()[1].rx < before, `${s.map()[1].rx} vs ${before}`);
});
