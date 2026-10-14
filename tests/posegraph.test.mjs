import { test } from "node:test";
import assert from "node:assert/strict";
import { Localization } from "../js/localization-sim.js";
import { ArrowSystem, MAX_POSES, PoseGraphSLAM, compose, invSPD } from "../js/posegraph.js";

function drive(seconds, opts = {}, setup = () => {}) {
  const sim = new Localization({ seed: 7, walls: [], ...opts });
  setup(sim);
  for (let i = 0; i < seconds / 0.05; i++) sim.step();
  return sim;
}

test("the arrow-shaped solve matches a plain dense solve", () => {
  const n = 9, m = 4, N = n + m;
  const sys = new ArrowSystem(n, m);
  let seed = 3;
  const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647) - 0.5;
  const H = Array.from({ length: N }, () => Array(N).fill(0));
  const info = [[2, 0.3, 0], [0.3, 1, 0], [0, 0, 1.5]];
  for (let k = 0; k < 6; k++) {
    // Two odometry-like edges between neighboring poses, four pose-landmark edges.
    const idx = k < 2
      ? [3 * k, 3 * k + 1, 3 * k + 2, 3 * k + 3, 3 * k + 4, 3 * k + 5]
      : [3 * (k % 3), 3 * (k % 3) + 1, 3 * (k % 3) + 2, n + 2 * (k % 2), n + 2 * (k % 2) + 1];
    const J = Array.from({ length: 3 }, () => idx.map(() => rnd()));
    sys.addEdge(idx, J, info, [rnd(), rnd(), rnd()]);
    for (let a = 0; a < idx.length; a++) {
      for (let b = 0; b < idx.length; b++) {
        let h = 0;
        for (let t = 0; t < 3; t++) for (let u = 0; u < 3; u++) h += J[t][a] * info[t][u] * J[u][b];
        H[idx[a]][idx[b]] += h;
      }
    }
  }
  for (let v = 0; v < N; v++) {
    sys.addH(v, v, 1);
    H[v][v] += 1;
  }
  const g = Array.from(sys.g);
  const sol = sys.solve();
  const Hi = invSPD(H);
  const dense = Hi.map((row) => -row.reduce((s, v, j) => s + v * g[j], 0));
  [...sol.dp, ...sol.dl].forEach((v, i) => assert.ok(Math.abs(v - dense[i]) < 1e-12));
  // Marginals: a pose's block of H⁻¹, and the landmarks' block from S⁻¹.
  const pc = sys.poseCov(3, sol.Y, sol.Si);
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) assert.ok(Math.abs(pc[r][c] - Hi[3 + r][3 + c]) < 1e-12);
  assert.ok(Math.abs(sol.Si[0][1] - Hi[n][n + 1]) < 1e-12);
});

test("odometry error is zero for a pose exactly where odometry put it", () => {
  const pg = new PoseGraphSLAM();
  const a = { x: 1, y: 2, th: 0.7 };
  const z = { x: 0.4, y: -0.1, th: 0.3 };
  const { e } = pg.odoError(a, compose(a, z), z);
  for (const v of e) assert.ok(Math.abs(v) < 1e-12);
});

test("graph SLAM maps every landmark close to where it is and tracks the robot", () => {
  const sim = drive(90);
  const map = sim.graph.map();
  assert.ok(map.length >= sim.landmarks.length - 1, `mapped ${map.length} of ${sim.landmarks.length}`);
  for (const m of map) {
    const t = sim.landmarks[m.j];
    const err = Math.hypot(m.x - t.x, m.y - t.y);
    assert.ok(err < 0.35, `landmark ${m.j} is ${err.toFixed(2)} m off`);
  }
  assert.ok(sim.errors().graph.pos < 0.3, `graph ${sim.errors().graph.pos.toFixed(3)} m off`);
});

test("graph SLAM closes loops and beats dead reckoning on noisy odometry", () => {
  const sim = drive(120, { walls: undefined }, (s) => (s.params.odomNoise = 3));
  const e = sim.errors();
  assert.ok(sim.graph.closures.length > 0, "no loop closure");
  assert.ok(e.graph.pos < e.dr.pos / 2, `graph ${e.graph.pos.toFixed(2)} vs DR ${e.dr.pos.toFixed(2)}`);
});

test("a long run thins old poses and keeps tracking", () => {
  const sim = drive(220, { walls: undefined });
  assert.ok(sim.graph.poses.length <= MAX_POSES + 1, `${sim.graph.poses.length} poses`);
  assert.equal(sim.graph.odo.length, sim.graph.poses.length - 1);
  for (const s of sim.graph.sights) assert.ok(s.i < sim.graph.poses.length && s.l < sim.graph.landmarks.length);
  assert.ok(sim.errors().graph.pos < 0.3, `graph ${sim.errors().graph.pos.toFixed(3)} m off`);
});

test("its covariances are symmetric and positive", () => {
  const g = drive(60).graph;
  assert.ok(g.P[0][0] > 0 && g.P[1][1] > 0 && g.P[2][2] > 0);
  assert.ok(Math.abs(g.P[0][1] - g.P[1][0]) < 1e-9);
  for (const l of g.landmarks) {
    assert.ok(l.P[0][0] > 0 && l.P[1][1] > 0);
    assert.ok(l.P[0][0] * l.P[1][1] - l.P[0][1] * l.P[1][0] > 0);
  }
});

test("removing a landmark drops it and its sightings from the graph", () => {
  const sim = drive(60);
  const before = sim.graph.landmarks.length;
  sim.removeLandmark(0);
  const g = sim.graph;
  assert.equal(g.landmarks.length, before - 1);
  for (const s of g.sights) assert.ok(s.l < g.landmarks.length);
  for (const l of g.landmarks) assert.ok(l.j < sim.landmarks.length);
  for (let i = 0; i < 40; i++) sim.step();
  assert.ok(sim.errors().graph.pos < 0.3);
});

test("look-alike landmarks: no wrong matches while every estimator is tracking", () => {
  for (const seed of [100, 101, 102]) {
    const sim = drive(30, { seed, walls: undefined }, (s) => (s.params.lookAlike = true));
    for (const k of ["ekf", "ukf", "slam", "graph"]) assert.equal(sim.wrongCount(k), 0, `${k}, seed ${seed}`);
    assert.equal(sim.slam.ids.length, new Set(sim.slam.ids).size, "EKF-SLAM mapped a landmark twice");
    assert.ok(sim.errors().ekf.pos < 0.1);
  }
});

test("look-alike landmarks: after a reset the particle filter recovers and the EKF matches wrongly", () => {
  const sim = drive(30, { seed: 1, walls: undefined }, (s) => (s.params.lookAlike = true));
  sim.resetFilters();
  for (let i = 0; i < 40 / 0.05; i++) sim.step();
  const e = sim.errors();
  assert.ok(e.pf.pos < 0.3, `PF ${e.pf.pos.toFixed(2)} m off`);
  assert.ok(e.ekf.pos > 1, `EKF ${e.ekf.pos.toFixed(2)} m off`);
  assert.ok(sim.wrongCount("ekf") > 0);
});
