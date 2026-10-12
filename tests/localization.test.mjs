import { test } from "node:test";
import assert from "node:assert/strict";
import {
  ALGO_KEYS, DEFAULT_WALLS, DT, GRID, Localization, ParticleFilter, RECOVERED, ROBOT_R, WALL_T, WAYPOINTS,
  chol3, distToSegment, makeGauss, mulberry32, planGrid, planPath, segmentToWall, systematicResample,
} from "../js/localization-sim.js";

// Default noise: odometry ×1 (about 0.65 m and 6° of dead-reckoning drift
// over 20 s), 0.1 m range and 2.9° bearing noise, 6 m sensor range. With the
// default walls about four of the five or six landmarks in range are in view.
// That is enough drift that a filter must use the landmarks, and enough
// landmarks that a working filter stays within a few centimeters (the grid
// within about 8 cm, a third of a cell). The 0.3 m bound leaves a wide margin,
// so it only fails if a filter is actually broken.
const SEEDS = [1, 2, 3, 4, 5];

function run(sim, seconds, each) {
  const n = Math.round(seconds / DT);
  for (let k = 0; k < n; k++) {
    sim.step();
    each?.(sim);
  }
}

test("both filters track the autopilot loop with default noise", () => {
  for (const seed of SEEDS) {
    const sim = new Localization({ seed });
    let pf = 0, ekf = 0, count = 0;
    run(sim, 25, (s) => {
      if (s.t < 5) return;
      const e = s.errors();
      pf += e.pf.pos;
      ekf += e.ekf.pos;
      count++;
    });
    assert.ok(pf / count < RECOVERED, `seed ${seed}: PF mean error ${pf / count}`);
    assert.ok(ekf / count < RECOVERED, `seed ${seed}: EKF mean error ${ekf / count}`);
  }
});

test("dead reckoning alone drifts, so the landmarks matter", () => {
  let drift = 0;
  for (const seed of SEEDS) {
    const sim = new Localization({ seed });
    sim.landmarks = [];
    run(sim, 20);
    drift += sim.errors().ekf.pos;
  }
  assert.ok(drift / SEEDS.length > 0.3, `mean drift ${drift / SEEDS.length}`);
});

test("particle weights stay finite and normalized, resampling keeps N", () => {
  const sim = new Localization({ seed: 7, particles: 500 });
  let resamples = 0;
  run(sim, 30, (s) => {
    const { w, n } = s.pf;
    assert.equal(w.length, 500);
    assert.equal(n, 500);
    let sum = 0;
    for (let i = 0; i < n; i++) {
      assert.ok(Number.isFinite(w[i]) && w[i] >= 0, `weight ${w[i]}`);
      sum += w[i];
    }
    assert.ok(Math.abs(sum - 1) < 1e-9, `sum ${sum}`);
    assert.ok(s.pf.neff >= 1 - 1e-9 && s.pf.neff <= n + 1e-9, `neff ${s.pf.neff}`);
    if (s.pf.resampled) {
      resamples++;
      assert.equal(s.pf.x.length, 500);
    }
  });
  assert.ok(resamples > 10, `resampled ${resamples} times`);

  // Tiny noise and many landmarks give likelihoods far below what a double
  // can hold. Log-weights must keep this finite.
  sim.params.sensorNoise = 0.005;
  run(sim, 2, (s) => {
    for (const v of s.pf.w) assert.ok(Number.isFinite(v));
    assert.ok(Number.isFinite(s.errors().pf.pos));
  });
});

test("resizing the particle set resamples to the new count", () => {
  const rng = mulberry32(3);
  const pf = new ParticleFilter(400, rng, makeGauss(rng));
  pf.initGaussian({ x: 4, y: 3, th: 1 }, 0.2, 0.1);
  for (const n of [50, 2000, 1000]) {
    pf.resize(n);
    assert.equal(pf.n, n);
    assert.equal(pf.x.length, n);
    assert.equal(pf.w.length, n);
    const e = pf.estimate();
    assert.ok(Math.abs(e.x - 4) < 0.1 && Math.abs(e.y - 3) < 0.1, `estimate after resize to ${n}`);
  }
});

test("systematic resampling gives the expected counts", () => {
  const counts = (w, n, u0) => {
    const c = new Array(w.length).fill(0);
    for (const i of systematicResample(Float64Array.from(w), n, u0)) c[i]++;
    return c;
  };
  // Pointers at 0.05, 0.15, ..., 0.95 against cumulative 0.1, 0.3, 0.6, 1.0.
  assert.deepEqual(counts([0.1, 0.2, 0.3, 0.4], 10, 0.05), [1, 2, 3, 4]);
  // Systematic resampling never strays a full copy from n × w for any offset
  // strictly inside (0, 1/n). At exactly 0 a pointer can sit on a boundary.
  const w = [0.05, 0.33, 0.02, 0.4, 0.2];
  for (const u0 of [0.001, 0.013, 0.05, 0.0999]) {
    const c = counts(w, 10, u0);
    assert.equal(c.reduce((a, b) => a + b), 10);
    c.forEach((k, i) => assert.ok(Math.abs(k - 10 * w[i]) < 1, `u0 ${u0}: index ${i} got ${k}`));
  }
  // Zero weights are never picked.
  assert.deepEqual(counts([0, 0.5, 0, 0.5], 4, 0.1), [0, 2, 0, 2]);
});

test("after a kidnap the PF recovers and the Gaussian filters do not", () => {
  for (const seed of SEEDS) {
    const sim = new Localization({ seed });
    run(sim, 20);
    sim.kidnap();
    assert.ok(sim.errors().pf.pos > 2, "the kidnap moved the robot well away");
    let recovered = null;
    const mean = { pf: 0, ekf: 0, ukf: 0, dr: 0 };
    let count = 0;
    run(sim, 10, (s) => {
      const e = s.errors();
      if (recovered === null && e.pf.pos < RECOVERED) recovered = s.t;
      for (const k of Object.keys(mean)) mean[k] += e[k].pos;
      count++;
    });
    for (const k of Object.keys(mean)) mean[k] /= count;
    const e = sim.errors();
    assert.ok(recovered !== null, `seed ${seed}: PF never got under ${RECOVERED} m`);
    assert.ok(e.pf.pos < RECOVERED, `seed ${seed}: PF error after 10 s ${e.pf.pos}`);
    // EKF and UKF reject every measurement through their gates and coast from
    // the old spot, like dead reckoning. Judged on the mean over the 10 s,
    // since the robot's loop can carry it back past a stale estimate.
    for (const k of ["ekf", "ukf", "dr"]) {
      assert.ok(mean[k] > 1, `seed ${seed}: ${k} mean error ${mean[k]}`);
      assert.ok(mean[k] > 5 * mean.pf, `seed ${seed}: ${k} ${mean[k]} vs PF ${mean.pf}`);
    }
  }
});


test("EKF and UKF covariances stay symmetric positive definite", () => {
  const sim = new Localization({ seed: 11 });
  const check = (s) => {
    for (const f of [s.ekf, s.ukf]) checkP(f.P);
  };
  const checkP = (P) => {
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        assert.ok(Number.isFinite(P[i][j]));
        assert.ok(Math.abs(P[i][j] - P[j][i]) < 1e-12, `asymmetric at ${i},${j}`);
      }
    }
    // Sylvester's criterion: all leading principal minors positive.
    const m1 = P[0][0];
    const m2 = P[0][0] * P[1][1] - P[0][1] * P[1][0];
    const m3 = P[0][0] * (P[1][1] * P[2][2] - P[1][2] * P[2][1])
      - P[0][1] * (P[1][0] * P[2][2] - P[1][2] * P[2][0])
      + P[0][2] * (P[1][0] * P[2][1] - P[1][1] * P[2][0]);
    assert.ok(m1 > 0 && m2 > 0 && m3 > 0, `minors ${m1} ${m2} ${m3}`);
  };
  run(sim, 20, check);
  // Stress it: tiny sensor noise, a broad prior, a kidnap, manual driving,
  // and a landmark taken away.
  sim.params.sensorNoise = 0.01;
  sim.resetFilters();
  run(sim, 10, check);
  sim.kidnap();
  run(sim, 5, check);
  sim.autopilot = false;
  sim.manual = { v: 0, w: 1.2 };
  sim.removeLandmark(0);
  run(sim, 5, check);
});

test("seeded runs are repeatable", () => {
  const a = new Localization({ seed: 42 });
  const b = new Localization({ seed: 42 });
  run(a, 5);
  run(b, 5);
  assert.deepEqual(a.truth, b.truth);
  assert.deepEqual(a.ekf.mu, b.ekf.mu);
  assert.deepEqual(a.pf.estimate(), b.pf.estimate());
});

// ---- Walls, planning and the other estimators ----

test("UKF tracks the loop as well as the EKF", () => {
  for (const seed of SEEDS) {
    const sim = new Localization({ seed });
    let ukf = 0, ekf = 0, count = 0;
    run(sim, 25, (s) => {
      if (s.t < 5) return;
      const e = s.errors();
      ukf += e.ukf.pos;
      ekf += e.ekf.pos;
      count++;
    });
    assert.ok(ukf / count < RECOVERED, `seed ${seed}: UKF mean error ${ukf / count}`);
    assert.ok(ukf / count < 1.2 * (ekf / count) + 0.01, `seed ${seed}: UKF ${ukf / count} vs EKF ${ekf / count}`);
  }
});

test("UKF sigma point weights sum to 1", () => {
  const { ukf } = new Localization({ seed: 1 });
  assert.ok(Math.abs(ukf.wm.reduce((a, b) => a + b) - 1) < 1e-12);
  assert.ok(ukf.wm.every((w) => w > 0), "all mean weights positive with λ = 1");
  assert.equal(ukf.chi.length, 7);
});

test("dead reckoning drifts", () => {
  let drift = 0;
  for (const seed of SEEDS) {
    const sim = new Localization({ seed });
    run(sim, 30);
    drift += sim.errors().dr.pos;
    assert.ok(sim.errors().pf.pos < RECOVERED, "the landmark filters still track");
  }
  assert.ok(drift / SEEDS.length > 0.3, `mean dead-reckoning error after 30 s ${drift / SEEDS.length}`);
});

test("grid filter converges on the loop and recovers from a kidnap", () => {
  for (const seed of SEEDS) {
    const sim = new Localization({ seed });
    let err = 0, count = 0;
    run(sim, 20, (s) => {
      if (s.t < 5) return;
      err += s.errors().grid.pos;
      count++;
    });
    assert.ok(err / count < RECOVERED, `seed ${seed}: grid mean error ${err / count}`);

    sim.kidnap();
    let recovered = null;
    run(sim, 10, (s) => {
      if (recovered === null && s.errors().grid.pos < RECOVERED) recovered = s.t;
    });
    assert.ok(recovered !== null, `seed ${seed}: grid never recovered`);
    assert.ok(sim.errors().grid.pos < RECOVERED, `seed ${seed}: grid error 10 s after kidnap ${sim.errors().grid.pos}`);
  }
});

test("grid converges from a uniform belief", () => {
  const sim = new Localization({ seed: 3 });
  run(sim, 8);
  sim.resetFilters();
  run(sim, 10);
  assert.ok(sim.errors().grid.pos < RECOVERED, `grid error ${sim.errors().grid.pos}`);
});

test("grid probabilities stay normalized, finite, and zero inside walls", () => {
  const sim = new Localization({ seed: 5 });
  const g = sim.grid;
  const plane = g.nx * g.ny;
  const check = () => {
    let sum = 0;
    for (let n = 0; n < g.N; n++) {
      const v = g.bel[n];
      assert.ok(Number.isFinite(v) && v >= 0, `cell ${n}: ${v}`);
      if (g.wall[n % plane]) assert.equal(v, 0, `wall cell ${n} holds ${v}`);
      sum += v;
    }
    assert.ok(Math.abs(sum - 1) < 1e-9, `sum ${sum}`);
  };
  let walled = 0;
  for (const c of g.wall) walled += c;
  assert.ok(walled > 0, "the default walls cover some cells");
  for (let k = 0; k < 6; k++) {
    run(sim, 5);
    check();
  }
  sim.kidnap();
  run(sim, 2);
  check();
  sim.resetFilters();
  check();
  sim.addWall(1, 1.5, 1, 5.5);
  check();
  run(sim, 2);
  check();
  assert.equal(GRID.every * DT, 0.2, "grid runs at 5 Hz");
});

test("a landmark behind a wall is never measured", () => {
  // Robot left of a wall, landmark right of it, both well in range.
  const sim = new Localization({ seed: 2, walls: [{ ax: 5, ay: 1, bx: 5, by: 6 }] });
  sim.landmarks = [{ x: 7, y: 3.5 }, { x: 3, y: 5.5 }];
  sim.truth = { x: 3, y: 3.5, th: 0 };
  sim.autopilot = false;
  for (let k = 0; k < 50; k++) {
    sim.step();
    assert.ok(!sim.z.some((m) => m.j === 0), "landmark behind the wall was measured");
    assert.ok(sim.z.some((m) => m.j === 1), "the visible landmark is measured");
    assert.deepEqual(sim.blocked, [0]);
  }
  // And on the default map over a long autopilot run, no measurement ever
  // crosses a wall.
  const loop = new Localization({ seed: 4 });
  let hidden = 0;
  run(loop, 40, (s) => {
    hidden += s.blocked.length;
    for (const m of s.z) {
      const lm = s.landmarks[m.j];
      for (const w of s.walls) assert.ok(segmentToWall(s.truth.x, s.truth.y, lm.x, lm.y, w) >= WALL_T / 2);
    }
  });
  assert.ok(hidden > 0, "the default walls hide landmarks somewhere on the loop");
});

test("the true robot never enters a wall", () => {
  const clear = ROBOT_R + WALL_T / 2 - 1e-9;
  const inside = (s) => {
    const { x, y } = s.truth;
    for (const w of s.walls) assert.ok(distToSegment(x, y, w) >= clear, `robot at ${x}, ${y} inside a wall`);
    assert.ok(x >= ROBOT_R && y >= ROBOT_R && x <= 10 - ROBOT_R && y <= 7 - ROBOT_R, "robot left the map");
  };
  const sim = new Localization({ seed: 6 });
  run(sim, 60, inside);
  // Drive straight at walls and the map edge, turning now and then.
  sim.autopilot = false;
  const cmds = [{ v: 0.8, w: 0 }, { v: 0.8, w: 0.4 }, { v: -0.8, w: 0 }, { v: 0.8, w: -1.2 }, { v: 0.8, w: 0 }];
  let k = 0;
  run(sim, 120, (s) => {
    s.manual = cmds[Math.floor(k++ * DT / 4) % cmds.length];
    inside(s);
  });
  assert.ok(sim.bumps > 0, "it actually hit something");

  // Kidnaps and new landmarks never land inside a wall.
  for (let i = 0; i < 50; i++) {
    sim.kidnap();
    inside(sim);
  }
  assert.equal(sim.addLandmark(3.4, 3.5), false);
  assert.equal(sim.addLandmark(4.5, 3.5), true);
});

test("a plan exists between consecutive waypoints on the default layout", () => {
  const grid = planGrid(DEFAULT_WALLS);
  WAYPOINTS.forEach(([ax, ay], i) => {
    const [bx, by] = WAYPOINTS[(i + 1) % WAYPOINTS.length];
    const path = planPath(grid, { x: ax, y: ay }, { x: bx, y: by });
    assert.ok(path, `no plan from waypoint ${i} to ${i + 1}`);
    // Every path point keeps the robot clear of the walls.
    for (const p of path.slice(0, -1)) {
      for (const w of DEFAULT_WALLS) assert.ok(distToSegment(p.x, p.y, w) > ROBOT_R + WALL_T / 2);
    }
  });
});

test("the autopilot keeps visiting waypoints and never gets stuck", () => {
  for (const seed of SEEDS) {
    const sim = new Localization({ seed });
    let lastVisit = 0, longest = 0, visits = 0;
    let lastPos = { ...sim.truth }, still = 0;
    run(sim, 60, (s) => {
      if (s.visits !== visits) {
        visits = s.visits;
        longest = Math.max(longest, s.t - lastVisit);
        lastVisit = s.t;
      }
      // Never parked: it moves at least 5 cm in every 2 s window.
      if (Math.round(s.t / DT) % 40 === 0) {
        if (Math.hypot(s.truth.x - lastPos.x, s.truth.y - lastPos.y) < 0.05) still++;
        lastPos = { ...s.truth };
      }
    });
    longest = Math.max(longest, sim.t - lastVisit);
    assert.ok(visits >= 10, `seed ${seed}: only ${visits} waypoints in 60 s`);
    assert.ok(longest < 12, `seed ${seed}: ${longest} s between waypoints`);
    assert.equal(still, 0, `seed ${seed}: parked`);
  }

  // A wall dropped across the loop: it replans around it or skips the
  // waypoint, and keeps going.
  const sim = new Localization({ seed: 2 });
  run(sim, 5);
  sim.addWall(4, 0.2, 4, 3.2);
  sim.addWall(0.2, 4.2, 2.8, 4.2);
  const before = sim.visits;
  run(sim, 60);
  assert.ok(sim.visits - before >= 8, `only ${sim.visits - before} waypoints after new walls`);
});

test("every estimator reports a finite pose", () => {
  const sim = new Localization({ seed: 8 });
  run(sim, 5);
  sim.kidnap();
  run(sim, 5);
  const e = sim.errors();
  for (const k of ALGO_KEYS) {
    assert.ok(Number.isFinite(e[k].pos) && Number.isFinite(e[k].head), k);
  }
  assert.ok(chol3([[1, 0, 0], [0, 1, 0], [0, 0, 1]]));
  assert.equal(chol3([[1, 2, 0], [2, 1, 0], [0, 0, 1]]), null);
});

test("Start over restores the default walls", async () => {
  const { Localization, DEFAULT_WALLS } = await import("../js/localization-sim.js");
  const sim = new Localization({ seed: 2 });
  sim.clearWalls();
  assert.equal(sim.walls.length, 0);
  sim.restoreWalls();
  sim.reset();
  assert.deepEqual(sim.walls, DEFAULT_WALLS);
  assert.notEqual(sim.walls[0], DEFAULT_WALLS[0], "a copy, so later edits leave the defaults alone");
});
