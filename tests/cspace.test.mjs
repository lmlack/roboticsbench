import { test } from "node:test";
import assert from "node:assert/strict";
import {
  ARM, CS_N, CS_PRESETS, angleOf, armHits, buildCSpace, fk, indexOf, planCSpace, poseAlong, segObstacleDist,
} from "../js/cspace.js";

const DEG = Math.PI / 180;
const close = (a, b, tol = 1e-9) => Math.abs(a - b) <= tol;

test("forward kinematics: straight out, folded, and at 90°", () => {
  let p = fk(ARM, 0, 0);
  assert.ok(close(p.tip.x, ARM.l1 + ARM.l2) && close(p.tip.y, 0));
  p = fk(ARM, 0, Math.PI);
  assert.ok(close(p.tip.x, ARM.l1 - ARM.l2) && close(p.tip.y, 0, 1e-12));
  p = fk(ARM, 90 * DEG, -90 * DEG);
  assert.ok(close(p.elbow.x, 0, 1e-12) && close(p.elbow.y, ARM.l1));
  assert.ok(close(p.tip.x, ARM.l2) && close(p.tip.y, ARM.l1));
});

test("segment to obstacle distances", () => {
  const circle = { type: "circle", x: 0, y: 1, r: 0.25 };
  assert.ok(close(segObstacleDist(-1, 0, 1, 0, circle), 0.75));
  assert.equal(segObstacleDist(-1, 1, 1, 1, circle), 0);
  const box = { type: "box", x: 2, y: 0, w: 1, h: 1 };
  assert.ok(close(segObstacleDist(0, 0, 1, 0, box), 0.5));
  assert.equal(segObstacleDist(0, 0, 3, 0, box), 0); // passes straight through
  // Off a corner: the closest pair is the segment's end (1, 1) and the corner (1.5, 0.5).
  assert.ok(close(segObstacleDist(0, 2, 1, 1, box), Math.hypot(0.5, 0.5)));
  // Matches dense sampling on random segments.
  let seed = 3;
  const rand = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  for (let k = 0; k < 300; k++) {
    const a = [rand() * 6 - 3, rand() * 6 - 3], b = [rand() * 6 - 3, rand() * 6 - 3];
    const o = k % 2 ? box : circle;
    let best = Infinity;
    for (let t = 0; t <= 1; t += 1 / 4000) {
      const x = a[0] + (b[0] - a[0]) * t, y = a[1] + (b[1] - a[1]) * t;
      const d = o.type === "circle"
        ? Math.max(0, Math.hypot(x - o.x, y - o.y) - o.r)
        : Math.hypot(Math.max(Math.abs(x - o.x) - o.w / 2, 0), Math.max(Math.abs(y - o.y) - o.h / 2, 0));
      best = Math.min(best, d);
    }
    assert.ok(Math.abs(segObstacleDist(a[0], a[1], b[0], b[1], o) - best) < 3e-3, `segment ${k}`);
  }
});

test("angles and cell indices round trip, wrapping at ±180°", () => {
  for (let i = 0; i < CS_N; i++) assert.equal(indexOf(angleOf(i, CS_N), CS_N), i);
  assert.equal(indexOf(Math.PI, CS_N), 0);
  assert.equal(indexOf(-Math.PI, CS_N), 0);
  assert.equal(indexOf(3 * Math.PI - 1e-9, CS_N), CS_N - 1);
});

test("an obstacle link 1 can reach blocks every θ2 for that θ1", () => {
  const cs = buildCSpace(ARM, [{ type: "circle", x: 0.6, y: 0, r: 0.15 }]);
  const i = indexOf(0, CS_N);
  for (let j = 0; j < CS_N; j++) assert.equal(cs.cells[j * CS_N + i], 1);
  // An obstacle only link 2 can reach leaves some θ2 free at every θ1.
  const far = buildCSpace(ARM, [{ type: "circle", x: 1.6, y: 0, r: 0.1 }]);
  for (let i2 = 0; i2 < CS_N; i2++) {
    let free = 0;
    for (let j = 0; j < CS_N; j++) free += 1 - far.cells[j * CS_N + i2];
    assert.ok(free > 0, `column ${i2}`);
  }
});

test("every layout has a path, and the arm never touches anything along it", () => {
  for (const [name, p] of Object.entries(CS_PRESETS)) {
    const cs = buildCSpace(ARM, p.obstacles);
    const r = planCSpace(cs, p.start, p.goal);
    assert.ok(r.found, name);
    assert.ok(close(r.path[0].t1, p.start.t1) && close(r.path[0].t2, p.start.t2), name);
    const end = r.path.at(-1);
    const same = (a, b) => close(Math.cos(a), Math.cos(b), 1e-9) && close(Math.sin(a), Math.sin(b), 1e-9);
    assert.ok(same(end.t1, p.goal.t1) && same(end.t2, p.goal.t2), name);
    // The padding is what makes this hold between cell centers, not only at them.
    for (let u = 0; u <= 1; u += 1 / 5000) {
      const q = poseAlong(r.path, u);
      assert.ok(!armHits(ARM, p.obstacles, q.t1, q.t2), `${name} at ${u.toFixed(4)}`);
    }
    // Steps are between neighboring cells: at most one cell per joint.
    for (let m = 1; m < r.path.length; m++) {
      assert.ok(Math.abs(r.path[m].t1 - r.path[m - 1].t1) <= (2 * Math.PI) / CS_N + 1e-9, name);
      assert.ok(Math.abs(r.path[m].t2 - r.path[m - 1].t2) <= (2 * Math.PI) / CS_N + 1e-9, name);
    }
  }
});

test("A* on the torus matches Dijkstra, and takes the short way round", () => {
  const cs = buildCSpace(ARM, []);
  const n = CS_N, step = (2 * Math.PI) / n;
  // From just left of +180° to just right of -180°: two cells apart across the seam.
  const r = planCSpace(cs, { t1: angleOf(n - 1, n), t2: 0 }, { t1: angleOf(1, n), t2: 0 });
  assert.ok(r.found);
  assert.ok(close(r.gridCost, 2 * step, 1e-9));
  assert.ok(r.path.some((q) => q.t1 > Math.PI), "the path crosses +180° rather than going the long way");

  const p = CS_PRESETS.shelf;
  const shelf = buildCSpace(ARM, p.obstacles);
  const a = planCSpace(shelf, p.start, p.goal);
  // Dijkstra on the same torus grid.
  const N = n * n, dist = new Float64Array(N).fill(Infinity), done = new Uint8Array(N);
  const s = indexOf(p.start.t2, n) * n + indexOf(p.start.t1, n), t = indexOf(p.goal.t2, n) * n + indexOf(p.goal.t1, n);
  dist[s] = 0;
  const queue = [[0, s]];
  while (queue.length) {
    queue.sort((x, y) => x[0] - y[0]);
    const [d, k] = queue.shift();
    if (done[k]) continue;
    done[k] = 1;
    if (k === t) break;
    const x = k % n, y = (k - x) / n;
    for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]]) {
      const nx = (x + dx + n) % n, ny = (y + dy + n) % n, m = ny * n + nx;
      if (shelf.cells[m]) continue;
      if (dx && dy && (shelf.cells[y * n + nx] || shelf.cells[ny * n + x])) continue;
      const nd = d + (dx && dy ? Math.SQRT2 : 1) * step;
      if (nd < dist[m] - 1e-12) {
        dist[m] = nd;
        queue.push([nd, m]);
      }
    }
  }
  assert.ok(close(a.gridCost, dist[t], 1e-9));
});

test("a blocked start or goal, or a walled-off goal, is reported", () => {
  const p = CS_PRESETS.posts;
  const cs = buildCSpace(ARM, p.obstacles);
  // Point link 1 straight at the first post.
  const post = p.obstacles[0];
  const into = { t1: Math.atan2(post.y, post.x), t2: 0 };
  assert.equal(planCSpace(cs, into, p.goal).blockedEnd, "start");
  assert.equal(planCSpace(cs, p.start, into).blockedEnd, "goal");
  // Two posts link 1 hits cut θ1 into two pieces.
  const cut = buildCSpace(ARM, [{ type: "circle", x: 0.7, y: 0, r: 0.1 }, { type: "circle", x: -0.7, y: 0, r: 0.1 }]);
  const r = planCSpace(cut, { t1: 90 * DEG, t2: 0 }, { t1: -90 * DEG, t2: 0 });
  assert.equal(r.found, false);
  assert.equal(r.blockedEnd, null);
});
