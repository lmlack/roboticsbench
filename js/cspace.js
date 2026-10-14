// Configuration space for a 2-link arm seen from above: which joint angles
// make the arm hit an obstacle, and A* through the angles that don't. Pure
// JS, no DOM, so it can be checked under node --test.
//
// Angles run from -π to π and wrap, so the grid is a torus: leaving the right
// edge comes back on the left, and the top meets the bottom.

import { MinHeap } from "./planning-sim.js";

export const ARM = { l1: 1.0, l2: 0.8, width: 0.06 }; // meters; width is half the link's thickness
export const VIEW = 2.1; // the room is the square from -VIEW to VIEW
export const CS_N = 120; // cells per joint: 3° each

const TAU = 2 * Math.PI;
const wrap = (a) => a - TAU * Math.floor((a + Math.PI) / TAU);

/** Where the elbow and tip are for joint angles t1, t2. */
export function fk(arm, t1, t2) {
  const ex = arm.l1 * Math.cos(t1), ey = arm.l1 * Math.sin(t1);
  return {
    elbow: { x: ex, y: ey },
    tip: { x: ex + arm.l2 * Math.cos(t1 + t2), y: ey + arm.l2 * Math.sin(t1 + t2) },
  };
}

// ---- Distances ----

export function pointSegDist(px, py, ax, ay, bx, by) {
  const dx = bx - ax, dy = by - ay;
  const L = dx * dx + dy * dy;
  const t = L ? Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / L)) : 0;
  return Math.hypot(px - ax - t * dx, py - ay - t * dy);
}

function pointBoxDist(px, py, b) {
  const dx = Math.max(Math.abs(px - b.x) - b.w / 2, 0);
  const dy = Math.max(Math.abs(py - b.y) - b.h / 2, 0);
  return Math.hypot(dx, dy);
}

/** Does the segment cross the box? Liang–Barsky clipping. */
function segCrossesBox(ax, ay, bx, by, b) {
  let t0 = 0, t1 = 1;
  const dx = bx - ax, dy = by - ay;
  const sides = [
    [-dx, ax - (b.x - b.w / 2)], [dx, b.x + b.w / 2 - ax],
    [-dy, ay - (b.y - b.h / 2)], [dy, b.y + b.h / 2 - ay],
  ];
  for (const [p, q] of sides) {
    if (p === 0) {
      if (q < 0) return false;
    } else {
      const r = q / p;
      if (p < 0) t0 = Math.max(t0, r);
      else t1 = Math.min(t1, r);
      if (t0 > t1) return false;
    }
  }
  return true;
}

/**
 * Distance from a segment to an obstacle, 0 if they touch. For a box, when
 * the two don't cross, the closest pair always includes a segment end or a
 * box corner, so checking those is exact.
 */
export function segObstacleDist(ax, ay, bx, by, o) {
  if (o.type === "circle") return Math.max(0, pointSegDist(o.x, o.y, ax, ay, bx, by) - o.r);
  if (segCrossesBox(ax, ay, bx, by, o)) return 0;
  let d = Math.min(pointBoxDist(ax, ay, o), pointBoxDist(bx, by, o));
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
    d = Math.min(d, pointSegDist(o.x + (sx * o.w) / 2, o.y + (sy * o.h) / 2, ax, ay, bx, by));
  }
  return d;
}

/** True if the arm at (t1, t2), fattened by `pad`, touches any obstacle. */
export function armHits(arm, obstacles, t1, t2, pad = 0) {
  const { elbow, tip } = fk(arm, t1, t2);
  const reach = arm.width + pad;
  for (const o of obstacles) {
    if (segObstacleDist(0, 0, elbow.x, elbow.y, o) <= reach) return true;
    if (segObstacleDist(elbow.x, elbow.y, tip.x, tip.y, o) <= reach) return true;
  }
  return false;
}

// ---- The grid ----

export const angleOf = (i, n) => -Math.PI + ((i + 0.5) * TAU) / n;
export const indexOf = (a, n) => Math.min(n - 1, Math.floor(((wrap(a) + Math.PI) / TAU) * n));

/**
 * How far any point on the arm can move while the joints stay inside one
 * cell and step to a neighbor: each joint moves at most a cell's width, and a
 * point at distance ℓ from a joint moves at most ℓ times that. Padding the
 * check by this much means a path through free cells is free all the way.
 */
export const cellPad = (arm, n) => (2 * (arm.l1 + arm.l2) * Math.PI) / n;

/** cells[j * n + i] is 1 where θ1 = angleOf(i), θ2 = angleOf(j) hits something. */
export function buildCSpace(arm, obstacles, n = CS_N) {
  const cells = new Uint8Array(n * n);
  const pad = cellPad(arm, n);
  let blocked = 0;
  for (let j = 0; j < n; j++) {
    const t2 = angleOf(j, n);
    for (let i = 0; i < n; i++) {
      if (armHits(arm, obstacles, angleOf(i, n), t2, pad)) {
        cells[j * n + i] = 1;
        blocked++;
      }
    }
  }
  return { n, cells, pad, blockedShare: blocked / (n * n) };
}

// ---- A* on the torus ----

const MOVES = [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]];

/** Shortest way around between two cell indices on a ring of n. */
const ringGap = (a, b, n) => {
  const d = Math.abs(a - b) % n;
  return Math.min(d, n - d);
};

/**
 * A* from one pose to another through free cells, 8-connected, wrapping at
 * ±180° on both joints. Cost is distance in joint space, in radians. The
 * heuristic is the octile distance the short way round, which never
 * overestimates, so the path is the shortest the grid allows.
 */
export function planCSpace(cs, startQ, goalQ) {
  const { n, cells } = cs;
  const step = TAU / n;
  const si = indexOf(startQ.t1, n), sj = indexOf(startQ.t2, n);
  const gi = indexOf(goalQ.t1, n), gj = indexOf(goalQ.t2, n);
  const s = sj * n + si, t = gj * n + gi;
  const closed = new Uint8Array(n * n);
  const out = { found: false, blockedEnd: null, path: null, cells: null, cost: Infinity, expanded: 0, closed };
  if (cells[s]) return { ...out, blockedEnd: "start" };
  if (cells[t]) return { ...out, blockedEnd: "goal" };

  const h = (k) => {
    const dx = ringGap(k % n, gi, n), dy = ringGap(Math.floor(k / n), gj, n);
    return step * (Math.max(dx, dy) + (Math.SQRT2 - 1) * Math.min(dx, dy));
  };
  const g = new Float64Array(n * n).fill(Infinity);
  const parent = new Int32Array(n * n).fill(-1);
  const heap = new MinHeap();
  let tie = 0;
  g[s] = 0;
  heap.push(s, h(s), tie++);
  while (heap.size) {
    const k = heap.pop().id;
    if (closed[k]) continue;
    closed[k] = 1;
    out.expanded++;
    if (k === t) break;
    const x = k % n, y = (k - x) / n;
    for (const [dx, dy] of MOVES) {
      const nx = (x + dx + n) % n, ny = (y + dy + n) % n;
      const m = ny * n + nx;
      if (cells[m] || closed[m]) continue;
      // No cutting a corner between two blocked cells.
      if (dx && dy && (cells[y * n + nx] || cells[ny * n + x])) continue;
      const ng = g[k] + (dx && dy ? Math.SQRT2 : 1) * step;
      if (ng < g[m] - 1e-12) {
        g[m] = ng;
        parent[m] = k;
        heap.push(m, ng + h(m), tie++);
      }
    }
  }
  if (!closed[t]) return out;

  const ks = [];
  for (let k = t; k >= 0; k = parent[k]) ks.push(k);
  ks.reverse();
  // Joint angles along the path, unwrapped so each step is the short way
  // round: the exact start pose, every cell center, then the exact goal
  // pose. Going through the start and goal cells' centers keeps every
  // straight piece inside cells that were checked.
  const path = [{ t1: startQ.t1, t2: startQ.t2 }];
  const pushNear = (a1, a2) => {
    const p = path[path.length - 1];
    path.push({ t1: p.t1 + wrap(a1 - p.t1), t2: p.t2 + wrap(a2 - p.t2) });
  };
  for (const k of ks) pushNear(angleOf(k % n, n), angleOf(Math.floor(k / n), n));
  pushNear(goalQ.t1, goalQ.t2);
  let cost = 0;
  for (let m = 1; m < path.length; m++) cost += Math.hypot(path[m].t1 - path[m - 1].t1, path[m].t2 - path[m - 1].t2);
  return { ...out, found: true, path, cells: ks, cost, gridCost: g[t] };
}

/** The pose a fraction u (0 to 1) of the way along a path, by joint distance. */
export function poseAlong(path, u) {
  const seg = [];
  let total = 0;
  for (let m = 1; m < path.length; m++) {
    const d = Math.hypot(path[m].t1 - path[m - 1].t1, path[m].t2 - path[m - 1].t2);
    seg.push(d);
    total += d;
  }
  let left = Math.max(0, Math.min(1, u)) * total;
  for (let m = 1; m < path.length; m++) {
    if (left <= seg[m - 1] || m === path.length - 1) {
      const f = seg[m - 1] ? Math.min(1, left / seg[m - 1]) : 1;
      return {
        t1: path[m - 1].t1 + (path[m].t1 - path[m - 1].t1) * f,
        t2: path[m - 1].t2 + (path[m].t2 - path[m - 1].t2) * f,
      };
    }
    left -= seg[m - 1];
  }
  return { ...path[0] };
}

// ---- Layouts ----

const DEG = Math.PI / 180;

export const CS_PRESETS = {
  posts: {
    name: "Two posts",
    obstacles: [
      { type: "circle", x: 1.15, y: 1.15, r: 0.28 },
      { type: "circle", x: -1.5, y: -0.5, r: 0.28 },
    ],
    start: { t1: -20 * DEG, t2: 30 * DEG },
    goal: { t1: 150 * DEG, t2: -40 * DEG },
  },
  shelf: {
    name: "Shelf",
    obstacles: [
      { type: "box", x: 0, y: 1.45, w: 1.8, h: 0.3 },
      { type: "box", x: 1.45, y: -0.4, w: 0.3, h: 1.0 },
      { type: "circle", x: -1.25, y: -0.95, r: 0.28 },
    ],
    start: { t1: -60 * DEG, t2: 0 },
    goal: { t1: 160 * DEG, t2: 60 * DEG },
  },
  gap: {
    name: "Narrow gap",
    obstacles: [
      { type: "box", x: 1.55, y: 0.62, w: 0.9, h: 0.3 },
      { type: "box", x: 1.55, y: -0.62, w: 0.9, h: 0.3 },
      { type: "circle", x: -1.2, y: 1.2, r: 0.3 },
    ],
    start: { t1: 150 * DEG, t2: 60 * DEG },
    goal: { t1: 0, t2: 0 },
  },
};
