// Motion planning on an occupancy grid: grid searches (BFS, Dijkstra, greedy,
// A*, weighted A*) and sampling planners (RRT, RRT*). Pure JS, no DOM, so the
// algorithms can be checked under node --test.
//
// Coordinates: cell (x, y) covers [x, x + 1) by [y, y + 1). Grid paths run
// through cell centers, so grid and tree paths share one collision check.

export const GRID_W = 61;
export const GRID_H = 41;
const SQRT2 = Math.SQRT2;

export const ALGOS = {
  bfs: {
    name: "BFS", kind: "grid",
    blurb: "Expands cells in the order it finds them. It finds the fewest moves, which is the shortest path only when every move costs the same.",
  },
  dijkstra: {
    name: "Dijkstra", kind: "grid",
    blurb: "Always expands the cell with the lowest cost so far. Optimal, but it searches in every direction equally.",
  },
  greedy: {
    name: "Greedy", kind: "grid",
    blurb: "Always expands the cell that looks closest to the goal. Fast on open maps, easily trapped, and not optimal.",
  },
  astar: {
    name: "A*", kind: "grid",
    blurb: "Ranks cells by cost so far plus estimated cost to go. Optimal when the estimate never overshoots the true cost.",
  },
  wastar: {
    name: "Weighted A*", kind: "grid",
    blurb: "A* with the estimate scaled by ε. It expands fewer cells, and the path costs at most ε times the optimum.",
  },
  rrt: {
    name: "RRT", kind: "tree",
    blurb: "Grows a tree toward random points until a branch reaches the goal. Finds a path quickly, but not a short one.",
  },
  rrtstar: {
    name: "RRT*", kind: "tree",
    blurb: "RRT that gives each new node the cheapest nearby parent and rewires neighbors through it. The path keeps getting shorter.",
  },
};

export const HEURISTICS = {
  manhattan: { name: "Manhattan", f: (dx, dy) => dx + dy },
  octile: { name: "Octile", f: (dx, dy) => Math.max(dx, dy) + (SQRT2 - 1) * Math.min(dx, dy) },
  euclidean: { name: "Euclidean", f: (dx, dy) => Math.hypot(dx, dy) },
};

/**
 * Admissible: never overestimates the true remaining cost. Consistent:
 * h(n) <= c(n, m) + h(m) for every move, which lets A* skip reopening.
 * Walls only make true costs larger, so empty-grid reasoning is enough.
 */
export function heuristicProps(heuristic, conn) {
  if (conn === 8 && heuristic === "manhattan") {
    return { admissible: false, consistent: false, exact: false, factor: SQRT2 };
  }
  const exact = (conn === 8 && heuristic === "octile") || (conn === 4 && heuristic === "manhattan");
  return { admissible: true, consistent: true, exact, factor: 1 };
}

export const defaultHeuristic = (conn) => (conn === 8 ? "octile" : "manhattan");

// ---- Random numbers ----

/** Small seeded PRNG so presets and RRT runs repeat exactly. */
export function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// ---- Grid ----

export function makeGrid(w = GRID_W, h = GRID_H) {
  return { w, h, cells: new Uint8Array(w * h) };
}

export const inBounds = (grid, x, y) => x >= 0 && y >= 0 && x < grid.w && y < grid.h;
export const blocked = (grid, x, y) => !inBounds(grid, x, y) || grid.cells[y * grid.w + x] === 1;

function fillRect(grid, x0, y0, x1, y1, v = 1) {
  for (let y = Math.max(0, y0); y <= Math.min(grid.h - 1, y1); y++) {
    for (let x = Math.max(0, x0); x <= Math.min(grid.w - 1, x1); x++) grid.cells[y * grid.w + x] = v;
  }
}

export function freeCount(grid) {
  let n = 0;
  for (const c of grid.cells) if (c === 0) n++;
  return n;
}

// ---- Map presets ----

export const PRESETS = {
  empty: "Empty",
  clutter: "Random clutter",
  maze: "Maze",
  rooms: "Rooms",
  bugtrap: "Bug trap",
  warehouse: "Warehouse",
  narrow: "Narrow passage",
  rings: "Rings",
  forest: "Forest",
  office: "Office",
  cave: "Cave",
};

/** Maps that draw a new layout for each seed. The rest are fixed. */
export const SEEDED_PRESETS = new Set(["clutter", "maze", "rooms", "warehouse", "narrow", "forest", "office", "cave"]);

/** Returns { grid, start, goal } with start and goal as integer cells. */
export function makePreset(name, seed = 1, w = GRID_W, h = GRID_H) {
  const make = { empty, clutter, maze, rooms, bugtrap, warehouse, narrow, rings, forest, office, cave }[name] ?? empty;
  return make(w, h, seed);
}

function empty(w, h) {
  const my = Math.floor(h / 2);
  return { grid: makeGrid(w, h), start: { x: 6, y: my }, goal: { x: w - 7, y: my } };
}

function clutter(w, h, seed) {
  const start = { x: 3, y: h - 4 }, goal = { x: w - 4, y: 3 };
  // Reroll until the two corners connect, even with 4-connected moves.
  for (let tries = 0; tries < 40; tries++) {
    const rand = mulberry32(seed * 7919 + tries);
    const grid = makeGrid(w, h);
    const count = Math.round((w * h) / 34);
    for (let i = 0; i < count; i++) {
      const bw = 1 + Math.floor(rand() * 4), bh = 1 + Math.floor(rand() * 4);
      const x = Math.floor(rand() * w), y = Math.floor(rand() * h);
      fillRect(grid, x, y, x + bw - 1, y + bh - 1);
    }
    fillRect(grid, start.x - 2, start.y - 2, start.x + 2, start.y + 2, 0);
    fillRect(grid, goal.x - 2, goal.y - 2, goal.x + 2, goal.y + 2, 0);
    if (run(gridSearch(grid, start, goal, { algo: "bfs", conn: 4 })).found) return { grid, start, goal };
  }
  return empty(w, h);
}

/** Recursive backtracker on odd cells, then a few walls knocked out so loops exist. */
function maze(w, h, seed) {
  const rand = mulberry32(seed * 104729 + 1);
  const grid = makeGrid(w, h);
  grid.cells.fill(1);
  const cw = Math.floor((w - 1) / 2), ch = Math.floor((h - 1) / 2);
  const seen = new Uint8Array(cw * ch);
  const stack = [0];
  seen[0] = 1;
  grid.cells[1 * w + 1] = 0;
  const dirs = [[1, 0], [-1, 0], [0, 1], [0, -1]];
  while (stack.length) {
    const c = stack[stack.length - 1];
    const cx = c % cw, cy = Math.floor(c / cw);
    const options = [];
    for (const [dx, dy] of dirs) {
      const nx = cx + dx, ny = cy + dy;
      if (nx >= 0 && ny >= 0 && nx < cw && ny < ch && !seen[ny * cw + nx]) options.push([nx, ny, dx, dy]);
    }
    if (!options.length) {
      stack.pop();
      continue;
    }
    const [nx, ny, dx, dy] = options[Math.floor(rand() * options.length)];
    seen[ny * cw + nx] = 1;
    grid.cells[(2 * cy + 1 + dy) * w + (2 * cx + 1 + dx)] = 0;
    grid.cells[(2 * ny + 1) * w + (2 * nx + 1)] = 0;
    stack.push(ny * cw + nx);
  }
  // A perfect maze has one route, so every planner would tie. Loops make
  // the optimal planners and the greedy ones disagree.
  for (let y = 1; y < 2 * ch; y++) {
    for (let x = 1; x < 2 * cw; x++) {
      const between = (x % 2 === 0) !== (y % 2 === 0);
      if (between && grid.cells[y * w + x] === 1 && rand() < 0.07) grid.cells[y * w + x] = 0;
    }
  }
  return { grid, start: { x: 1, y: 1 }, goal: { x: 2 * cw - 1, y: 2 * ch - 1 } };
}

function rooms(w, h, seed) {
  const rand = mulberry32(seed * 15485863 + 3);
  const grid = makeGrid(w, h);
  const vx = [Math.round(w / 3), Math.round((2 * w) / 3)];
  const hy = Math.round(h / 2);
  for (const x of vx) fillRect(grid, x, 0, x, h - 1);
  fillRect(grid, 0, hy, w - 1, hy);
  const door = (lo, hi) => lo + 2 + Math.floor(rand() * Math.max(1, hi - lo - 6));
  // One three-cell doorway in every wall segment between two rooms.
  for (const x of vx) {
    for (const [lo, hi] of [[0, hy - 1], [hy + 1, h - 1]]) {
      const d = door(lo, hi);
      fillRect(grid, x, d, x, d + 2, 0);
    }
  }
  for (const [lo, hi] of [[0, vx[0] - 1], [vx[0] + 1, vx[1] - 1], [vx[1] + 1, w - 1]]) {
    const d = door(lo, hi);
    fillRect(grid, d, hy, d + 2, hy, 0);
  }
  return { grid, start: { x: Math.round(vx[0] / 2), y: Math.round(hy / 2) }, goal: { x: w - 1 - Math.round(vx[0] / 2), y: h - 1 - Math.round(hy / 2) } };
}

/** A cup that opens away from the goal, with the start inside it. */
function bugtrap(w, h) {
  const grid = makeGrid(w, h);
  const cy = Math.floor(h / 2);
  const x0 = Math.round(w * 0.28), x1 = Math.round(w * 0.62);
  const y0 = cy - 12, y1 = cy + 12;
  fillRect(grid, x1, y0, x1 + 1, y1); // closed side, facing the goal
  fillRect(grid, x0, y0, x1, y0 + 1);
  fillRect(grid, x0, y1 - 1, x1, y1);
  // Lips at the mouth make the way out harder to stumble on.
  fillRect(grid, x0, y0, x0 + 1, cy - 5);
  fillRect(grid, x0, cy + 5, x0 + 1, y1);
  return { grid, start: { x: Math.round((x0 + x1) / 2) + 4, y: cy }, goal: { x: w - 6, y: cy } };
}

/** Racks of shelving with aisles between them; some cross aisles are blocked by pallets. */
function warehouse(w, h, seed) {
  const rand = mulberry32(seed * 7727 + 11);
  const grid = makeGrid(w, h);
  const cy = Math.floor(h / 2);
  const racks = [];
  for (let x = 5; x <= w - 7; x += 5) racks.push(x);
  for (const x of racks) {
    fillRect(grid, x, 4, x + 1, h - 5);
    // The middle cross aisle is open at most racks, but not all, so the
    // shortest route sometimes has to go around an end.
    if (rand() > 0.35) fillRect(grid, x, cy - 1, x + 1, cy + 1, 0);
  }
  // A few pallets left in the aisles, never sealing one off.
  for (let i = 0; i < 10; i++) {
    const r = racks[Math.floor(rand() * (racks.length - 1))];
    const y = 6 + Math.floor(rand() * (h - 12));
    fillRect(grid, r + 2, y, r + 3, y + 1);
  }
  const aisle = racks[Math.floor(racks.length / 2)] + 3;
  const goal = { x: aisle, y: 7 };
  fillRect(grid, goal.x - 1, goal.y - 1, goal.x + 1, goal.y + 1, 0); // no pallet on the goal
  return { grid, start: { x: 2, y: h - 3 }, goal };
}

/** One long wall with a single two-cell gap: easy for grid search, hard for RRT. */
function narrow(w, h, seed) {
  const rand = mulberry32(seed * 3301 + 5);
  const grid = makeGrid(w, h);
  const x = Math.floor(w / 2);
  fillRect(grid, x - 1, 0, x, h - 1);
  const gy = 4 + Math.floor(rand() * (h - 10));
  fillRect(grid, x - 1, gy, x, gy + 1, 0);
  const my = Math.floor(h / 2);
  return { grid, start: { x: 6, y: my }, goal: { x: w - 7, y: my } };
}

/** Nested walls with openings on alternating sides, so the path winds out from the center. */
function rings(w, h) {
  const grid = makeGrid(w, h);
  for (let k = 1; ; k++) {
    const x0 = k * 4 - 1, y0 = k * 4 - 1, x1 = w - k * 4, y1 = h - k * 4;
    if (x1 - x0 < 8 || y1 - y0 < 6) break;
    fillRect(grid, x0, y0, x1, y0);
    fillRect(grid, x0, y1, x1, y1);
    fillRect(grid, x0, y0, x0, y1);
    fillRect(grid, x1, y0, x1, y1);
    const my = Math.floor((y0 + y1) / 2);
    if (k % 2) fillRect(grid, x1, my - 1, x1, my + 1, 0);
    else fillRect(grid, x0, my - 1, x0, my + 1, 0);
  }
  return { grid, start: { x: Math.floor(w / 2), y: Math.floor(h / 2) }, goal: { x: 1, y: 1 } };
}

/** Round trees scattered at random, rerolled until the two edges connect. */
function forest(w, h, seed) {
  const my = Math.floor(h / 2);
  const start = { x: 2, y: my }, goal = { x: w - 3, y: my };
  for (let tries = 0; tries < 40; tries++) {
    const rand = mulberry32(seed * 6271 + tries);
    const grid = makeGrid(w, h);
    for (let i = 0; i < 70; i++) {
      const cx = 4 + rand() * (w - 8), cy = rand() * h, r = 0.8 + rand() * 1.8;
      for (let y = Math.floor(cy - r); y <= cy + r; y++) {
        for (let x = Math.floor(cx - r); x <= cx + r; x++) {
          if ((x - cx) ** 2 + (y - cy) ** 2 <= r * r) fillRect(grid, x, y, x, y);
        }
      }
    }
    fillRect(grid, 0, my - 2, 3, my + 2, 0);
    fillRect(grid, w - 4, my - 2, w - 1, my + 2, 0);
    if (run(gridSearch(grid, start, goal, { algo: "bfs", conn: 4 })).found) return { grid, start, goal };
  }
  return empty(w, h);
}

/** A corridor through the middle with offices on both sides, one door each. */
function office(w, h, seed) {
  const rand = mulberry32(seed * 911 + 17);
  const grid = makeGrid(w, h);
  const cy = Math.floor(h / 2);
  const top = cy - 2, bottom = cy + 2; // corridor walls
  fillRect(grid, 0, top, w - 1, top);
  fillRect(grid, 0, bottom, w - 1, bottom);
  const walls = [];
  for (let x = 0; x < w; ) {
    walls.push(x);
    x += 7 + Math.floor(rand() * 4);
  }
  for (const x of walls) {
    fillRect(grid, x, 0, x, top);
    fillRect(grid, x, bottom, x, h - 1);
  }
  walls.push(w);
  // A two-cell door from each office to the corridor.
  for (let i = 0; i < walls.length - 1; i++) {
    const a = walls[i] + 1, b = walls[i + 1] - 1;
    if (b - a < 2) continue;
    for (const wy of [top, bottom]) {
      const d = a + Math.floor(rand() * (b - a - 1));
      fillRect(grid, d, wy, d + 1, wy, 0);
    }
  }
  const sx = Math.floor((walls[0] + walls[1]) / 2), gx = Math.floor((walls[walls.length - 3] + walls[walls.length - 2]) / 2);
  return { grid, start: { x: Math.max(1, sx), y: 3 }, goal: { x: Math.min(w - 2, gx), y: h - 4 } };
}

/** Cellular-automaton caverns. Only the largest open region is kept, so every open cell is reachable. */
function cave(w, h, seed) {
  for (let tries = 0; tries < 20; tries++) {
    const rand = mulberry32(seed * 4093 + tries);
    let cells = new Uint8Array(w * h).map(() => (rand() < 0.5 ? 1 : 0));
    const wallAt = (c, x, y) => (x < 0 || y < 0 || x >= w || y >= h ? 1 : c[y * w + x]);
    // Five or more wall neighbors makes a wall, three or fewer opens it, four
    // keeps it as is. The "keep" band is what leaves winding passages rather
    // than one big open hall.
    for (let it = 0; it < 4; it++) {
      const next = new Uint8Array(w * h);
      for (let y = 0; y < h; y++) {
        for (let x = 0; x < w; x++) {
          let n = 0;
          for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) if (dx || dy) n += wallAt(cells, x + dx, y + dy);
          next[y * w + x] = n >= 5 ? 1 : n <= 3 ? 0 : cells[y * w + x];
        }
      }
      cells = next;
    }
    // Flood fill each open region and keep the biggest.
    const region = new Int32Array(w * h).fill(-1);
    let best = -1, bestSize = 0;
    for (let i = 0, id = 0; i < w * h; i++) {
      if (cells[i] || region[i] >= 0) continue;
      const stack = [i];
      region[i] = id;
      let size = 0;
      while (stack.length) {
        const c = stack.pop();
        size++;
        const x = c % w, y = Math.floor(c / w);
        for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
          const nx = x + dx, ny = y + dy, ni = ny * w + nx;
          if (nx >= 0 && ny >= 0 && nx < w && ny < h && !cells[ni] && region[ni] < 0) {
            region[ni] = id;
            stack.push(ni);
          }
        }
      }
      if (size > bestSize) [best, bestSize] = [id, size];
      id++;
    }
    if (bestSize < w * h * 0.25) continue;
    const grid = makeGrid(w, h);
    for (let i = 0; i < w * h; i++) grid.cells[i] = region[i] === best ? 0 : 1;
    // Start and goal: the open cells closest to the left and right middles.
    const near = (tx, ty) => {
      let pick = null, d = Infinity;
      for (let i = 0; i < w * h; i++) {
        if (grid.cells[i]) continue;
        const x = i % w, y = Math.floor(i / w), dd = (x - tx) ** 2 + (y - ty) ** 2;
        if (dd < d) [pick, d] = [{ x, y }, dd];
      }
      return pick;
    };
    return { grid, start: near(2, h / 2), goal: near(w - 3, h / 2) };
  }
  return empty(w, h);
}

// ---- Binary heap ----

/** Min-heap on (key, tie). Stale entries are skipped by the caller. */
export class MinHeap {
  constructor() {
    this.ids = [];
    this.keys = [];
    this.ties = [];
  }
  get size() {
    return this.ids.length;
  }
  less(a, b) {
    const ka = this.keys[a], kb = this.keys[b];
    // Keys are sums of 1 and √2, so equal costs can differ in the last bits.
    if (Math.abs(ka - kb) > 1e-9) return ka < kb;
    return this.ties[a] < this.ties[b];
  }
  swap(a, b) {
    for (const arr of [this.ids, this.keys, this.ties]) {
      const t = arr[a];
      arr[a] = arr[b];
      arr[b] = t;
    }
  }
  push(id, key, tie) {
    this.ids.push(id);
    this.keys.push(key);
    this.ties.push(tie);
    let i = this.ids.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!this.less(i, p)) break;
      this.swap(i, p);
      i = p;
    }
  }
  /** Removes the top; returns { id, key }. */
  pop() {
    const top = { id: this.ids[0], key: this.keys[0] };
    const last = this.ids.length - 1;
    this.swap(0, last);
    this.ids.pop();
    this.keys.pop();
    this.ties.pop();
    let i = 0;
    const n = this.ids.length;
    for (;;) {
      const l = 2 * i + 1, r = l + 1;
      let m = i;
      if (l < n && this.less(l, m)) m = l;
      if (r < n && this.less(r, m)) m = r;
      if (m === i) break;
      this.swap(i, m);
      i = m;
    }
    return top;
  }
}

// ---- Grid search ----

const MOVES4 = [[1, 0], [-1, 0], [0, 1], [0, -1]];
const MOVES8 = [...MOVES4, [1, 1], [1, -1], [-1, 1], [-1, -1]];

/**
 * Grid search as a generator: yields the same state object after every node
 * expansion, and once more with done = true. Options:
 *   algo: bfs | dijkstra | greedy | astar | wastar
 *   conn: 4 | 8, heuristic: manhattan | octile | euclidean, weight: ε for wastar
 * mark[i] is 0 unseen, 1 open, 2 closed.
 */
export function* gridSearch(grid, start, goal, opts = {}) {
  const { algo = "astar", conn = 8, heuristic = defaultHeuristic(conn), weight = 1 } = opts;
  const { w, h, cells } = grid;
  const N = w * h;
  const g = new Float64Array(N).fill(Infinity);
  const prio = new Float64Array(N).fill(Infinity);
  const parent = new Int32Array(N).fill(-1);
  const mark = new Uint8Array(N);
  const hfun = HEURISTICS[heuristic].f;
  const hOf = (i) => hfun(Math.abs((i % w) - goal.x), Math.abs(Math.floor(i / w) - goal.y));
  const eps = algo === "wastar" ? weight : algo === "astar" ? 1 : algo === "greedy" ? 1 : 0;
  const key = (gi, hi) => (algo === "greedy" ? hi : gi + eps * hi);
  const moves = conn === 8 ? MOVES8 : MOVES4;

  const st = {
    kind: "grid", w, h, mark, g, parent,
    current: -1, gCur: NaN, hCur: NaN, fCur: NaN, eps,
    expanded: 0, frontier: 0, done: false, found: false, path: null, cost: Infinity,
  };
  const s = start.y * w + start.x, t = goal.y * w + goal.x;
  if (blocked(grid, start.x, start.y) || blocked(grid, goal.x, goal.y)) {
    st.done = true;
    yield st;
    return st;
  }

  // BFS gets a plain FIFO; everything else shares the heap.
  const queue = algo === "bfs" ? new Int32Array(N) : null;
  let qh = 0, qt = 0;
  const heap = algo === "bfs" ? null : new MinHeap();
  let order = 0;
  // A*-family ties go to the node nearer the goal; others keep insertion order.
  const tie = (i) => (algo === "astar" || algo === "wastar" ? hOf(i) * 1e7 + order++ : order++);

  g[s] = 0;
  mark[s] = 1;
  st.frontier = 1;
  if (queue) queue[qt++] = s;
  else {
    prio[s] = key(0, hOf(s));
    heap.push(s, prio[s], tie(s));
  }

  for (;;) {
    let i = -1;
    if (queue) {
      if (qh < qt) i = queue[qh++];
    } else {
      while (heap.size) {
        const top = heap.pop();
        if (mark[top.id] === 1 && top.key === prio[top.id]) {
          i = top.id;
          break;
        }
      }
    }
    if (i < 0) break;

    mark[i] = 2;
    st.frontier--;
    st.expanded++;
    st.current = i;
    st.gCur = g[i];
    st.hCur = hOf(i);
    st.fCur = algo === "bfs" ? NaN : key(g[i], st.hCur);

    if (i === t) {
      st.found = true;
      st.cost = g[i];
      st.path = tracePath(parent, i, w);
      st.done = true;
      yield st;
      return st;
    }

    const x = i % w, y = (i - x) / w;
    for (const [dx, dy] of moves) {
      const nx = x + dx, ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const j = ny * w + nx;
      if (cells[j]) continue;
      // A diagonal needs both side cells free: no squeezing past a corner.
      if (dx && dy && (cells[y * w + nx] || cells[ny * w + x])) continue;
      const ng = g[i] + (dx && dy ? SQRT2 : 1);
      if (algo === "bfs" || algo === "greedy") {
        if (mark[j] !== 0) continue;
        g[j] = ng;
        parent[j] = i;
        mark[j] = 1;
        st.frontier++;
        if (queue) queue[qt++] = j;
        else {
          prio[j] = key(ng, hOf(j));
          heap.push(j, prio[j], tie(j));
        }
      } else if (ng < g[j] - 1e-12) {
        // Reopening closed nodes keeps the bounds true even when h is not
        // consistent; with a consistent h it never happens.
        g[j] = ng;
        parent[j] = i;
        if (mark[j] !== 1) st.frontier++;
        mark[j] = 1;
        prio[j] = key(ng, hOf(j));
        heap.push(j, prio[j], tie(j));
      }
    }
    yield st;
  }
  st.done = true;
  st.current = -1;
  yield st;
  return st;
}

function tracePath(parent, i, w) {
  const out = [];
  for (let n = i; n >= 0; n = parent[n]) out.push({ x: (n % w) + 0.5, y: Math.floor(n / w) + 0.5 });
  return out.reverse();
}

// ---- Collision checks in continuous space ----

export function pointFree(grid, x, y) {
  if (!(x >= 0 && y >= 0 && x < grid.w && y < grid.h)) return false;
  return grid.cells[Math.floor(y) * grid.w + Math.floor(x)] === 0;
}

/**
 * Walks every cell the segment passes through (Amanatides and Woo), so no
 * corner can be clipped between samples. Passing exactly through a grid
 * corner needs both side cells free, the same rule the grid searches use.
 */
export function segmentFree(grid, ax, ay, bx, by) {
  if (!pointFree(grid, ax, ay) || !pointFree(grid, bx, by)) return false;
  let x = Math.floor(ax), y = Math.floor(ay);
  const ex = Math.floor(bx), ey = Math.floor(by);
  const dx = bx - ax, dy = by - ay;
  const sx = Math.sign(dx), sy = Math.sign(dy);
  const tdx = dx ? Math.abs(1 / dx) : Infinity, tdy = dy ? Math.abs(1 / dy) : Infinity;
  let tx = dx > 0 ? (x + 1 - ax) / dx : dx < 0 ? (ax - x) / -dx : Infinity;
  let ty = dy > 0 ? (y + 1 - ay) / dy : dy < 0 ? (ay - y) / -dy : Infinity;
  const isBlocked = (cx, cy) => blocked(grid, cx, cy);
  for (let guard = Math.abs(ex - x) + Math.abs(ey - y) + 2; (x !== ex || y !== ey) && guard > 0; guard--) {
    if (Math.abs(tx - ty) < 1e-12) {
      if (isBlocked(x + sx, y) || isBlocked(x, y + sy)) return false;
      x += sx;
      y += sy;
      tx += tdx;
      ty += tdy;
    } else if (tx < ty) {
      x += sx;
      tx += tdx;
    } else {
      y += sy;
      ty += tdy;
    }
    if (isBlocked(x, y)) return false;
  }
  return true;
}

/** Dense point samples: a slow, independent check used by the tests. */
export function segmentFreeSampled(grid, ax, ay, bx, by, res = 0.005) {
  const n = Math.max(1, Math.ceil(Math.hypot(bx - ax, by - ay) / res));
  for (let k = 0; k <= n; k++) {
    const t = k / n;
    if (!pointFree(grid, ax + (bx - ax) * t, ay + (by - ay) * t)) return false;
  }
  return true;
}

export function pathFree(grid, path, check = segmentFree) {
  for (let k = 1; k < path.length; k++) {
    if (!check(grid, path[k - 1].x, path[k - 1].y, path[k].x, path[k].y)) return false;
  }
  return true;
}

export function pathLength(path) {
  let L = 0;
  for (let k = 1; k < path.length; k++) L += Math.hypot(path[k].x - path[k - 1].x, path[k].y - path[k - 1].y);
  return L;
}

// ---- RRT and RRT* ----

/**
 * The γ from Karaman and Frazzoli for d = 2: anything above
 * 2 (1 + 1/d)^(1/d) (free area / unit disk area)^(1/d) keeps RRT* optimal.
 */
export const rrtGamma = (freeArea) => 2 * Math.sqrt(1.5) * Math.sqrt(freeArea / Math.PI);

export function rrtRadius(gamma, n, step) {
  if (n < 2) return 0;
  return Math.min(gamma * Math.sqrt(Math.log(n) / n), step);
}

/**
 * RRT or RRT* as a generator: yields the same state object after every
 * iteration, and once more with done = true. RRT stops at the first path;
 * RRT* spends the whole budget improving it. Options:
 *   star, step (cells), goalBias (0 to 1), budget (iterations), seed
 */
export function* treeSearch(grid, start, goal, opts = {}) {
  const { star = false, step = 3, goalBias = 0.05, budget = 3000, seed = 1 } = opts;
  const rand = mulberry32(seed);
  const cap = budget + 1;
  const xs = new Float64Array(cap), ys = new Float64Array(cap);
  const parent = new Int32Array(cap).fill(-1);
  const cost = new Float64Array(cap);
  const children = star ? [[]] : null;
  const gx = goal.x + 0.5, gy = goal.y + 0.5;
  xs[0] = start.x + 0.5;
  ys[0] = start.y + 0.5;
  const gamma = rrtGamma(freeCount(grid));
  const goalLinks = [];

  const st = {
    kind: "tree", xs, ys, parent, n: 1, iter: 0, budget,
    sample: null, newNode: -1, radius: 0, gamma, near: 0, rewired: 0,
    done: false, found: false, firstIter: null, path: null, cost: Infinity, bestNode: -1,
  };
  if (!pointFree(grid, xs[0], ys[0]) || !pointFree(grid, gx, gy)) {
    st.done = true;
    yield st;
    return st;
  }

  const near = [];
  for (let it = 1; it <= budget; it++) {
    st.iter = it;
    st.newNode = -1;
    const goalPick = rand() < goalBias;
    const sx = goalPick ? gx : rand() * grid.w;
    const sy = goalPick ? gy : rand() * grid.h;
    st.sample = { x: sx, y: sy };

    let best = 0, bestD = Infinity;
    for (let k = 0; k < st.n; k++) {
      const d = (xs[k] - sx) ** 2 + (ys[k] - sy) ** 2;
      if (d < bestD) {
        bestD = d;
        best = k;
      }
    }
    bestD = Math.sqrt(bestD);
    if (bestD < 1e-9) {
      yield st;
      continue;
    }
    const f = Math.min(1, step / bestD);
    const nx = xs[best] + (sx - xs[best]) * f, ny = ys[best] + (sy - ys[best]) * f;
    if (!segmentFree(grid, xs[best], ys[best], nx, ny)) {
      yield st;
      continue;
    }

    let p = best;
    let c = cost[best] + Math.hypot(nx - xs[best], ny - ys[best]);
    near.length = 0;
    if (star) {
      const r = rrtRadius(gamma, st.n, step);
      st.radius = r;
      for (let k = 0; k < st.n; k++) {
        if ((xs[k] - nx) ** 2 + (ys[k] - ny) ** 2 <= r * r) near.push(k);
      }
      st.near = near.length;
      for (const k of near) {
        const ck = cost[k] + Math.hypot(nx - xs[k], ny - ys[k]);
        if (ck < c - 1e-12 && segmentFree(grid, xs[k], ys[k], nx, ny)) {
          p = k;
          c = ck;
        }
      }
    }

    const n = st.n++;
    xs[n] = nx;
    ys[n] = ny;
    parent[n] = p;
    cost[n] = c;
    st.newNode = n;

    if (star) {
      children.push([]);
      children[p].push(n);
      st.rewired = 0;
      for (const k of near) {
        if (k === p) continue;
        const ck = c + Math.hypot(nx - xs[k], ny - ys[k]);
        if (ck < cost[k] - 1e-12 && segmentFree(grid, nx, ny, xs[k], ys[k])) {
          const old = children[parent[k]];
          old.splice(old.indexOf(k), 1);
          parent[k] = n;
          children[n].push(k);
          // The whole subtree under k gets cheaper by the same amount.
          const delta = ck - cost[k];
          const stack = [k];
          while (stack.length) {
            const m = stack.pop();
            cost[m] += delta;
            for (const ch of children[m]) stack.push(ch);
          }
          st.rewired++;
        }
      }
    }

    if (Math.hypot(gx - nx, gy - ny) <= step && segmentFree(grid, nx, ny, gx, gy)) goalLinks.push(n);
    let bestCost = st.cost, bestNode = st.bestNode;
    for (const k of goalLinks) {
      const ck = cost[k] + Math.hypot(gx - xs[k], gy - ys[k]);
      if (ck < bestCost - 1e-12) {
        bestCost = ck;
        bestNode = k;
      }
    }
    if (bestNode >= 0 && (bestNode !== st.bestNode || bestCost < st.cost)) {
      if (!st.found) st.firstIter = it;
      st.found = true;
      st.cost = bestCost;
      st.bestNode = bestNode;
    }
    if (st.found) st.path = treePath(st, gx, gy);
    if (st.found && !star) break;
    yield st;
  }
  st.done = true;
  yield st;
  return st;
}

function treePath(st, gx, gy) {
  const out = [{ x: gx, y: gy }];
  for (let k = st.bestNode; k >= 0; k = st.parent[k]) out.push({ x: st.xs[k], y: st.ys[k] });
  return out.reverse();
}

// ---- Helpers ----

/** Drains a planner generator and returns its final state. */
export function run(gen) {
  let last = null;
  for (const s of gen) last = s;
  return last;
}

export function planner(algo, grid, start, goal, opts = {}) {
  if (ALGOS[algo].kind === "tree") return treeSearch(grid, start, goal, { ...opts, star: algo === "rrtstar" });
  return gridSearch(grid, start, goal, { ...opts, algo });
}

const now = () => (globalThis.performance ? performance.now() : Date.now());

/**
 * Runs every algorithm to completion on one map. Grid rows report expanded
 * nodes; tree rows report iterations used.
 */
export function compareAll(grid, start, goal, opts = {}) {
  const rows = [];
  for (const algo of Object.keys(ALGOS)) {
    const t0 = now();
    const s = run(planner(algo, grid, start, goal, opts));
    const ms = now() - t0;
    const grid_ = ALGOS[algo].kind === "grid";
    rows.push({
      algo,
      work: grid_ ? s.expanded : s.found && algo === "rrt" ? s.firstIter : s.iter,
      found: s.found,
      length: s.found ? (grid_ ? s.cost : pathLength(s.path)) : null,
      ms,
    });
  }
  const opt = rows.find((r) => r.algo === "dijkstra").length;
  for (const r of rows) {
    r.ratio = r.length !== null && opt ? r.length / opt : null;
    r.optimal = ALGOS[r.algo].kind === "grid" && r.length !== null && opt !== null ? Math.abs(r.length - opt) < 1e-6 : null;
  }
  return rows;
}
