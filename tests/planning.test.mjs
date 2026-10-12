import { test } from "node:test";
import assert from "node:assert/strict";
import {
  ALGOS, MAP_FORMAT, MinHeap, PRESETS, blocked, compareAll, formatMap, gridSearch, makeGrid, makePreset,
  parseMap, pathFree, pathLength, run, segmentFree, segmentFreeSampled, treeSearch,
} from "../js/planning-sim.js";

const close = (a, b, tol = 1e-6) => Math.abs(a - b) <= tol;
const MAPS = Object.keys(PRESETS).flatMap((name) => [1, 2, 3].map((seed) => ({ name, seed, ...makePreset(name, seed) })));
// Dense point sampling, independent of the planner's own cell walk.
const FINE = segmentFreeSampled;

const table = [];
for (const m of MAPS.filter((m) => m.seed === 1)) {
  for (const r of compareAll(m.grid, m.start, m.goal, { conn: 8, heuristic: "octile", weight: 2, budget: 3000, seed: 1 })) {
    table.push({ map: m.name, algo: r.algo, work: r.work, length: r.length?.toFixed(2) ?? "none", ms: r.ms.toFixed(1) });
  }
}
console.table(table);

test("heap pops in key order", () => {
  const h = new MinHeap();
  const keys = [5, 3, 9, 1, 7, 3, 8, 2];
  keys.forEach((k, i) => h.push(i, k, i));
  const out = [];
  while (h.size) out.push(h.pop().key);
  assert.deepEqual(out, [...keys].sort((a, b) => a - b));
});

test("every preset has a path from start to goal", () => {
  for (const m of MAPS) {
    const s = run(gridSearch(m.grid, m.start, m.goal, { algo: "dijkstra", conn: 4 }));
    assert.ok(s.found, `${m.name} seed ${m.seed}`);
  }
});

test("Dijkstra and A* (octile, 8-connected) agree on cost; A* expands no more", () => {
  for (const m of MAPS) {
    const d = run(gridSearch(m.grid, m.start, m.goal, { algo: "dijkstra", conn: 8 }));
    const a = run(gridSearch(m.grid, m.start, m.goal, { algo: "astar", conn: 8, heuristic: "octile" }));
    assert.ok(d.found && a.found, `${m.name} seed ${m.seed}`);
    assert.ok(close(d.cost, a.cost), `${m.name} seed ${m.seed}: ${d.cost} vs ${a.cost}`);
    assert.ok(close(pathLength(a.path), a.cost), "path length matches g");
    assert.ok(a.expanded <= d.expanded, `${m.name} seed ${m.seed}: A* ${a.expanded} vs Dijkstra ${d.expanded}`);
  }
});

test("A* with an admissible heuristic is optimal in both connectivities", () => {
  for (const m of MAPS) {
    for (const [conn, heuristics] of [[4, ["manhattan", "octile", "euclidean"]], [8, ["octile", "euclidean"]]]) {
      const d = run(gridSearch(m.grid, m.start, m.goal, { algo: "dijkstra", conn }));
      for (const heuristic of heuristics) {
        const a = run(gridSearch(m.grid, m.start, m.goal, { algo: "astar", conn, heuristic }));
        assert.ok(close(d.cost, a.cost), `${m.name} ${conn} ${heuristic}`);
      }
    }
  }
});

test("BFS on a 4-connected grid matches Dijkstra", () => {
  for (const m of MAPS) {
    const d = run(gridSearch(m.grid, m.start, m.goal, { algo: "dijkstra", conn: 4 }));
    const b = run(gridSearch(m.grid, m.start, m.goal, { algo: "bfs", conn: 4 }));
    assert.ok(close(d.cost, b.cost), `${m.name} seed ${m.seed}: ${d.cost} vs ${b.cost}`);
  }
});

test("weighted A* cost is at most ε times optimal", () => {
  for (const m of MAPS) {
    const d = run(gridSearch(m.grid, m.start, m.goal, { algo: "dijkstra", conn: 8 }));
    for (const weight of [1, 1.5, 2, 3, 5]) {
      const w = run(gridSearch(m.grid, m.start, m.goal, { algo: "wastar", conn: 8, heuristic: "octile", weight }));
      assert.ok(w.found);
      assert.ok(w.cost <= weight * d.cost + 1e-9, `${m.name} ε=${weight}: ${w.cost} > ${weight} × ${d.cost}`);
    }
  }
});

test("greedy floods the bug trap but crosses an empty map directly", () => {
  const m = makePreset("bugtrap");
  const open = makePreset("empty");
  const trapped = run(gridSearch(m.grid, m.start, m.goal, { algo: "greedy", conn: 8 }));
  const direct = run(gridSearch(open.grid, m.start, m.goal, { algo: "greedy", conn: 8 }));
  assert.ok(trapped.found);
  assert.ok(trapped.expanded > 10 * direct.expanded, `trap ${trapped.expanded} vs open ${direct.expanded}`);
});

test("the cell walk agrees with dense sampling on random segments", () => {
  const m = makePreset("clutter", 2);
  let seed = 12345;
  const rand = () => ((seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648);
  for (let k = 0; k < 4000; k++) {
    const ax = rand() * m.grid.w, ay = rand() * m.grid.h;
    const bx = ax + (rand() - 0.5) * 8, by = ay + (rand() - 0.5) * 8;
    const exact = segmentFree(m.grid, ax, ay, bx, by);
    // The walk may be stricter (exact corners), never looser.
    if (exact) assert.ok(segmentFreeSampled(m.grid, ax, ay, bx, by), `${ax},${ay} to ${bx},${by}`);
  }
});

test("paths never cross walls or cut corners", () => {
  for (const m of MAPS) {
    for (const algo of ["bfs", "dijkstra", "greedy", "astar", "wastar"]) {
      for (const conn of [4, 8]) {
        const s = run(gridSearch(m.grid, m.start, m.goal, { algo, conn, weight: 3 }));
        assert.ok(s.found, `${m.name} ${algo} ${conn}`);
        assert.ok(pathFree(m.grid, s.path, FINE), `${m.name} ${algo} ${conn} crosses a wall`);
        for (let k = 1; k < s.path.length; k++) {
          const a = s.path[k - 1], b = s.path[k];
          const dx = Math.round(b.x - a.x), dy = Math.round(b.y - a.y);
          assert.ok(Math.abs(dx) <= 1 && Math.abs(dy) <= 1, "one cell per move");
          if (conn === 4) assert.ok(dx === 0 || dy === 0, "4-connected moves are orthogonal");
          if (dx && dy) {
            const x = Math.floor(a.x), y = Math.floor(a.y), w = m.grid.w;
            assert.equal(m.grid.cells[y * w + x + dx], 0, `${m.name} ${algo} cuts a corner`);
            assert.equal(m.grid.cells[(y + dy) * w + x], 0, `${m.name} ${algo} cuts a corner`);
          }
        }
      }
    }
  }
});

test("a diagonal between two blocked cells is not a path", () => {
  const grid = makeGrid(3, 3);
  grid.cells[0 * 3 + 1] = 1;
  grid.cells[1 * 3 + 0] = 1;
  for (const algo of Object.keys(ALGOS).filter((a) => ALGOS[a].kind === "grid")) {
    const s = run(gridSearch(grid, { x: 0, y: 0 }, { x: 1, y: 1 }, { algo, conn: 8 }));
    assert.equal(s.found, false, algo);
  }
});

test("a walled-in or blocked goal returns no path cleanly", () => {
  const { grid, start } = makePreset("empty");
  const goal = { x: 40, y: 20 };
  for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) if (dx || dy) grid.cells[(goal.y + dy) * grid.w + goal.x + dx] = 1;
  for (const algo of Object.keys(ALGOS)) {
    const gen = ALGOS[algo].kind === "grid"
      ? gridSearch(grid, start, goal, { algo, conn: 8 })
      : treeSearch(grid, start, goal, { star: algo === "rrtstar", budget: 400, seed: 3 });
    const s = run(gen);
    assert.equal(s.done, true, algo);
    assert.equal(s.found, false, algo);
    assert.equal(s.path, null, algo);
  }
  grid.cells[goal.y * grid.w + goal.x] = 1;
  const s = run(gridSearch(grid, start, goal, { algo: "astar" }));
  assert.equal(s.found, false);
  assert.equal(s.expanded, 0);
});

test("RRT finds a collision-free path on an open map", () => {
  const m = makePreset("empty");
  for (const seed of [1, 2, 3, 4, 5]) {
    const s = run(treeSearch(m.grid, m.start, m.goal, { step: 3, goalBias: 0.05, budget: 3000, seed }));
    assert.ok(s.found, `seed ${seed}`);
    assert.ok(s.iter < 3000);
    assert.ok(pathFree(m.grid, s.path, FINE), `seed ${seed}`);
    assert.ok(close(pathLength(s.path), s.cost, 1e-6));
  }
});

test("RRT and RRT* paths are collision free on cluttered maps", () => {
  for (const name of ["clutter", "rooms", "bugtrap"]) {
    const m = makePreset(name);
    for (const star of [false, true]) {
      const s = run(treeSearch(m.grid, m.start, m.goal, { star, step: 2.5, goalBias: 0.05, budget: 4000, seed: 7 }));
      assert.ok(s.found, `${name} star=${star}`);
      assert.ok(pathFree(m.grid, s.path, FINE), `${name} star=${star}`);
    }
  }
});

test("RRT* cost never increases as iterations go on", () => {
  for (const name of ["empty", "clutter"]) {
    const m = makePreset(name);
    const checkpoints = [500, 1000, 1500, 2000, 3000];
    const costs = [];
    let prev = Infinity;
    for (const s of treeSearch(m.grid, m.start, m.goal, { star: true, step: 3, goalBias: 0.05, budget: 3000, seed: 11 })) {
      assert.ok(s.cost <= prev + 1e-9, `${name}: cost rose at iteration ${s.iter}`);
      prev = s.cost;
      if (checkpoints.includes(s.iter) && !costs.find((c) => c.iter === s.iter)) {
        costs.push({ iter: s.iter, cost: s.cost });
        if (s.found) assert.ok(pathFree(m.grid, s.path, FINE));
        if (s.found) assert.ok(close(pathLength(s.path), s.cost, 1e-6), "tree costs stay in sync after rewiring");
      }
    }
    assert.equal(costs.length, checkpoints.length);
    assert.ok(Number.isFinite(costs[0].cost), `${name}: found by iteration 500`);
    for (let k = 1; k < costs.length; k++) assert.ok(costs[k].cost <= costs[k - 1].cost);
    assert.ok(costs.at(-1).cost < costs[0].cost, `${name}: RRT* improved`);
  }
});

test("runs repeat exactly with the same seed", () => {
  const m = makePreset("clutter", 4);
  const a = run(treeSearch(m.grid, m.start, m.goal, { star: true, budget: 800, seed: 9 }));
  const b = run(treeSearch(m.grid, m.start, m.goal, { star: true, budget: 800, seed: 9 }));
  assert.equal(a.cost, b.cost);
  assert.equal(a.n, b.n);
});

test("every map connects start and goal, 4- and 8-connected, several seeds", async () => {
  const { gridSearch, run: runGen, blocked } = await import("../js/planning-sim.js");
  for (const name of Object.keys(PRESETS)) {
    for (const seed of [1, 2, 3, 4, 5]) {
      const m = makePreset(name, seed);
      assert.ok(!blocked(m.grid, m.start.x, m.start.y) && !blocked(m.grid, m.goal.x, m.goal.y), `${name} ${seed} endpoints free`);
      for (const conn of [4, 8]) {
        assert.ok(runGen(gridSearch(m.grid, m.start, m.goal, { algo: "bfs", conn })).found, `${name} seed ${seed} conn ${conn}`);
      }
    }
  }
});

// ---- Maps as text ----

test("the documented example parses to what it looks like", () => {
  const { grid, start, goal, placed } = parseMap(MAP_FORMAT.example);
  assert.equal(grid.w, 10);
  assert.equal(grid.h, 5);
  assert.deepEqual(start, { x: 1, y: 1 });
  assert.deepEqual(goal, { x: 8, y: 3 });
  assert.deepEqual(placed, []);
  assert.equal(blocked(grid, 5, 1), true, "the wall is where the drawing puts it");
  assert.equal(blocked(grid, 4, 1), false);
});

test("a map survives the round trip through text", () => {
  for (const name of ["clutter", "maze", "rooms", "bugtrap", "cave"]) {
    const a = makePreset(name, 3);
    const text = formatMap(a.grid, a.start, a.goal);
    const b = parseMap(text);
    assert.equal(b.grid.w, a.grid.w);
    assert.equal(b.grid.h, a.grid.h);
    assert.deepEqual([...b.grid.cells], [...a.grid.cells], `${name}: the walls came back different`);
    assert.deepEqual(b.start, a.start);
    assert.deepEqual(b.goal, a.goal);
    assert.equal(formatMap(b.grid, b.start, b.goal), text, `${name}: not a fixed point`);
  }
});

test("ragged rows are padded and notes are ignored", () => {
  const { grid, start, goal } = parseMap("; a note\n#####\n#S\n#\n#...G\n#####\n; another");
  assert.equal(grid.w, 5);
  assert.equal(grid.h, 5);
  assert.deepEqual(start, { x: 1, y: 1 });
  assert.deepEqual(goal, { x: 4, y: 3 });
  assert.equal(blocked(grid, 4, 1), false, "the padding is free space");
});

test("a map with no S or G still opens, with both placed", () => {
  const { start, goal, placed } = parseMap("#####\n#...#\n#...#\n#...#\n#####");
  assert.deepEqual(placed, ["start", "goal"]);
  assert.deepEqual(start, { x: 1, y: 1 });
  assert.deepEqual(goal, { x: 3, y: 3 });
});

test("a planner actually runs on a parsed map", () => {
  const { grid, start, goal } = parseMap(["S....", ".###.", ".....", ".###.", "....G"].join("\n"));
  const res = run(gridSearch(grid, start, goal, { algo: "astar", conn: 8 }));
  assert.equal(res.found, true);
  assert.ok(res.path.length > 4);
  // Path points are cell centers.
  assert.deepEqual(res.path[0], { x: start.x + 0.5, y: start.y + 0.5 });
  assert.deepEqual(res.path[res.path.length - 1], { x: goal.x + 0.5, y: goal.y + 0.5 });
});

test("broken maps are refused with a message that says where", () => {
  const cases = [
    ["", /no map in it/],
    ["..\n..", /smallest allowed/],
    [".".repeat(201) + "\n" + (".".repeat(201) + "\n").repeat(4), /largest allowed/],
    ["..........\n..x.......\n..........\n..........\n..........", /Line 2, column 3.*"x"/],
    ["#####\n#####\n#####\n#####\n#####", /nowhere to plan/],
    ["S.S..\n.....\n.....\n.....\n....G", /Two starts/],
    ["S...G\n.....\n.....\n.....\n....G", /Two goals/],
  ];
  for (const [text, re] of cases) {
    assert.throws(() => parseMap(text), re, `should have been refused: ${JSON.stringify(text.slice(0, 20))}`);
  }
});
