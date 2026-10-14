// Motion planning playground UI: a paintable occupancy grid, grid searches
// and sampling planners animated one step at a time, a table that runs every
// planner on the current map, and the math behind each one, typeset with
// KaTeX through tex.js.

import {
  ALGOS, GRID_H, GRID_W, HEURISTICS, MAP_FORMAT, PRESETS, SEEDED_PRESETS,
  compareAll, defaultHeuristic, formatMap, freeCount, heuristicProps, makePreset, parseMap, planner,
  rrtGamma, rrtRadius,
} from "./planning-sim.js";
import { copyText, cssVar, dockWhenNarrow, el, evalNumber, fmt, segmented } from "./util.js";
import { inl, num, tex } from "./tex.js";
import { createArmPlanner } from "./planning-arm.js";

// Expansions (or iterations) per frame. Fractions skip frames.
const SPEEDS = [0.25, 0.5, 1, 2, 3, 5, 10, 20, 50, 100, 250, 1000, Infinity];
const FRAME_MS = 12; // stop stepping early so a frame never janks
const COMPARE_DELAY = 250;
const COMPARE_SEED = 1;

export function start(root) {
  const $ = (sel) => root.querySelector(sel);
  const wrap = $("#plan-stage-wrap");
  // On a phone, Run and the timeline sit right under the grid, not past the
  // stats and the comparison table.
  dockWhenNarrow($("#plan-run").closest(".side-section"), wrap.closest(".panel"));
  let map = makePreset("clutter", 1);
  const opt = { algo: "astar", conn: 8, heuristic: "octile", weight: 2, step: 3, goalBias: 0.05, budget: 3000, samples: 400, k: 10 };
  let speedIdx = 6;
  let editMode = "draw";
  let active = true;
  let raf = 0;
  let colors = readColors();

  // Animated run. Every planner here is a deterministic generator, so a run
  // is really a recording: step N is whatever the generator yields on its Nth
  // call. That makes scrubbing possible without storing 2500 copies of the
  // grid. Going forward advances the live generator; going back rebuilds it
  // from the start and fast-forwards, which costs well under a millisecond
  // for the grid searches and about 25 ms for a full RRT*.
  let gen = null;
  let cur = null;
  let curStep = 0;    // how many steps of the recording have been played
  let totalSteps = 0; // its full length, measured when the run is created
  let runOpts = null; // the options the recording was made with, including the seed
  let pendingSeek = null;
  let playing = false;
  let stepAcc = 0;
  let runSeed = 1;

  let compareRows = null;
  let compareTimer = 0;
  let compareStale = true;

  window.addEventListener("rb:theme", () => {
    colors = readColors();
    drawThumbs();
    requestRender();
  });

  function readColors() {
    const c = (n) => cssVar(n);
    return {
      panel: c("--panel"), line: c("--line"), lineStrong: c("--line-strong"), fg: c("--fg"),
      accent: c("--accent"), accentSoft: c("--accent-soft"), highlight: c("--highlight"), solid: c("--solid"),
      start: c("--plot-d"), goal: c("--plot-error"), raised: c("--raised"),
    };
  }

  // ---- Algorithm picker ----
  const algoBox = $("#plan-algo");
  for (const [k, a] of Object.entries(ALGOS)) {
    algoBox.append(el("button", { type: "button", "data-v": k, "aria-pressed": String(k === opt.algo), text: a.name }));
  }
  segmented(algoBox, (k) => {
    opt.algo = k;
    resetRun();
    syncAlgo();
    renderCompare();
  });
  function syncAlgo() {
    const a = ALGOS[opt.algo];
    $("#plan-blurb").textContent = a.blurb;
    $("#plan-grid-opts").hidden = a.kind !== "grid";
    $("#plan-tree-opts").hidden = a.kind !== "tree";
    $("#plan-prm-opts").hidden = a.kind !== "roadmap";
    const tree = a.kind === "tree", roadmap = a.kind === "roadmap";
    $("#plan-m-work-label").textContent = tree ? "Iterations" : roadmap ? "Roadmap nodes" : "Nodes expanded";
    $("#plan-m-frontier-label").textContent = tree ? "Tree nodes" : roadmap ? "Roadmap links" : "Frontier size";
    updateMath();
  }

  // ---- Sliders with typed values, as on the PID page ----
  function bindGain(id, key, { digits, min, max, onSet }) {
    const slider = $(`#${id}`), num = $(`#${id}-num`);
    const set = (v, from) => {
      v = Math.min(max, Math.max(min, v));
      opt[key] = v;
      if (from !== "slider") slider.value = v;
      if (from !== "num") num.value = fmt(v, digits);
      // The first call only fills the fields; nothing has run yet.
      if (from !== "init") onSet?.();
    };
    slider.addEventListener("input", () => set(Number(slider.value), "slider"));
    num.addEventListener("change", () => {
      const v = evalNumber(num.value);
      if (Number.isFinite(v)) set(v, "num");
      num.value = fmt(opt[key], digits);
    });
    set(opt[key], "init");
  }
  const optionChanged = () => {
    invalidateRun();
    scheduleCompare();
  };
  bindGain("plan-weight", "weight", { digits: 1, min: 1, max: 5, onSet: optionChanged });
  bindGain("plan-stepsize", "step", { digits: 1, min: 0.5, max: 8, onSet: optionChanged });
  bindGain("plan-bias", "goalBias", { digits: 2, min: 0, max: 0.5, onSet: optionChanged });
  bindGain("plan-budget", "budget", { digits: 0, min: 100, max: 5000, onSet: optionChanged });
  bindGain("plan-samples", "samples", { digits: 0, min: 50, max: 1500, onSet: optionChanged });
  bindGain("plan-k", "k", { digits: 0, min: 2, max: 30, onSet: optionChanged });

  const speed = $("#plan-speed");
  speed.max = SPEEDS.length - 1;
  speed.value = speedIdx;
  speed.addEventListener("input", () => {
    speedIdx = Number(speed.value);
    syncSpeed();
  });
  function syncSpeed() {
    const v = SPEEDS[speedIdx];
    const num = $("#plan-speed-num"), unit = $("#plan-speed-unit");
    if (v === Infinity) {
      num.textContent = "";
      unit.textContent = "instant";
    } else if (v < 1) {
      num.textContent = String(Math.round(1 / v));
      unit.textContent = "frames per step";
    } else {
      num.textContent = String(v);
      unit.textContent = v === 1 ? "step per frame" : "steps per frame";
    }
  }

  // ---- Grid options ----
  const conn = segmented($("#plan-conn"), (v) => {
    opt.conn = Number(v);
    // Each connectivity has its own natural heuristic; switching follows it.
    opt.heuristic = defaultHeuristic(opt.conn);
    heurSel.value = opt.heuristic;
    optionChanged();
  });
  conn.set(String(opt.conn));
  const heurSel = $("#plan-heuristic");
  for (const [k, h] of Object.entries(HEURISTICS)) heurSel.append(el("option", { value: k, text: h.name }));
  heurSel.value = opt.heuristic;
  heurSel.addEventListener("change", () => {
    opt.heuristic = heurSel.value;
    optionChanged();
  });

  // ---- Map presets and editing ----
  segmented($("#plan-mode"), (v) => (editMode = v));
  // Map picker: a thumbnail per map. Clicking a generated map again draws a
  // new layout, and its thumbnail follows.
  const presetBox = $("#plan-presets");
  const strip = $("#plan-maps");
  const seeds = Object.fromEntries(Object.keys(PRESETS).map((k) => [k, 1]));
  const cards = {};
  const UPLOAD_KEY = "__upload";
  let uploaded = null; // the parsed file, kept as written
  let currentPreset = "clutter"; // matches the map loaded above
  for (const [k, name] of Object.entries(PRESETS)) {
    const thumb = el("canvas", { width: GRID_W, height: GRID_H, "aria-hidden": "true" });
    const card = el("button", {
      type: "button", class: "map-card", "aria-pressed": "false",
      title: SEEDED_PRESETS.has(k) ? `${name}. Click again for a new layout.` : name,
      onclick: () => {
        if (k === currentPreset && SEEDED_PRESETS.has(k)) seeds[k]++;
        loadPreset(k);
      },
    }, thumb, el("span", { text: name }));
    cards[k] = { card, thumb };
    strip.append(card);
  }
  function drawThumb(k) {
    const src = k === UPLOAD_KEY ? uploaded : makePreset(k, seeds[k]);
    if (!src) return;
    const { grid, start, goal } = src;
    const thumb = cards[k].thumb;
    if (thumb.width !== grid.w || thumb.height !== grid.h) {
      thumb.width = grid.w;
      thumb.height = grid.h;
    }
    const ctx = thumb.getContext("2d");
    const img = ctx.createImageData(grid.w, grid.h);
    const rgb = (hex) => [1, 3, 5].map((i) => parseInt(hex.slice(i, i + 2), 16));
    const wall = rgb(colors.solid), free = rgb(colors.raised);
    for (let i = 0; i < grid.cells.length; i++) {
      const c = grid.cells[i] ? wall : free;
      img.data.set([c[0], c[1], c[2], 255], i * 4);
    }
    const dot = (p, hex) => {
      const c = rgb(hex);
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
        const x = p.x + dx, y = p.y + dy;
        if (x >= 0 && y >= 0 && x < grid.w && y < grid.h) img.data.set([c[0], c[1], c[2], 255], (y * grid.w + x) * 4);
      }
    };
    dot(start, colors.start);
    dot(goal, colors.goal);
    ctx.putImageData(img, 0, 0);
  }
  function drawThumbs() {
    for (const k of Object.keys(cards)) drawThumb(k);
  }
  const cloneMap = (m) => ({
    grid: { w: m.grid.w, h: m.grid.h, cells: new Uint8Array(m.grid.cells) },
    start: { ...m.start }, goal: { ...m.goal },
  });
  function loadPreset(k) {
    currentPreset = k;
    // The uploaded map is kept pristine, so picking its card reloads the file
    // as written rather than whatever has been drawn on top of it since.
    map = k === UPLOAD_KEY ? cloneMap(uploaded) : makePreset(k, seeds[k]);
    keepAsPicked();
    for (const [key, { card }] of Object.entries(cards)) card.setAttribute("aria-pressed", String(key === k));
    drawThumb(k);
    mapChanged(true);
  }
  // ---- Your own map ----
  // The card leads the strip, because a card at the far end of twelve is a
  // card nobody finds. Before a file is in it, it is the button that opens
  // the picker; after, it behaves like any other map.
  const fileInput = $("#plan-map-file");
  const mapStatus = $("#plan-map-status");
  const uploadCard = {};
  {
    const thumb = el("canvas", { width: GRID_W, height: GRID_H, "aria-hidden": "true", hidden: true });
    // A pixelated 61 by 41 canvas is no place for an icon, so the empty state
    // is its own element and the two swap.
    const empty = el("span", { class: "map-card-empty", "aria-hidden": "true" });
    empty.innerHTML = '<svg viewBox="0 0 16 16"><path d="M8 11V2.5M4.5 6 8 2.5 11.5 6M2.5 10.5v2a1 1 0 0 0 1 1h9a1 1 0 0 0 1-1v-2" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/></svg>';
    const label = el("span", { text: "Your map" });
    const card = el("button", {
      type: "button", class: "map-card map-card-upload", "aria-pressed": "false",
      title: "Load a map from a text file",
      onclick: () => (uploaded ? loadPreset(UPLOAD_KEY) : fileInput.click()),
    }, empty, thumb, label);
    cards[UPLOAD_KEY] = { card, thumb };
    Object.assign(uploadCard, { card, thumb, empty, label });
    strip.prepend(card);
  }
  /** The card shows the file once there is one, and the icon until then. */
  function syncUploadCard() {
    uploadCard.empty.hidden = !!uploaded;
    uploadCard.thumb.hidden = !uploaded;
    uploadCard.card.title = uploaded ? "Your map. Click to reload it as written." : "Load a map from a text file";
  }

  function setMapStatus(text, bad = false) {
    mapStatus.textContent = text;
    mapStatus.className = bad ? "small is-bad" : "small subtle";
  }

  /** Parse one file's text and make it the current map, or explain why not. */
  function loadMapText(text, name = "your file") {
    let parsed;
    try {
      parsed = parseMap(text);
    } catch (err) {
      setMapStatus(`${name} could not be read. ${err.message}`, true);
      return false;
    }
    uploaded = parsed;
    syncUploadCard();
    loadPreset(UPLOAD_KEY);
    const placed = parsed.placed.length ? ` No ${parsed.placed.join(" or ")} in the file, so it was placed for you.` : "";
    setMapStatus(`Loaded ${name}: ${parsed.grid.w} by ${parsed.grid.h} cells.${placed}`);
    return true;
  }

  function readFile(file) {
    if (!file) return;
    if (file.size > 2e6) {
      setMapStatus(`${file.name} is too big to be a map file.`, true);
      return;
    }
    file.text().then((t) => loadMapText(t, file.name)).catch(() => setMapStatus(`${file.name} could not be opened.`, true));
  }
  fileInput.addEventListener("change", () => {
    readFile(fileInput.files[0]);
    // Clearing it means picking the same file twice in a row still fires.
    fileInput.value = "";
  });

  // Dropping a file on the grid is the other obvious way to try this.
  for (const ev of ["dragenter", "dragover"]) {
    wrap.addEventListener(ev, (e) => {
      e.preventDefault();
      wrap.classList.add("drop-target");
    });
  }
  for (const ev of ["dragleave", "drop"]) {
    wrap.addEventListener(ev, () => wrap.classList.remove("drop-target"));
  }
  wrap.addEventListener("drop", (e) => {
    e.preventDefault();
    readFile(e.dataTransfer?.files?.[0]);
  });

  $("#plan-map-pick").addEventListener("click", () => fileInput.click());
  $("#plan-map-example").textContent = MAP_FORMAT.example;
  $("#plan-map-copy").addEventListener("click", (e) => copyText(MAP_FORMAT.example, e.currentTarget));
  $("#plan-map-download").addEventListener("click", () => {
    const text = formatMap(map.grid, map.start, map.goal);
    const url = URL.createObjectURL(new Blob([text], { type: "text/plain" }));
    const a = el("a", { href: url, download: `${currentPreset === UPLOAD_KEY ? "map" : currentPreset}.txt` });
    a.click();
    // Revoking at once can beat the download on some browsers.
    setTimeout(() => URL.revokeObjectURL(url), 10000);
    setMapStatus("Downloaded the map as it is drawn right now, walls, start and goal included.");
  });

  syncUploadCard();
  cards[currentPreset].card.setAttribute("aria-pressed", "true");
  drawThumbs();

  presetBox.append(el("button", {
    type: "button", class: "btn", text: "Clear walls",
    onclick: () => {
      map.grid.cells.fill(0);
      keepAsPicked();
      currentPreset = null;
      for (const { card } of Object.values(cards)) card.setAttribute("aria-pressed", "false");
      mapChanged(true);
    },
  }));

  // The map as it was picked, so "Clear drawing" can take away what was drawn
  // on it, and only that.
  let asPicked = null;
  const undraw = $("#plan-undraw");
  function keepAsPicked() {
    asPicked = cloneMap(map);
    undraw.disabled = true;
  }
  undraw.addEventListener("click", () => {
    map = cloneMap(asPicked);
    undraw.disabled = true;
    mapChanged(true);
  });
  keepAsPicked();

  function mapChanged(hard = false) {
    setStageAspect();
    if (hard) resetRun();
    else invalidateRun();
    scheduleCompare();
    requestRender();
  }

  // ---- Playback ----
  const atEnd = () => gen && curStep >= totalSteps;
  $("#plan-run").addEventListener("click", () => {
    // Finished runs start over; a run scrubbed back to the middle resumes.
    if (!gen || atEnd()) newRun();
    playing = true;
    requestRender();
  });
  $("#plan-step").addEventListener("click", () => {
    if (!gen || atEnd()) newRun();
    playing = false;
    seekTo(curStep + 1);
    requestRender();
  });
  $("#plan-back").addEventListener("click", () => {
    // Never starts a run: there is nothing behind the beginning of one.
    playing = false;
    seekTo(curStep - 1);
    requestRender();
  });
  $("#plan-pause").addEventListener("click", () => {
    playing = false;
    requestRender();
  });
  $("#plan-reset").addEventListener("click", () => {
    resetRun();
    requestRender();
  });

  // The scrubber. Dragging fires many input events per frame, so the seek is
  // deferred to the next frame and only the last position of each frame runs.
  const scrub = $("#plan-scrub");
  scrub.addEventListener("input", () => {
    if (!gen) {
      scrub.value = "0";
      return;
    }
    playing = false;
    pendingSeek = Number(scrub.value);
    requestRender();
  });
  function syncScrub() {
    const label = $("#plan-scrub-num");
    scrub.disabled = !gen;
    scrub.max = String(Math.max(1, totalSteps));
    if (document.activeElement !== scrub || pendingSeek === null) scrub.value = String(curStep);
    const unit = ALGOS[opt.algo].kind === "tree" ? "iteration" : "step";
    label.textContent = gen ? `${unit} ${curStep} of ${totalSteps}` : "nothing recorded yet";
  }

  const makeGen = () => planner(opt.algo, map.grid, map.start, map.goal, runOpts);
  /** The length of the recording, so the scrubber can span the whole run. */
  function countSteps() {
    let n = 0;
    for (const _ of makeGen()) n++;
    return n;
  }
  function newRun() {
    // Sampling planners draw new points every run; grid searches never vary.
    const random = ALGOS[opt.algo].kind !== "grid";
    runOpts = { ...opt, seed: random ? runSeed++ : 0 };
    totalSteps = countSteps();
    gen = makeGen();
    cur = null;
    curStep = 0;
    stepAcc = 0;
  }
  function resetRun() {
    gen = null;
    cur = null;
    curStep = 0;
    totalSteps = 0;
    runOpts = null;
    pendingSeek = null;
    playing = false;
  }
  /** Move the recording to `target`. Backward means replaying from the start. */
  function seekTo(target) {
    if (!gen) return;
    target = Math.max(0, Math.min(totalSteps, target));
    if (target < curStep) {
      gen = makeGen();
      cur = null;
      curStep = 0;
    }
    advance(target - curStep, Infinity);
  }
  /**
   * After an edit, a finished grid search reruns at once so the path follows
   * the edit live. Tree runs are random and slow to redo, so they reset.
   */
  function invalidateRun() {
    const replan = cur?.done && !playing && ALGOS[opt.algo].kind === "grid";
    resetRun();
    if (replan) {
      newRun();
      advance(Infinity, Infinity);
    }
    requestRender();
  }
  function advance(n, timeLimit = FRAME_MS) {
    if (!gen || n <= 0) return;
    const t0 = performance.now();
    const target = n === Infinity ? totalSteps : Math.min(totalSteps, curStep + n);
    while (curStep < target) {
      const r = gen.next();
      if (r.done) {
        // The recording turned out shorter than measured: trust what ran.
        totalSteps = curStep;
        break;
      }
      cur = r.value;
      curStep++;
      // Checking the clock every step would cost more than the steps.
      if ((curStep & 63) === 63 && performance.now() - t0 > timeLimit) break;
    }
    if (curStep >= totalSteps) playing = false;
  }

  // ---- Canvas sizing ----
  function setStageAspect() {
    wrap.style.aspectRatio = `${map.grid.w} / ${map.grid.h}`;
  }
  setStageAspect();
  const canvas = $("#plan-canvas");
  let size = null;
  new ResizeObserver(() => {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(canvas.clientWidth * dpr);
    canvas.height = Math.round(canvas.clientHeight * dpr);
    size = { w: canvas.clientWidth, h: canvas.clientHeight, dpr };
    requestRender();
  }).observe(canvas);

  // ---- Pointer editing ----
  let drag = null; // "start" | "goal" | "draw" | "erase"
  let lastCell = null;

  function cellAt(e) {
    const r = canvas.getBoundingClientRect();
    const fx = ((e.clientX - r.left) / r.width) * map.grid.w;
    const fy = ((e.clientY - r.top) / r.height) * map.grid.h;
    return {
      x: Math.min(map.grid.w - 1, Math.max(0, Math.floor(fx))),
      y: Math.min(map.grid.h - 1, Math.max(0, Math.floor(fy))),
      fx, fy, cellPx: r.width / map.grid.w,
    };
  }
  function nearMarker(p, m) {
    // Markers get a finger-sized hit area even when cells are tiny.
    const reach = Math.max(1, 16 / p.cellPx);
    return Math.hypot(p.fx - (m.x + 0.5), p.fy - (m.y + 0.5)) <= reach;
  }
  const isMarker = (x, y) => (x === map.start.x && y === map.start.y) || (x === map.goal.x && y === map.goal.y);

  canvas.addEventListener("contextmenu", (e) => e.preventDefault());
  canvas.addEventListener("pointerdown", (e) => {
    const p = cellAt(e);
    if (nearMarker(p, map.start)) drag = "start";
    else if (nearMarker(p, map.goal)) drag = "goal";
    else drag = editMode === "erase" || e.shiftKey || e.altKey || e.button === 2 ? "erase" : "draw";
    canvas.setPointerCapture(e.pointerId);
    lastCell = null;
    pointerAt(p);
  });
  canvas.addEventListener("pointermove", (e) => {
    const p = cellAt(e);
    if (drag) pointerAt(p);
    else canvas.style.cursor = nearMarker(p, map.start) || nearMarker(p, map.goal) ? "grab" : "";
  });
  const endDrag = () => {
    drag = null;
    lastCell = null;
  };
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", endDrag);

  function pointerAt(p) {
    const { grid } = map;
    if (drag === "start" || drag === "goal") {
      const m = map[drag], other = map[drag === "start" ? "goal" : "start"];
      if (grid.cells[p.y * grid.w + p.x] || (p.x === other.x && p.y === other.y)) return;
      if (p.x === m.x && p.y === m.y) return;
      m.x = p.x;
      m.y = p.y;
      undraw.disabled = false;
      mapChanged();
      return;
    }
    const v = drag === "draw" ? 1 : 0;
    let changed = false;
    // Fill the gap since the last event so fast strokes stay unbroken.
    for (const [x, y] of lineCells(lastCell ?? p, p)) {
      if (isMarker(x, y)) continue;
      const i = y * grid.w + x;
      if (grid.cells[i] !== v) {
        grid.cells[i] = v;
        changed = true;
      }
    }
    lastCell = p;
    if (changed) {
      undraw.disabled = false;
      mapChanged();
    }
  }

  function lineCells(a, b) {
    const out = [];
    let x = a.x, y = a.y;
    const dx = Math.abs(b.x - x), dy = -Math.abs(b.y - y);
    const sx = x < b.x ? 1 : -1, sy = y < b.y ? 1 : -1;
    let err = dx + dy;
    for (;;) {
      out.push([x, y]);
      if (x === b.x && y === b.y) return out;
      const e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y += sy;
      }
    }
  }

  // ---- Drawing ----
  // Drawn in device pixels with rounded cell edges, so neighboring cells
  // never show hairline seams.
  function draw() {
    if (!size || !size.w) return;
    const ctx = canvas.getContext("2d");
    const W = canvas.width, H = canvas.height, dpr = size.dpr;
    const kx = W / map.grid.w, ky = H / map.grid.h;
    const X = (x) => Math.round(x * kx), Y = (y) => Math.round(y * ky);
    const cell = (x, y) => ctx.fillRect(X(x), Y(y), X(x + 1) - X(x), Y(y + 1) - Y(y));
    const { grid } = map;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.globalAlpha = 1;
    ctx.fillStyle = colors.panel;
    ctx.fillRect(0, 0, W, H);

    if (cur?.kind === "grid") {
      // accent-soft alone barely shows on the panel color, so closed cells
      // use the accent at low alpha over it.
      ctx.fillStyle = colors.accentSoft;
      for (let i = 0; i < cur.mark.length; i++) if (cur.mark[i] === 2) cell(i % grid.w, Math.floor(i / grid.w));
      ctx.fillStyle = colors.accent;
      ctx.globalAlpha = 0.2;
      for (let i = 0; i < cur.mark.length; i++) if (cur.mark[i] === 2) cell(i % grid.w, Math.floor(i / grid.w));
      ctx.fillStyle = colors.highlight;
      ctx.globalAlpha = 0.5;
      for (let i = 0; i < cur.mark.length; i++) if (cur.mark[i] === 1) cell(i % grid.w, Math.floor(i / grid.w));
      ctx.globalAlpha = 1;
    }

    ctx.fillStyle = colors.solid;
    for (let i = 0; i < grid.cells.length; i++) if (grid.cells[i]) cell(i % grid.w, Math.floor(i / grid.w));

    if (kx / dpr >= 9) {
      ctx.strokeStyle = colors.line;
      ctx.lineWidth = 1;
      ctx.globalAlpha = 0.7;
      ctx.beginPath();
      for (let x = 1; x < map.grid.w; x++) {
        ctx.moveTo(X(x) + 0.5, 0);
        ctx.lineTo(X(x) + 0.5, H);
      }
      for (let y = 1; y < map.grid.h; y++) {
        ctx.moveTo(0, Y(y) + 0.5);
        ctx.lineTo(W, Y(y) + 0.5);
      }
      ctx.stroke();
      ctx.globalAlpha = 1;
    }

    if (cur?.kind === "grid" && cur.current >= 0 && !cur.found) {
      const x = cur.current % grid.w, y = Math.floor(cur.current / grid.w);
      ctx.fillStyle = colors.accent;
      cell(x, y);
    }

    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    if (cur?.kind === "tree") {
      ctx.strokeStyle = colors.lineStrong;
      ctx.lineWidth = Math.max(1, dpr);
      ctx.beginPath();
      for (let k = 1; k < cur.n; k++) {
        const p = cur.parent[k];
        ctx.moveTo(cur.xs[p] * kx, cur.ys[p] * ky);
        ctx.lineTo(cur.xs[k] * kx, cur.ys[k] * ky);
      }
      ctx.stroke();
      // The neighborhood RRT* searched for the newest node.
      if (opt.algo === "rrtstar" && cur.newNode > 0 && cur.radius > 0 && !cur.done) {
        ctx.strokeStyle = colors.accent;
        ctx.lineWidth = 1.5 * dpr;
        ctx.setLineDash([4 * dpr, 4 * dpr]);
        ctx.beginPath();
        ctx.ellipse(cur.xs[cur.newNode] * kx, cur.ys[cur.newNode] * ky, cur.radius * kx, cur.radius * ky, 0, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      if (cur.sample && !cur.done) {
        ctx.fillStyle = colors.highlight;
        ctx.beginPath();
        ctx.arc(cur.sample.x * kx, cur.sample.y * ky, Math.max(3 * dpr, kx * 0.3), 0, Math.PI * 2);
        ctx.fill();
      }
    }

    if (cur?.kind === "roadmap") {
      ctx.strokeStyle = colors.lineStrong;
      ctx.lineWidth = Math.max(1, dpr);
      ctx.globalAlpha = 0.8;
      ctx.beginPath();
      const E = cur.edges;
      for (let m = 0; m < E.length; m += 2) {
        ctx.moveTo(cur.xs[E[m]] * kx, cur.ys[E[m]] * ky);
        ctx.lineTo(cur.xs[E[m + 1]] * kx, cur.ys[E[m + 1]] * ky);
      }
      ctx.stroke();
      ctx.globalAlpha = 1;
      const dotR = Math.max(1.6 * dpr, kx * 0.18);
      ctx.fillStyle = colors.fg;
      ctx.beginPath();
      for (let i = 2; i < cur.n; i++) {
        ctx.moveTo(cur.xs[i] * kx + dotR, cur.ys[i] * ky);
        ctx.arc(cur.xs[i] * kx, cur.ys[i] * ky, dotR, 0, Math.PI * 2);
      }
      ctx.fill();
      // Nodes the query has closed, as the grid searches show closed cells.
      if (cur.closed) {
        ctx.fillStyle = colors.accent;
        ctx.beginPath();
        for (let i = 2; i < cur.n; i++) {
          if (!cur.closed[i]) continue;
          ctx.moveTo(cur.xs[i] * kx + dotR * 1.6, cur.ys[i] * ky);
          ctx.arc(cur.xs[i] * kx, cur.ys[i] * ky, dotR * 1.6, 0, Math.PI * 2);
        }
        ctx.fill();
      }
      if (cur.current >= 2 && !cur.done) {
        ctx.fillStyle = colors.highlight;
        ctx.beginPath();
        ctx.arc(cur.xs[cur.current] * kx, cur.ys[cur.current] * ky, Math.max(3.5 * dpr, kx * 0.35), 0, Math.PI * 2);
        ctx.fill();
      }
    }

    if (cur?.path) {
      ctx.strokeStyle = colors.accent;
      ctx.lineWidth = Math.max(3 * dpr, kx * 0.4);
      ctx.beginPath();
      cur.path.forEach((p, k) => (k ? ctx.lineTo(p.x * kx, p.y * ky) : ctx.moveTo(p.x * kx, p.y * ky)));
      ctx.stroke();
    }

    // Start is a circle, goal a diamond, so they differ without color.
    const r = Math.max(7 * dpr, kx * 0.8);
    const sx = (map.start.x + 0.5) * kx, sy = (map.start.y + 0.5) * ky;
    const gx = (map.goal.x + 0.5) * kx, gy = (map.goal.y + 0.5) * ky;
    ctx.lineWidth = 2 * dpr;
    ctx.strokeStyle = colors.panel;
    ctx.fillStyle = colors.start;
    ctx.beginPath();
    ctx.arc(sx, sy, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = colors.goal;
    ctx.beginPath();
    ctx.moveTo(gx, gy - r * 1.2);
    ctx.lineTo(gx + r * 1.2, gy);
    ctx.lineTo(gx, gy + r * 1.2);
    ctx.lineTo(gx - r * 1.2, gy);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }

  // ---- Readouts ----
  function updateReadouts() {
    const tree = ALGOS[opt.algo].kind === "tree";
    const roadmap = ALGOS[opt.algo].kind === "roadmap";
    const set = (id, text, pending = false) => {
      const node = $(id);
      node.textContent = text;
      node.classList.toggle("pending", pending);
      // Mono is for numbers only; words like "yes" stay in the text face.
      node.style.fontFamily = /\d/.test(text) ? "" : "var(--font-sans)";
    };
    if (!cur) {
      for (const id of ["#plan-m-work", "#plan-m-frontier", "#plan-m-length", "#plan-m-found"]) set(id, "n/a", true);
    } else {
      set("#plan-m-work", tree ? `${cur.iter} / ${cur.budget}` : roadmap ? `${cur.n - 2} / ${cur.samples}` : String(cur.expanded));
      set("#plan-m-frontier", String(tree ? cur.n : roadmap ? cur.edgeCount : cur.frontier));
      set("#plan-m-length", cur.found ? fmt(cur.cost, 2) : "n/a", !cur.found);
      set("#plan-m-found", cur.found ? "yes" : cur.done ? "no" : "not yet", !cur.done && !cur.found);
    }

    const badge = $("#plan-badge");
    let text, bad = false;
    if (!cur) text = gen ? "Ready" : "Press Run or Step";
    else if (cur.done && !cur.found) {
      text = tree ? "Budget used, no path" : roadmap ? "No path through the roadmap" : "No path";
      bad = true;
    } else if (cur.done) text = tree && opt.algo === "rrtstar" ? "Done, best path shown" : "Path found";
    else if (playing && roadmap) text = { sample: "Sampling", connect: "Connecting", query: "Searching the roadmap" }[cur.phase];
    else if (playing) text = tree ? (cur.found ? "Improving the path" : "Growing the tree") : "Searching";
    else text = "Paused";
    badge.textContent = text;
    badge.className = bad ? "sim-badge bad" : "sim-badge";
  }

  // ---- Compare table ----
  function scheduleCompare() {
    compareStale = true;
    clearTimeout(compareTimer);
    if (!active) return;
    compareTimer = setTimeout(() => {
      compareRows = compareAll(map.grid, map.start, map.goal, { ...opt, seed: COMPARE_SEED });
      compareStale = false;
      renderCompare();
      updateMath();
    }, COMPARE_DELAY);
  }

  function renderCompare() {
    const body = $("#plan-compare-body");
    if (!compareRows) return;
    const sans = "font-family:var(--font-sans)";
    body.replaceChildren(...compareRows.map((r) => {
      const a = ALGOS[r.algo];
      const name = r.algo === "wastar" ? `${a.name} (ε = ${fmt(opt.weight, 1)})` : a.name;
      const tree = a.kind === "tree";
      const unit = tree ? " iter." : a.kind === "roadmap" ? " nodes" : null;
      const workCell = el("td", {}, el("span", { text: String(r.work) }), unit ? el("span", { class: "subtle", style: sans, text: unit }) : null);
      const row = el("tr", { style: r.algo === opt.algo ? "background:var(--accent-soft)" : "" },
        el("th", { class: "row", scope: "row", text: name }),
        workCell,
        r.found ? el("td", { text: fmt(r.length, 2) }) : el("td", { style: sans, class: "subtle", text: "no path" }),
        r.ratio !== null ? el("td", { text: `${fmt(r.ratio, 3)}×` }) : el("td", { style: sans, class: "subtle", text: "n/a" }),
        el("td", {
          style: sans,
          class: r.optimal === true ? "ok" : r.optimal === false ? "bad" : "subtle",
          text: r.optimal === true ? "yes" : r.optimal === false ? "no" : "n/a",
        }),
        el("td", { text: fmt(r.ms, 1) }),
      );
      return row;
    }));
  }

  // ---- The math ----
  // Equations are LaTeX typeset by KaTeX (see tex.js); sentences stay plain
  // text. Everything passed in is built from numbers, never user text.
  const H_FORM = {
    manhattan: "h(n) = |\\Delta x| + |\\Delta y|",
    octile: "h(n) = \\max(|\\Delta x|, |\\Delta y|) + (\\sqrt{2} - 1)\\min(|\\Delta x|, |\\Delta y|)",
    euclidean: "h(n) = \\sqrt{\\Delta x^2 + \\Delta y^2}",
  };
  const ALGO_F = {
    bfs: "BFS (breadth-first search): no priority. First found, first expanded.",
    dijkstra: `Dijkstra: ${inl("h = 0")}, so ${inl("f = g")}.`,
    greedy: `Greedy: ${inl("g")} ignored, so ${inl("f = h")}.`,
    astar: `A*: ${inl("\\varepsilon = 1")}.`,
    wastar: null, // filled live with the current ε
    rrt: "RRT, RRT* (rapidly-exploring random tree): no priority. The tree grows toward random samples.",
    prm: `PRM (probabilistic roadmap): A* over the roadmap's links, with ${inl("h")} the straight-line distance.`,
  };
  const mk = (tag, cls) => el(tag, { class: cls });
  tex($("#plan-math-f"), "f(n) = g(n) + \\varepsilon\\, h(n)");
  $("#plan-math-defs").innerHTML =
    `${inl("g")}: cost from the start so far. ${inl("h")}: estimated cost to the goal. Moves cost ${inl("1")} straight and ${inl("\\sqrt{2} \\approx 1.414")} diagonally; a diagonal needs both side cells free.`;
  // A sentence under the live row for when there is no node to show.
  const liveNote = mk("p", "math-status subtle");
  $("#plan-math-live").after(liveNote);
  $("#plan-math-adm").innerHTML =
    `Admissible: ${inl("h(n) \\le h^*(n)")}, never more than the true cost to go. Consistent: ${inl("h(n) \\le c(n, m) + h(m)")} for every move from ${inl("n")} to ${inl("m")}.`;
  // The guarantee block: an optional bound row, then a sentence.
  const guarantee = $("#plan-math-guarantee");
  guarantee.className = "";
  const boundRow = mk("div", "eq");
  const guaranteeText = mk("p", "math-status");
  guarantee.replaceChildren(boundRow, guaranteeText);
  tex($("#plan-math-r"), "r = \\min\\!\\big(\\gamma\\,(\\log n / n)^{1/2},\\ \\text{step}\\big)");
  tex($("#plan-math-gamma"), "\\gamma = 2\\Big(1 + \\frac{1}{d}\\Big)^{1/d} \\Big(\\frac{\\mu(X_\\text{free})}{\\zeta_d}\\Big)^{1/d}, \\quad d = 2,\\ \\zeta_2 = \\pi");
  const rText = $("#plan-math-r").parentElement.querySelector("p:not([id]):not(.eyebrow)");
  if (rText) {
    rText.innerHTML = `Each new node takes the cheapest parent within ${inl("r")}, then any neighbor within ${inl("r")} that would be cheaper through the new node switches to it. Edges are checked cell by cell, so no corner is clipped.`;
  }

  tex($("#plan-math-prm-v"), "V = \\{q_\\text{start},\\ q_\\text{goal},\\ q_1, \\dots, q_n\\}");
  tex($("#plan-math-prm-e"), "E = \\{\\, (i, j) : j \\in \\operatorname{kNN}(i),\\ \\overline{q_i q_j} \\text{ free} \\,\\}");
  $("#plan-math-prm-note").innerHTML = `${inl("q_1")} to ${inl("q_n")} are random points in free space. ${inl("\\operatorname{kNN}(i)")} is the ${inl("k")} nearest neighbors of point ${inl("i")}, and ${inl("\\overline{q_i q_j}")} is the straight line between two points. A link is kept only if that line misses every wall. A* then searches this network instead of the grid. Nothing in the roadmap depends on the start or goal, so on a map that doesn't change, a robot builds it once and reuses it for every trip.`;

  let mathClock = 0;
  function updateMath() {
    const algo = opt.algo;
    const tree = ALGOS[algo].kind === "tree";
    const roadmap = ALGOS[algo].kind === "roadmap";
    const optimum = compareRows?.find((r) => r.algo === "dijkstra")?.length ?? null;

    // Priority: which line applies, and the live node.
    const lines = Object.entries(ALGO_F).map(([k, text]) => {
      const t = k === "wastar" ? `Weighted A*: ${inl(`\\varepsilon = ${num(opt.weight, 1)}`)}.` : text;
      const on = k === algo || (k === "rrt" && tree);
      return `<div${on ? ' style="font-weight:600;color:var(--fg)"' : ""}>${t}</div>`;
    });
    const algosHtml = lines.join("");
    const algosEl = $("#plan-math-algos");
    if (algosEl.dataset.src !== algosHtml) {
      algosEl.dataset.src = algosHtml;
      algosEl.innerHTML = algosHtml;
    }
    const liveEl = $("#plan-math-live");
    if (!tree && cur?.kind === "grid" && cur.current >= 0) {
      const x = cur.current % map.grid.w, y = Math.floor(cur.current / map.grid.w);
      const node = `n = (${x}, ${y}):\\quad g = ${num(cur.gCur)}`;
      let live;
      if (algo === "bfs") live = `${node} \\quad (h \\text{ not used})`;
      else if (algo === "greedy") live = `${node},\\ \\ h = ${num(cur.hCur)},\\ \\ f = h = ${num(cur.fCur)}`;
      else if (algo === "dijkstra") live = `${node},\\ \\ h = 0,\\ \\ f = ${num(cur.fCur)}`;
      else live = `${node},\\ \\ h = ${num(cur.hCur)},\\ \\ f = ${num(cur.gCur)} + ${num(cur.eps, 1)} \\cdot ${num(cur.hCur)} = ${num(cur.fCur)}`;
      liveEl.hidden = false;
      tex(liveEl, live);
      liveNote.hidden = true;
    } else {
      liveEl.hidden = true;
      liveNote.hidden = false;
      liveNote.textContent = tree ? "RRT and RRT* do not rank nodes."
        : roadmap ? "PRM ranks roadmap nodes, not cells, once the roadmap is built."
        : "Press Step to see g, h and f for each expanded node.";
    }

    // Heuristic
    const props = heuristicProps(opt.heuristic, opt.conn);
    tex($("#plan-math-h"), H_FORM[opt.heuristic]);
    const hv = $("#plan-math-hverdict");
    const hName = HEURISTICS[opt.heuristic].name;
    const best = HEURISTICS[defaultHeuristic(opt.conn)].name;
    const onGrid = opt.conn === 8 ? "on an 8-connected grid" : "on a 4-connected grid";
    let verdict;
    if (!props.admissible) {
      verdict = `${hName} ${onGrid} is not admissible: one diagonal step costs ${inl("\\sqrt{2} \\approx 1.414")}, but ${hName} counts ${inl("2")}. A* can return a path up to ${inl("\\sqrt{2}")} times the optimum.`;
    } else if (props.exact) {
      verdict = `${hName} ${onGrid} is admissible and consistent. On an empty grid it equals the true cost, so it is the strongest choice here.`;
    } else {
      verdict = `${hName} ${onGrid} is admissible and consistent, but weaker than ${best}: it underestimates more, so A* expands more cells.`;
    }
    if (hv.dataset.src !== verdict) {
      hv.dataset.src = verdict;
      hv.innerHTML = verdict;
    }
    hv.className = `small verdict ${props.admissible ? "is-ok" : "is-bad"}`;
    const sx = Math.abs(map.start.x - map.goal.x), sy = Math.abs(map.start.y - map.goal.y);
    const hStart = HEURISTICS[opt.heuristic].f(sx, sy);
    tex($("#plan-math-hlive"), optimum !== null
      ? `h(\\text{start}) = ${num(hStart)},\\quad h^*(\\text{start}) = ${num(optimum)}`
      : `h(\\text{start}) = ${num(hStart)}`);

    // Guarantee for the selected planner
    const check = $("#plan-math-check");
    const found = cur?.done && cur.found ? cur.cost : null;
    let bound = null;
    let form = null;
    let text;
    if (algo === "bfs") {
      text = opt.conn === 4
        ? "Fewest moves. With 4 connections every move costs 1, so fewest moves is also shortest."
        : `Fewest moves. A diagonal costs ${inl("\\sqrt{2}")} but counts as one move, so fewest moves is not always shortest.`;
      if (opt.conn === 4) bound = 1;
    } else if (algo === "dijkstra") {
      form = "\\text{cost} = \\text{optimal}";
      text = `Optimal. It expands every cell with ${inl("g")} below the optimal cost before it reaches the goal.`;
      bound = 1;
    } else if (algo === "greedy") {
      text = "No bound on cost. It ignores how far it has come, so a wall in the way can send it on a long detour.";
    } else if (algo === "astar" || algo === "wastar") {
      const eps = algo === "astar" ? 1 : opt.weight;
      bound = eps * props.factor;
      form = algo === "astar" ? "\\text{cost} = \\text{optimal}" : "\\text{cost} \\le \\varepsilon \\cdot \\text{optimal}";
      if (props.factor > 1) form = algo === "astar" ? "\\text{cost} \\le \\sqrt{2} \\cdot \\text{optimal}" : "\\text{cost} \\le \\varepsilon \\cdot \\sqrt{2} \\cdot \\text{optimal}";
      text = algo === "wastar"
        ? `Scaling ${inl("h")} by ${inl("\\varepsilon")} makes the search head for the goal sooner and expand fewer cells. In exchange the path can cost up to ${inl("\\varepsilon")} times the optimum.`
        : props.admissible ? `Optimal, because ${inl("h")} never overestimates.` : `Not guaranteed optimal, because ${inl("h")} can overestimate.`;
    } else if (algo === "prm") {
      text = "Probabilistically complete, like RRT. The path is the shortest one through the roadmap, not through free space, so it's only as good as the points it scattered. Narrow gaps need many samples, because a point has to land in the gap.";
    } else if (algo === "rrt") {
      text = "Probabilistically complete: if a path exists, the chance of finding it goes to 1 as samples grow. The first path found is kept, so it is rarely short.";
    } else {
      text = "Asymptotically optimal: as samples grow, the path cost converges to the shortest path through free space, which can beat the grid optimum.";
    }
    boundRow.hidden = form === null;
    if (form) tex(boundRow, form);
    if (guaranteeText.dataset.src !== text) {
      guaranteeText.dataset.src = text;
      guaranteeText.innerHTML = text;
    }
    if (found !== null && optimum !== null && bound !== null) {
      const ok = found <= bound * optimum + 1e-9;
      check.hidden = false;
      check.className = `small verdict ${ok ? "is-ok" : "is-bad"}`;
      check.innerHTML = `This run: ${inl(`${num(found)} ${ok ? "\\le" : ">"} ${num(bound, 3)} \\cdot ${num(optimum)} = ${num(bound * optimum)}`)}, so the bound <span class="${ok ? "ok" : "bad"}">${ok ? "holds" : "fails"}</span>.`;
    } else if (found !== null && optimum !== null) {
      check.hidden = false;
      check.className = "small verdict";
      check.innerHTML = `This run: ${inl(`${num(found)} / ${num(optimum)} = ${num(found / optimum, 3)}`)} times the grid optimum.`;
    } else check.hidden = true;

    // PRM roadmap size
    const rm = roadmap && cur?.kind === "roadmap" ? cur : null;
    tex($("#plan-math-prm-live"), rm
      ? `n = ${rm.n - 2},\\ \\ k = ${rm.k},\\ \\ |E| = ${rm.edgeCount}`
      : `n = ${opt.samples},\\ \\ k = ${opt.k}`);

    // RRT* radius
    const gamma = rrtGamma(freeCount(map.grid));
    const n = algo === "rrtstar" && cur?.kind === "tree" ? cur.n : opt.budget;
    const r = rrtRadius(gamma, n, opt.step);
    const raw = n >= 2 ? gamma * Math.sqrt(Math.log(n) / n) : 0;
    tex($("#plan-math-rlive"),
      `\\gamma = ${num(gamma, 1)},\\ \\ n = ${n},\\ \\ r = \\min(${num(raw)},\\ ${num(opt.step, 1)}) = ${num(r)}`);
    $("#plan-math-rnote").innerHTML = algo === "rrtstar" && cur
      ? `Live from the current run. ${cur.rewired ?? 0} neighbors rewired on the last iteration.`
      : `Shown for ${inl(`n = ${n}`)}, the iteration budget. Select RRT* and run it to see ${inl("r")} shrink as the tree grows.`;
  }

  // ---- Loop ----
  function requestRender() {
    if (!raf && active) raf = requestAnimationFrame(frame);
  }
  function frame(now) {
    raf = 0;
    if (!active) return;
    if (pendingSeek !== null) {
      seekTo(pendingSeek);
      pendingSeek = null;
    }
    if (playing && gen) {
      const s = SPEEDS[speedIdx];
      stepAcc += s;
      const n = s === Infinity ? Infinity : Math.floor(stepAcc);
      if (n > 0) {
        if (s !== Infinity) stepAcc -= n;
        advance(n, s === Infinity ? Infinity : FRAME_MS);
      }
    }
    draw();
    updateReadouts();
    syncScrub();
    // The math panel rebuilds HTML, so during playback it runs at about 10 Hz.
    if (!playing || now - mathClock > 100) {
      mathClock = now;
      updateMath();
    }
    if (playing) requestRender();
  }

  syncSpeed();
  syncAlgo();
  scheduleCompare();
  requestRender();
  const arm = createArmPlanner(root);
  arm.setActive(true);

  return {
    setActive(on) {
      active = on;
      arm.setActive(on);
      if (!on) {
        clearTimeout(compareTimer);
        return;
      }
      if (compareStale) scheduleCompare();
      requestRender();
    },
  };
}
