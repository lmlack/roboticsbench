// Localization UI: a top-down map with landmarks, walls, the true robot and
// five estimators (particle cloud, EKF and UKF ellipses, grid heatmap, dead
// reckoning), a zoom view around the robot, an error chart, and the math with
// live numbers. The simulation lives in localization-sim.js, the math panel
// in localization-math.js.

import {
  GRID, HISTORY_SECONDS, Localization, RECOVERED, WALL_T, WORLD, distToSegment,
} from "./localization-sim.js";
import { cssVar, evalNumber, fmt, segmented } from "./util.js";
import { createMathPanel } from "./localization-math.js";

const DEG = Math.PI / 180;
const DRIVE = { v: 0.8, w: 1.2 }; // manual speeds, m/s and rad/s
const MAX_LANDMARKS = 24;
const MAX_WALLS = 16;
const MIN_WALL = 0.3; // m, shorter drags count as a click
const KEYS = {
  arrowup: "fwd", w: "fwd", arrowdown: "back", s: "back",
  arrowleft: "left", a: "left", arrowright: "right", d: "right",
};
// Drawing order: dead reckoning and grid underneath, Gaussians on top.
const ALGOS = [
  { key: "dr", name: "Dead reckoning", color: "--fg-subtle", dashed: true },
  { key: "grid", name: "Grid", color: "--plot-p" },
  { key: "pf", name: "Particle filter", color: "--plot-measured" },
  { key: "ukf", name: "UKF", color: "--plot-i" },
  { key: "ekf", name: "EKF", color: "--plot-control" },
];
// The table lists them in the order of the toggles.
const TABLE_ORDER = ["pf", "ekf", "ukf", "grid", "dr"];
const NAME = Object.fromEntries(ALGOS.map((a) => [a.key, a.name]));
const HINTS = {
  landmark: "Click to add or remove a landmark",
  wall: "Drag to draw a wall, click one to remove it",
};

export function start(root) {
  const $ = (sel) => root.querySelector(sel);
  // A fresh seed per visit; the tests pin theirs.
  const sim = new Localization({ seed: (Math.random() * 2 ** 32) >>> 0 });
  const shown = new Set();
  // Opens at rest: the first frame is drawn, nothing moves until Play.
  let paused = true;
  let active = false;
  let raf = 0;
  let colors = readColors();
  window.addEventListener("rb:theme", () => (colors = readColors()));

  function readColors() {
    const c = (n) => cssVar(n);
    const out = {
      bg: c("--raised"), panel: c("--panel"), line: c("--line"), lineStrong: c("--line-strong"),
      fg: c("--fg"), muted: c("--fg-muted"), subtle: c("--fg-subtle"), accent: c("--accent"),
      highlight: c("--highlight"), bad: c("--bad-fg"),
    };
    for (const a of ALGOS) out[a.key] = c(a.color);
    return out;
  }
  const on = (key) => shown.has(key);
  const visibleAlgos = () => ALGOS.filter((a) => on(a.key));

  // ---- Algorithm toggles ----
  const legendEl = $("#loc-chart-legend");
  for (const box of root.querySelectorAll("#loc-algos input[data-algo]")) {
    if (box.checked) shown.add(box.dataset.algo);
    box.addEventListener("change", () => {
      if (box.checked) shown.add(box.dataset.algo);
      else shown.delete(box.dataset.algo);
      chartLegend();
      updateReadouts();
      trackStatus();
    });
  }
  function chartLegend() {
    legendEl.innerHTML = TABLE_ORDER.filter(on)
      .map((k) => `<span class="loc-key"><span class="loc-swatch loc-c-${k}"></span>${NAME[k]}</span>`).join("");
  }
  chartLegend();

  // ---- Sliders ----
  const SLIDERS = {
    n: { min: 50, max: 2000, step: 10, digits: 0, get: () => sim.pf.n, set: (v) => sim.pf.resize(Math.round(v)) },
    odom: { min: 0, max: 3, step: 0.1, digits: 1, get: () => sim.params.odomNoise, set: (v) => (sim.params.odomNoise = v) },
    sensor: { min: 0.02, max: 0.5, step: 0.01, digits: 2, get: () => sim.params.sensorNoise, set: (v) => (sim.params.sensorNoise = v) },
    range: { min: 1, max: 12, step: 0.5, digits: 1, get: () => sim.params.maxRange, set: (v) => (sim.params.maxRange = v) },
    // The robot's own top speed is 0.7 m/s, so half a meter per second of wind
    // is most of what it can push against.
    wind: { min: 0, max: 0.5, step: 0.01, digits: 2, get: () => sim.params.wind, set: (v) => (sim.params.wind = v) },
    winddir: { min: 0, max: 355, step: 5, digits: 0, get: () => sim.params.windDir / DEG, set: (v) => (sim.params.windDir = v * DEG) },
  };
  for (const [key, cfg] of Object.entries(SLIDERS)) {
    const slider = $(`#loc-${key}`);
    const num = $(`#loc-${key}-num`);
    Object.assign(slider, { min: cfg.min, max: cfg.max, step: cfg.step, value: cfg.get() });
    num.value = fmt(cfg.get(), cfg.digits);
    const apply = (v) => {
      // Clamp instead of stretching like PID: 10,000 particles would stall the page.
      const c = Math.min(cfg.max, Math.max(cfg.min, v));
      cfg.set(c);
      slider.value = c;
      num.value = fmt(cfg.get(), cfg.digits);
      if (key === "sensor") sensorNote();
      if (key === "wind" || key === "winddir") windNote();
    };
    slider.addEventListener("input", () => apply(Number(slider.value)));
    num.addEventListener("change", () => {
      const v = evalNumber(num.value);
      if (Number.isFinite(v)) apply(v);
      else num.value = fmt(cfg.get(), cfg.digits);
    });
  }
  $("#loc-gusty").addEventListener("change", (e) => {
    sim.params.gusty = e.target.checked;
    windNote();
  });
  function windNote() {
    const v = sim.params.wind;
    const note = $("#loc-wind-note");
    if (v <= 0) {
      note.textContent = "Still air. The robot goes where it is told, give or take the wheel slip.";
      return;
    }
    const deg = Math.round(sim.params.windDir / DEG);
    note.innerHTML =
      `Pushing <span class="mono">${fmt(v, 2)} m/s</span> toward <span class="mono">${deg}&deg;</span>` +
      `${sim.params.gusty ? ", gusting" : ""}. The odometry handed to every filter is still the commanded motion, ` +
      "so the wind is an error none of them models. Dead reckoning has no way to see it; the EKF and UKF gate it out and get lost; " +
      "the particle and grid filters keep hold because they never commit to one answer.";
  }

  function sensorNote() {
    const { r, b } = sim.sigma;
    $("#loc-sensor-note").innerHTML = `Range <span class="mono">σ<sub>r</sub> = ${fmt(r, 2)} m</span>, bearing <span class="mono">σ<sub>φ</sub> = ${fmt(b / DEG, 1)}°</span>`;
  }
  sensorNote();
  windNote();

  // ---- Buttons ----
  let event = null; // the last kidnap or reset, for the status line
  const statusEl = $("#loc-status");
  const DEFAULT_STATUS = "Every estimator starts at the robot's true pose. Kidnap the robot or reset the filters to see them differ.";
  statusEl.textContent = DEFAULT_STATUS;

  const newEvent = (kind) => {
    event = { kind, t0: sim.t, found: {}, below: {} };
    trackStatus();
  };
  $("#loc-kidnap").addEventListener("click", () => {
    sim.kidnap();
    newEvent("kidnap");
  });
  $("#loc-reset").addEventListener("click", () => {
    sim.resetFilters();
    newEvent("reset");
  });
  $("#loc-restart").addEventListener("click", () => {
    // Walls first, so the reset plans the autopilot's route around them.
    sim.restoreWalls();
    sim.reset();
    event = null;
    statusEl.textContent = DEFAULT_STATUS;
    range = null;
    insetR = null;
    flash("Started over. The default walls are back.");
  });
  const pauseBtn = $("#loc-pause");
  function setPaused(p) {
    paused = p;
    pauseBtn.textContent = paused ? "Play" : "Pause";
    // While at rest, Play is the thing to press, so it gets the solid style.
    pauseBtn.classList.toggle("solid", paused);
  }
  pauseBtn.addEventListener("click", () => setPaused(!paused));
  setPaused(true);

  // ---- Driving ----
  const autoBox = $("#loc-auto");
  autoBox.addEventListener("change", () => (sim.autopilot = autoBox.checked));
  const heldKeys = new Set();
  const heldPad = new Set();
  function updateDrive() {
    const held = (d) => heldKeys.has(d) || heldPad.has(d);
    if ([...heldKeys, ...heldPad].length && paused) setPaused(false);
    if ([...heldKeys, ...heldPad].length && sim.autopilot) {
      sim.autopilot = false;
      autoBox.checked = false;
    }
    sim.manual = {
      v: ((held("fwd") ? 1 : 0) - (held("back") ? 1 : 0)) * DRIVE.v,
      w: ((held("left") ? 1 : 0) - (held("right") ? 1 : 0)) * DRIVE.w,
    };
    for (const b of padButtons) b.classList.toggle("is-held", held(b.dataset.drive));
  }
  function releaseAll() {
    heldKeys.clear();
    heldPad.clear();
    updateDrive();
  }

  const map = $("#loc-map");
  const mapWrap = $("#loc-map-wrap");
  const hint = $("#loc-hint");
  let focused = false;
  map.addEventListener("keydown", (e) => {
    const d = KEYS[e.key.toLowerCase()];
    if (!d || e.metaKey || e.ctrlKey || e.altKey) return;
    // Only while the map has focus, so arrow keys still scroll the page elsewhere.
    e.preventDefault();
    heldKeys.add(d);
    updateDrive();
  });
  map.addEventListener("keyup", (e) => {
    const d = KEYS[e.key.toLowerCase()];
    if (d) {
      heldKeys.delete(d);
      updateDrive();
    }
  });
  map.addEventListener("focus", () => {
    focused = true;
    showHint();
  });
  map.addEventListener("blur", () => {
    focused = false;
    showHint();
    releaseAll();
  });

  const padButtons = [...$("#loc-pad").querySelectorAll("[data-drive]")];
  for (const b of padButtons) {
    const d = b.dataset.drive;
    b.addEventListener("pointerdown", (e) => {
      b.setPointerCapture(e.pointerId);
      heldPad.add(d);
      updateDrive();
    });
    const up = () => {
      heldPad.delete(d);
      updateDrive();
    };
    b.addEventListener("pointerup", up);
    b.addEventListener("pointercancel", up);
    b.addEventListener("lostpointercapture", up);
    // Space or Enter on a focused pad button gives one short nudge.
    b.addEventListener("keydown", (e) => {
      if (e.key !== " " && e.key !== "Enter") return;
      e.preventDefault();
      heldPad.add(d);
      updateDrive();
      setTimeout(up, 250);
    });
  }

  // ---- Edit mode and hint ----
  let editMode = "landmark";
  let flashText = null, flashUntil = 0;
  segmented($("#loc-edit"), (v) => {
    editMode = v;
    // Wall drawing needs the drag, so the page must not scroll under it.
    mapWrap.classList.toggle("is-wall-mode", v === "wall");
    showHint();
  });
  $("#loc-clear-walls").addEventListener("click", () => {
    sim.clearWalls();
    flash("Walls cleared. The autopilot replans.");
  });
  function flash(text) {
    flashText = text;
    flashUntil = performance.now() + 2500;
    showHint();
  }
  function showHint() {
    const text = flashText && performance.now() < flashUntil
      ? flashText
      : focused ? "Arrow keys or WASD drive the robot" : HINTS[editMode];
    if (hint.textContent !== text) hint.textContent = text;
  }

  // ---- Canvas sizing ----
  const chart = $("#loc-chart");
  const inset = $("#loc-inset");
  const sizes = new Map();
  for (const c of [map, chart, inset]) {
    new ResizeObserver(() => {
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      c.width = Math.round(c.clientWidth * dpr);
      c.height = Math.round(c.clientHeight * dpr);
      sizes.set(c, { w: c.clientWidth, h: c.clientHeight, dpr });
    }).observe(c);
  }
  const ctxOf = (c) => {
    const s = sizes.get(c);
    if (!s || !s.w) return null;
    const ctx = c.getContext("2d");
    ctx.setTransform(s.dpr, 0, 0, s.dpr, 0, 0);
    return { ctx, w: s.w, h: s.h };
  };

  /** World meters to canvas pixels, fitted and centered, y up. */
  function geom(w, h) {
    const s = Math.min(w / WORLD.w, h / WORLD.h);
    const ox = (w - s * WORLD.w) / 2, oy = (h - s * WORLD.h) / 2;
    return {
      s,
      X: (x) => ox + x * s,
      Y: (y) => oy + (WORLD.h - y) * s,
      toWorld: (px, py) => [(px - ox) / s, WORLD.h - (py - oy) / s],
    };
  }

  // ---- Map pointer: landmarks by click, walls by drag ----
  let down = null; // { px, py, wx, wy } at pointerdown
  let draft = null; // wall being drawn, world coordinates
  const clampX = (x) => Math.min(WORLD.w, Math.max(0, x));
  const clampY = (y) => Math.min(WORLD.h, Math.max(0, y));
  function worldAt(e) {
    const r = map.getBoundingClientRect();
    const g = geom(r.width, r.height);
    const [x, y] = g.toWorld(e.clientX - r.left, e.clientY - r.top);
    return { x, y, g };
  }
  map.addEventListener("pointerdown", (e) => {
    const p = worldAt(e);
    down = { px: e.clientX, py: e.clientY, wx: p.x, wy: p.y };
    if (editMode === "wall") map.setPointerCapture(e.pointerId);
  });
  map.addEventListener("pointermove", (e) => {
    if (!down || editMode !== "wall") return;
    const p = worldAt(e);
    const len = Math.hypot(p.x - down.wx, p.y - down.wy);
    draft = len >= MIN_WALL / 2 ? { ax: clampX(down.wx), ay: clampY(down.wy), bx: clampX(p.x), by: clampY(p.y) } : null;
  });
  map.addEventListener("pointercancel", () => {
    down = null;
    draft = null;
  });
  map.addEventListener("pointerup", (e) => {
    if (!down) return;
    const start = down;
    down = null;
    const p = worldAt(e);
    const moved = Math.hypot(e.clientX - start.px, e.clientY - start.py) > 6;
    if (editMode === "wall") {
      const wall = draft;
      draft = null;
      if (wall && Math.hypot(wall.bx - wall.ax, wall.by - wall.ay) >= MIN_WALL) {
        if (sim.walls.length >= MAX_WALLS) flash(`At most ${MAX_WALLS} walls.`);
        else if (!sim.addWall(wall.ax, wall.ay, wall.bx, wall.by)) flash("A wall cannot go through the robot.");
        return;
      }
      if (!moved) clickWall(p);
      return;
    }
    if (!moved) clickLandmark(p);
  });

  function clickWall({ x, y, g }) {
    const hit = Math.max(0.15, 14 / g.s);
    let best = -1, bestD = hit;
    sim.walls.forEach((w, i) => {
      const d = distToSegment(x, y, w);
      if (d < bestD) {
        bestD = d;
        best = i;
      }
    });
    if (best >= 0) sim.removeWall(best);
    else flash("Drag to draw a wall.");
  }

  function clickLandmark({ x, y, g }) {
    // Hit radius in pixels so landmarks stay easy to tap on a phone.
    const hit = Math.max(0.25, 16 / g.s);
    let best = -1, bestD = hit;
    sim.landmarks.forEach((lm, j) => {
      const d = Math.hypot(lm.x - x, lm.y - y);
      if (d < bestD) {
        bestD = d;
        best = j;
      }
    });
    if (best >= 0) sim.removeLandmark(best);
    else if (x > 0.1 && x < WORLD.w - 0.1 && y > 0.1 && y < WORLD.h - 0.1) {
      if (sim.landmarks.length >= MAX_LANDMARKS) flash(`At most ${MAX_LANDMARKS} landmarks.`);
      else if (!sim.addLandmark(x, y)) flash("A landmark cannot go inside a wall.");
    }
  }

  // ---- Drawing: map ----
  function drawMap() {
    const c = ctxOf(map);
    if (!c) return;
    const { ctx, w, h } = c;
    const g = geom(w, h);
    ctx.clearRect(0, 0, w, h);
    ctx.lineCap = "round";
    ctx.lineJoin = "round";

    // 1 m grid
    ctx.strokeStyle = colors.line;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = 1; x < WORLD.w; x++) {
      ctx.moveTo(g.X(x), g.Y(0));
      ctx.lineTo(g.X(x), g.Y(WORLD.h));
    }
    for (let y = 1; y < WORLD.h; y++) {
      ctx.moveTo(g.X(0), g.Y(y));
      ctx.lineTo(g.X(WORLD.w), g.Y(y));
    }
    ctx.stroke();
    ctx.strokeStyle = colors.lineStrong;
    ctx.lineWidth = 1.5;
    ctx.strokeRect(g.X(0) + 0.75, g.Y(WORLD.h) + 0.75, WORLD.w * g.s - 1.5, WORLD.h * g.s - 1.5);

    // Wind sits behind everything: it is the stage, not an actor.
    if (sim.params.wind > 0) drawWind(ctx, g);

    if (on("grid")) drawHeat(ctx, g);

    const tr = sim.truth;
    const visible = new Set(sim.z.map((m) => m.j));
    const hidden = new Set(sim.blocked);

    // Sensor range and rays, faint: they explain, they are not the subject.
    ctx.save();
    ctx.setLineDash([4, 5]);
    ctx.strokeStyle = colors.lineStrong;
    ctx.globalAlpha = 0.6;
    ctx.beginPath();
    ctx.arc(g.X(tr.x), g.Y(tr.y), sim.params.maxRange * g.s, 0, Math.PI * 2);
    ctx.stroke();
    ray(ctx, g, tr, sim.z.map((m) => m.j), colors.subtle, 0.5, [4, 5]);
    // Blocked views in the warning color, fainter and with a tighter dash.
    ray(ctx, g, tr, sim.blocked, colors.bad, 0.45, [2, 4]);
    ctx.restore();

    drawTrails(ctx, g);

    // Particles: one path of small squares, a single fill call.
    if (on("pf")) {
      const { x, y, n } = sim.pf;
      const size = Math.max(2, Math.min(3, g.s * 0.03));
      ctx.fillStyle = colors.pf;
      ctx.globalAlpha = n > 1000 ? 0.3 : 0.45;
      ctx.beginPath();
      for (let i = 0; i < n; i++) ctx.rect(g.X(x[i]) - size / 2, g.Y(y[i]) - size / 2, size, size);
      ctx.fill();
      ctx.globalAlpha = 1;
    }

    if (on("ukf")) drawGaussian(ctx, g, sim.ukf, colors.ukf);
    if (on("ekf")) drawGaussian(ctx, g, sim.ekf, colors.ekf);

    // Walls, with round caps: the collision and sight tests treat each wall
    // as every point within half its thickness of the center line.
    ctx.strokeStyle = colors.muted;
    ctx.lineWidth = Math.max(3, WALL_T * g.s);
    ctx.beginPath();
    for (const wl of sim.walls) {
      ctx.moveTo(g.X(wl.ax), g.Y(wl.ay));
      ctx.lineTo(g.X(wl.bx), g.Y(wl.by));
    }
    ctx.stroke();
    if (draft) {
      ctx.save();
      ctx.strokeStyle = colors.accent;
      ctx.globalAlpha = 0.7;
      ctx.setLineDash([8, 6]);
      ctx.beginPath();
      ctx.moveTo(g.X(draft.ax), g.Y(draft.ay));
      ctx.lineTo(g.X(draft.bx), g.Y(draft.by));
      ctx.stroke();
      ctx.restore();
    }

    // Landmarks: 4-point stars, numbered to match the math panel. Out of
    // range is dimmed; in range but hidden gets a small slash.
    ctx.font = "15px Hyper, system-ui, sans-serif";
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    sim.landmarks.forEach((lm, j) => {
      const px = g.X(lm.x), py = g.Y(lm.y);
      ctx.globalAlpha = visible.has(j) ? 1 : 0.5;
      star(ctx, px, py, 11, 3.5);
      ctx.fillStyle = colors.highlight;
      ctx.fill();
      ctx.strokeStyle = colors.fg;
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.globalAlpha = 1;
      if (hidden.has(j)) {
        ctx.strokeStyle = colors.bad;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(px - 9, py + 9);
        ctx.lineTo(px + 9, py - 9);
        ctx.stroke();
      }
      ctx.fillStyle = colors.subtle;
      ctx.fillText(String(j + 1), px + 9, py - 5);
    });

    // Estimates drawn a little larger than the truth so they ring it when right.
    const est = sim.estimates();
    for (const a of ALGOS) {
      if (on(a.key)) robot(ctx, g, est[a.key], 1.3, colors[a.key], false, a.dashed);
    }
    robot(ctx, g, tr, 1, colors.fg, true);
  }

  function ray(ctx, g, tr, js, color, alpha, dash) {
    ctx.strokeStyle = color;
    ctx.globalAlpha = alpha;
    ctx.setLineDash(dash);
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (const j of js) {
      const lm = sim.landmarks[j];
      if (!lm) continue;
      ctx.moveTo(g.X(tr.x), g.Y(tr.y));
      ctx.lineTo(g.X(lm.x), g.Y(lm.y));
    }
    ctx.stroke();
  }

  /** Grid belief, max over heading, one rectangle per cell. */
  /**
   * Streaks drifting the way the air pushes, plus the strength in the corner.
   * An invisible force on the robot is worth making visible on the page, or
   * the estimators simply look broken.
   */
  function drawWind(ctx, g) {
    const air = sim.windVector();
    const speed = Math.hypot(air.x, air.y);
    if (speed <= 1e-6) return;
    // Canvas y runs down while world y runs up, so the arrow flips with it.
    const ux = Math.cos(sim.params.windDir), uy = -Math.sin(sim.params.windDir);
    const x0 = g.X(0), y0 = g.Y(WORLD.h), W = WORLD.w * g.s, H = WORLD.h * g.s;
    const span = Math.hypot(W, H) / 2 + 40;
    const across = Math.max(22, g.s * 0.42);
    const along = across * 2.6;
    const len = Math.min(34, g.s * 0.6);
    // The whole field slides along the wind at the wind's own speed, so the
    // streaks move at the rate the robot is actually being pushed.
    const drift = (((sim.t * speed * g.s) % along) + along) % along;
    const cx = x0 + W / 2, cy = y0 + H / 2;

    ctx.save();
    ctx.beginPath();
    ctx.rect(x0, y0, W, H);
    ctx.clip();
    ctx.strokeStyle = colors.subtle;
    ctx.globalAlpha = Math.min(0.42, 0.12 + speed * 0.55);
    ctx.lineWidth = 1.5;
    ctx.lineCap = "round";
    ctx.beginPath();
    for (let a = -span; a <= span; a += across) {
      for (let b = -span; b <= span; b += along) {
        const px = cx + ux * (b + drift) - uy * a;
        const py = cy + uy * (b + drift) + ux * a;
        ctx.moveTo(px, py);
        ctx.lineTo(px + ux * len, py + uy * len);
      }
    }
    ctx.stroke();
    ctx.restore();

    // The rose, bottom right, clear of the robot's loop through the middle.
    const r = Math.max(16, Math.min(24, g.s * 0.42));
    const rx = x0 + W - r - 14, ry = y0 + H - r - 24;
    ctx.save();
    ctx.globalAlpha = 0.9;
    ctx.fillStyle = colors.panel;
    ctx.beginPath();
    ctx.arc(rx, ry, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = colors.line;
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.strokeStyle = colors.bad;
    ctx.fillStyle = colors.bad;
    ctx.lineWidth = 2;
    const tipX = rx + ux * r * 0.68, tipY = ry + uy * r * 0.68;
    ctx.beginPath();
    ctx.moveTo(rx - ux * r * 0.68, ry - uy * r * 0.68);
    ctx.lineTo(tipX, tipY);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(tipX + ux * 6, tipY + uy * 6);
    ctx.lineTo(tipX - ux * 5 - uy * 5, tipY - uy * 5 + ux * 5);
    ctx.lineTo(tipX - ux * 5 + uy * 5, tipY - uy * 5 - ux * 5);
    ctx.closePath();
    ctx.fill();
    ctx.font = "600 12px Hyper, system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    ctx.fillStyle = colors.fg;
    ctx.fillText(`${fmt(speed, 2)} m/s`, rx, ry + r + 3);
    ctx.restore();
  }

  function drawHeat(ctx, g) {
    const grid = sim.grid;
    if (!(grid.heatMax > 0)) return;
    const cs = GRID.cell * g.s;
    ctx.fillStyle = colors.grid;
    for (let j = 0; j < grid.ny; j++) {
      for (let i = 0; i < grid.nx; i++) {
        const f = grid.heat[j * grid.nx + i] / grid.heatMax;
        if (f < 0.01) continue;
        // Square root so a second, weaker hypothesis still shows.
        ctx.globalAlpha = 0.45 * Math.sqrt(f);
        ctx.fillRect(g.X(i * GRID.cell), g.Y((j + 1) * GRID.cell), cs + 0.5, cs + 0.5);
      }
    }
    ctx.globalAlpha = 1;
  }

  // ---- Zoom inset ----
  // When tracking works the estimates sit a few centimeters from the robot,
  // invisible at map scale. The inset is centered on the true robot and draws
  // each estimate as an offset from it, so the last few seconds of estimates
  // show as a trail wandering around the robot. Its scale adapts, so a 2 cm
  // wobble and a 3 m kidnap both fit.
  let insetR = null; // meters shown from center to edge, eased
  const scaleEl = $("#loc-zoom-scale");
  const NICE = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5];

  function drawInset() {
    const c = ctxOf(inset);
    if (!c) return;
    const { ctx, w, h } = c;
    ctx.clearRect(0, 0, w, h);
    const R = Math.min(w, h) / 2 - 4; // px
    const cx = w / 2, cy = h / 2;
    const tr = sim.truth;
    const trail = sim.trail;
    const algos = visibleAlgos();
    const est = sim.estimates();

    // Fit the largest offset in view: trails, estimates and the ellipses.
    let need = 0.03;
    for (const p of trail) {
      for (const a of algos) need = Math.max(need, Math.hypot(p[a.key][0] - p.x, p[a.key][1] - p.y));
    }
    for (const key of ["ekf", "ukf"]) {
      if (!on(key)) continue;
      const e = sim[key].ellipse();
      const [mx, my] = sim[key].mu;
      need = Math.max(need, Math.hypot(mx - tr.x, my - tr.y) + Math.max(e.rx, e.ry));
    }
    need = Math.min(need * 1.25, 6);
    insetR = insetR ? insetR + (need - insetR) * (need > insetR ? 0.3 : 0.05) : need;
    const S = R / insetR; // px per meter
    const X = (dx) => cx + dx * S;
    const Y = (dy) => cy - dy * S;

    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, Math.PI * 2);
    ctx.fillStyle = colors.panel;
    ctx.globalAlpha = 0.94;
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = colors.lineStrong;
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.clip();

    // Distance rings at a round step, labeled once.
    const step = NICE.find((n) => n * 2 >= insetR / 1.5) ?? 5;
    ctx.strokeStyle = colors.line;
    ctx.lineWidth = 1;
    for (let r = step; r < insetR; r += step) {
      ctx.beginPath();
      ctx.arc(cx, cy, r * S, 0, Math.PI * 2);
      ctx.stroke();
    }

    // Walls nearby, so a bump reads at this scale too.
    ctx.strokeStyle = colors.muted;
    ctx.lineCap = "round";
    ctx.lineWidth = Math.max(2, WALL_T * S);
    ctx.beginPath();
    for (const wl of sim.walls) {
      ctx.moveTo(X(wl.ax - tr.x), Y(wl.ay - tr.y));
      ctx.lineTo(X(wl.bx - tr.x), Y(wl.by - tr.y));
    }
    ctx.stroke();

    // Particles, as offsets from the truth.
    if (on("pf")) {
      const { x, y, n } = sim.pf;
      ctx.fillStyle = colors.pf;
      ctx.globalAlpha = 0.35;
      ctx.beginPath();
      for (let i = 0; i < n; i++) ctx.rect(X(x[i] - tr.x) - 1, Y(y[i] - tr.y) - 1, 2, 2);
      ctx.fill();
      ctx.globalAlpha = 1;
    }

    // EKF and UKF 95% ellipses, true to scale here.
    for (const key of ["ukf", "ekf"]) {
      if (!on(key)) continue;
      const e = sim[key].ellipse();
      const [mx, my] = sim[key].mu;
      ctx.beginPath();
      ctx.ellipse(X(mx - tr.x), Y(my - tr.y), Math.max(1.5, e.rx * S), Math.max(1.5, e.ry * S), -e.angle, 0, Math.PI * 2);
      ctx.fillStyle = colors[key];
      ctx.globalAlpha = 0.14;
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = colors[key];
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // Offset trails: where each estimate sat relative to the robot over time.
    for (const a of algos) {
      ctx.strokeStyle = colors[a.key];
      ctx.lineWidth = 2;
      ctx.setLineDash(a.dashed ? [5, 4] : []);
      ctx.beginPath();
      trail.forEach((p, i) => {
        const px = X(p[a.key][0] - p.x), py = Y(p[a.key][1] - p.y);
        if (i === 0 || p.brk) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.globalAlpha = 0.75;
      ctx.stroke();
      ctx.globalAlpha = 1;
    }
    ctx.setLineDash([]);

    // Current estimates and the robot at the center, in pixels so they stay readable.
    for (const a of algos) {
      const p = est[a.key];
      wedge(ctx, X(p.x - tr.x), Y(p.y - tr.y), p.th, 13, colors[a.key], false, a.dashed);
    }
    wedge(ctx, cx, cy, tr.th, 10, colors.fg, true);
    ctx.restore();

    const unit = (m) => (m < 1 ? `${Math.round(m * 100)} cm` : `${m} m`);
    const label = `rings every ${unit(step)}`;
    if (scaleEl.textContent !== label) scaleEl.textContent = label;
  }

  function star(ctx, x, y, R, r) {
    ctx.beginPath();
    for (let k = 0; k < 8; k++) {
      const a = (k * Math.PI) / 4 - Math.PI / 2;
      const rad = k % 2 === 0 ? R : r;
      ctx.lineTo(x + Math.cos(a) * rad, y + Math.sin(a) * rad);
    }
    ctx.closePath();
  }

  /** A wedge pointing along the heading. Screen y is flipped, so -θ. */
  function robot(ctx, g, pose, scale, color, filled, dashed = false) {
    const L = Math.max(0.3, 14 / g.s) * scale;
    wedge(ctx, g.X(pose.x), g.Y(pose.y), pose.th, L * g.s, color, filled, dashed);
  }

  function wedge(ctx, px, py, th, Lpx, color, filled, dashed = false) {
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-th);
    ctx.beginPath();
    ctx.moveTo(Lpx, 0);
    ctx.lineTo(-0.55 * Lpx, 0.55 * Lpx);
    ctx.lineTo(-0.3 * Lpx, 0);
    ctx.lineTo(-0.55 * Lpx, -0.55 * Lpx);
    ctx.closePath();
    if (filled) {
      ctx.fillStyle = color;
      ctx.fill();
      ctx.strokeStyle = colors.panel;
      ctx.lineWidth = 1.5;
      ctx.stroke();
    } else {
      ctx.strokeStyle = color;
      ctx.lineWidth = 2.5;
      ctx.setLineDash(dashed ? [4, 3] : []);
      ctx.stroke();
    }
    ctx.restore();
  }

  /** 95% position ellipse and a ±2σθ heading wedge for a Gaussian filter. */
  function drawGaussian(ctx, g, f, color) {
    const e = f.ellipse();
    const [mx, my, mth] = f.mu;
    const px = g.X(mx), py = g.Y(my);
    ctx.save();
    const half = Math.min(Math.PI, 2 * Math.sqrt(Math.max(0, f.P[2][2])));
    const R = Math.max(0.7 * g.s, 30);
    ctx.fillStyle = color;
    ctx.globalAlpha = 0.12;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.arc(px, py, R, -mth - half, -mth + half);
    ctx.closePath();
    ctx.fill();
    // Floor its size so a tight fix is still visible.
    ctx.globalAlpha = 0.14;
    ctx.beginPath();
    ctx.ellipse(px, py, Math.max(2, e.rx * g.s), Math.max(2, e.ry * g.s), -e.angle, 0, Math.PI * 2);
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.restore();
  }

  function drawTrails(ctx, g) {
    const tr = sim.trail;
    if (tr.length < 2) return;
    const line = (get, color, width, alpha, breaks, dashed) => {
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.globalAlpha = alpha;
      ctx.setLineDash(dashed ? [6, 5] : []);
      ctx.beginPath();
      tr.forEach((p, i) => {
        const [x, y] = get(p);
        if (i === 0 || (breaks && p.brk)) ctx.moveTo(g.X(x), g.Y(y));
        else ctx.lineTo(g.X(x), g.Y(y));
      });
      ctx.stroke();
    };
    ctx.save();
    for (const a of visibleAlgos()) line((p) => p[a.key], colors[a.key], 2, 0.7, false, a.dashed);
    line((p) => [p.x, p.y], colors.fg, 1.5, 0.45, true, false);
    ctx.restore();
  }

  // ---- Drawing: error chart ----
  let range = null;
  function drawChart() {
    const c = ctxOf(chart);
    if (!c) return;
    const { ctx, w, h } = c;
    ctx.clearRect(0, 0, w, h);
    ctx.font = "15px Hyper, system-ui, sans-serif";
    const now = sim.t;
    const data = sim.history.filter((d) => d.t >= now - HISTORY_SECONDS);
    const left = 50, right = w - 10, top = 24, axisH = 24;
    const p = { y0: top, y1: h - axisH - 6 };
    const tx = (t) => right - ((now - t) / HISTORY_SECONDS) * (right - left);
    const algos = visibleAlgos();

    let hi = 0;
    for (const d of data) for (const a of algos) hi = Math.max(hi, d[a.key]);
    const target = Math.max(0.5, hi * 1.15);
    range = range === null ? target : range + (target - range) * 0.12;
    const r = { lo: 0, hi: range };

    // Panel and gridlines
    ctx.fillStyle = colors.panel;
    ctx.fillRect(left, p.y0, right - left, p.y1 - p.y0);
    ctx.strokeStyle = colors.line;
    ctx.lineWidth = 1;
    ctx.strokeRect(left + 0.5, p.y0 + 0.5, right - left - 1, p.y1 - p.y0 - 1);
    ctx.fillStyle = colors.subtle;
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (const v of niceTicks(r.lo, r.hi, 4)) {
      const y = mapY(v, p, r);
      if (y < p.y0 + 2 || y > p.y1) continue;
      ctx.beginPath();
      ctx.moveTo(left, y);
      ctx.lineTo(right, y);
      ctx.stroke();
      ctx.fillText(trimNum(v), left - 6, y);
    }
    // The "found it" threshold, so recoveries read at a glance.
    const yb = mapY(RECOVERED, p, r);
    ctx.save();
    ctx.setLineDash([6, 6]);
    ctx.strokeStyle = colors.lineStrong;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(left, yb);
    ctx.lineTo(right, yb);
    ctx.stroke();
    ctx.restore();
    ctx.textAlign = "right";
    ctx.textBaseline = "bottom";
    ctx.fillText(`${RECOVERED} m`, right - 4, yb - 2);
    ctx.textAlign = "left";
    ctx.fillText("m", left, p.y0 - 4);

    for (const a of algos) series(ctx, data, a.key, p, r, tx, colors[a.key], a.dashed);

    // Time axis
    ctx.fillStyle = colors.subtle;
    ctx.textBaseline = "top";
    for (let s = 0; s <= HISTORY_SECONDS; s += 5) {
      const x = right - (s / HISTORY_SECONDS) * (right - left);
      ctx.textAlign = s === 0 ? "right" : s === HISTORY_SECONDS ? "left" : "center";
      ctx.fillText(s === 0 ? "now" : `−${s} s`, x, p.y1 + 6);
    }
  }

  const mapY = (v, p, r) => p.y1 - ((v - r.lo) / (r.hi - r.lo)) * (p.y1 - p.y0);

  function series(ctx, data, key, p, r, tx, color, dashed) {
    if (data.length < 2) return;
    ctx.save();
    ctx.beginPath();
    ctx.rect(tx(data[0].t) - 2, p.y0, 1e5, p.y1 - p.y0);
    ctx.clip();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.5;
    ctx.setLineDash(dashed ? [6, 5] : []);
    ctx.beginPath();
    data.forEach((d, i) => {
      const x = tx(d.t), y = mapY(Math.min(d[key], r.hi * 1.05), p, r);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
    ctx.restore();
  }

  function niceTicks(lo, hi, n) {
    const raw = (hi - lo) / n;
    const mag = 10 ** Math.floor(Math.log10(raw));
    const stepSize = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= raw) ?? 10 * mag;
    const out = [];
    for (let v = Math.ceil(lo / stepSize) * stepSize; v <= hi + 1e-9; v += stepSize) out.push(Math.abs(v) < stepSize * 1e-6 ? 0 : v);
    return out;
  }
  const trimNum = (v) => String(Number(v.toPrecision(4)));

  // ---- Readouts ----
  /** Error about a second ago, from the history, to tell recovering from lost. */
  function errorAgo(key, seconds) {
    const t = sim.t - seconds;
    const hist = sim.history;
    for (let i = hist.length - 1; i >= 0; i--) if (hist[i].t <= t) return hist[i][key];
    return null;
  }

  function statusWord(key, e) {
    if (key === "pf" && sim.pf.inject > 0) return ["searching", "bad"];
    if (e.pos < RECOVERED) return ["tracking", "ok"];
    if (key === "dr") return ["drifting", "bad"];
    const before = errorAgo(key, 1);
    if (before !== null && before > e.pos * 1.5 && before - e.pos > 0.2) return ["recovering", ""];
    return e.pos < 1 ? ["off", "bad"] : ["lost", "bad"];
  }

  function updateReadouts() {
    const e = sim.errors();
    $("#loc-table").innerHTML = TABLE_ORDER.filter(on).map((k) => {
      const [word, cls] = statusWord(k, e[k]);
      return `<tr><th class="row" scope="row"><span class="loc-swatch loc-c-${k}"></span>${NAME[k]}</th>` +
        `<td>${fmt(e[k].pos, 3)} m</td><td>${fmt(e[k].head / DEG, 1)}°</td>` +
        `<td class="loc-word ${cls}">${word}</td></tr>`;
    }).join("") || `<tr><td class="loc-word" colspan="4">Turn on an algorithm above.</td></tr>`;

    const pf = sim.pf;
    const set = (id, html) => ($(id).innerHTML = html);
    set("#loc-m-neff", `${Math.round(pf.neff)} / ${pf.n}`);
    set("#loc-m-neff-sub", pf.inject > 0
      ? `injecting <span class="mono">${Math.round(pf.inject * 100)}%</span> random`
      : `resample below <span class="mono">${Math.round(pf.n / 2)}</span>`);
    set("#loc-m-vis", `${sim.z.length} / ${sim.inRange}`);
    set("#loc-m-vis-sub", `<span class="mono">${sim.blocked.length}</span> behind walls, range <span class="mono">${fmt(sim.params.maxRange, 1)} m</span>`);

    const badge = $("#loc-badge");
    badge.hidden = !(sim.t - sim.bumpT < 1.2);
  }

  /** Status line after a kidnap or reset: what each visible estimator does about it. */
  function trackStatus() {
    if (!event) return;
    const e = sim.errors();
    const since = sim.t - event.t0;
    // Count it only after a full second under the bound, so an estimate that
    // sweeps past the robot on its way somewhere else does not count.
    for (const k of TABLE_ORDER) {
      if (event.found[k] !== undefined) continue;
      if (e[k].pos >= RECOVERED) event.below[k] = undefined;
      else if (event.below[k] === undefined) event.below[k] = since;
      else if (since - event.below[k] >= 1) event.found[k] = event.below[k];
    }
    const off = (k) => `${fmt(e[k].pos, 1)} m off`;
    const kidnap = {
      pf: "searching. The measurements stopped matching, so it mixes random particles into the set.",
      grid: "searching. Its small uniform floor lets cells near the new spot win as soon as the scans favor them.",
      ekf: `${off("ekf")}. It keeps one Gaussian near the old spot, and its gate rejects the new measurements as outliers, so it will not recover on its own.`,
      ukf: `${off("ukf")}. Like the EKF it keeps one Gaussian and gates out the new measurements.`,
      dr: `${off("dr")}. It never uses the landmarks, so it cannot notice.`,
    };
    const reset = {
      pf: "spread over the whole map, narrowing down.",
      grid: "uniform over every free cell, narrowing down.",
      ekf: `started from a broad guess at the map center, now ${off("ekf")}. When the guess is far from the truth, the linearization can settle on a wrong pose.`,
      ukf: `started from the same broad guess, now ${off("ukf")}. Its sigma points span the whole guess, so it usually copes with a far-off start better than the EKF.`,
      dr: `restarted from the map center guess, ${off("dr")}. It never uses the landmarks, so it stays wrong.`,
    };
    const lines = event.kind === "kidnap" ? kidnap : reset;
    const parts = [event.kind === "kidnap" ? "Robot kidnapped. No estimator was told." : "Filters reset."];
    for (const k of TABLE_ORDER.filter(on)) {
      const found = event.found[k];
      parts.push(`${NAME[k]}: ${found !== undefined ? `within ${RECOVERED} m after ${fmt(found, 1)} s.` : lines[k]}`);
    }
    statusEl.textContent = parts.join(" ");
  }

  // ---- The math ----
  const math = createMathPanel(root, sim);

  // ---- Loop ----
  let last = performance.now();
  let readoutClock = 1;
  function frame(now) {
    raf = 0;
    if (!active) return;
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    if (!paused) sim.advance(dt);
    drawMap();
    drawChart();
    drawInset();
    readoutClock += dt;
    if (readoutClock > 0.1) {
      readoutClock = 0;
      updateReadouts();
      math.update();
      trackStatus();
      showHint();
    }
    raf = requestAnimationFrame(frame);
  }

  return {
    setActive(isOn) {
      active = isOn;
      if (isOn && !raf) {
        last = performance.now();
        readoutClock = 1;
        raf = requestAnimationFrame(frame);
      } else if (!isOn) {
        if (raf) cancelAnimationFrame(raf);
        raf = 0;
        releaseAll();
      }
    },
  };
}
