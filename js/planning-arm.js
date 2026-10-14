// Planning for a 2-link arm in configuration space: the room from above on
// the left, the arm's joint angles on the right, and A* between them. Drag an
// obstacle and the C-space picture and the path follow at once.

import { ARM, CS_N, CS_PRESETS, VIEW, buildCSpace, fk, indexOf, planCSpace, poseAlong } from "./cspace.js";
import { cssVar, segmented } from "./util.js";
import { inl, num, tex } from "./tex.js";

const DEG = 180 / Math.PI;
const SPEED = 1.8; // rad/s along the path in joint space

export function createArmPlanner(root) {
  const $ = (sel) => root.querySelector(sel);
  const work = $("#cs-work"), cmap = $("#cs-map");
  let preset = "posts";
  let obstacles, start, goal;
  let cs = null, plan = null;
  let probe = null;     // the pose under the pointer in the C-space picture
  let anim = null;      // { t0 } while the arm plays the path
  let active = false;
  let raf = 0;
  let colors = readColors();
  // The C-space picture is drawn once per change into a small canvas, one
  // pixel per cell, then scaled up.
  const cells = document.createElement("canvas");
  cells.width = cells.height = CS_N;
  const searched = document.createElement("canvas");
  searched.width = searched.height = CS_N;

  function readColors() {
    const c = (n) => cssVar(n);
    return {
      panel: c("--panel"), raised: c("--raised"), solid: c("--solid"), line: c("--line"), lineStrong: c("--line-strong"),
      fg: c("--fg"), muted: c("--fg-muted"), teal: c("--teal"), accent: c("--accent"), highlight: c("--highlight"),
      start: c("--plot-d"), goal: c("--plot-error"),
    };
  }
  window.addEventListener("rb:theme", () => {
    colors = readColors();
    paintCells();
    paintSearched();
    requestRender();
  });

  // The link in the page intro scrolls here. A plain #hash would switch tools.
  root.querySelector("[data-jump]")?.addEventListener("click", (e) => {
    e.preventDefault();
    $("#plan-arm").scrollIntoView({ behavior: "smooth", block: "start" });
  });

  segmented($("#cs-preset"), (k) => load(k));
  function load(k) {
    preset = k;
    const p = CS_PRESETS[k];
    obstacles = p.obstacles.map((o) => ({ ...o }));
    start = { ...p.start };
    goal = { ...p.goal };
    anim = null;
    rebuild();
  }

  /** New obstacles: redo the C-space grid, then the plan. */
  function rebuild() {
    cs = buildCSpace(ARM, obstacles);
    paintCells();
    replan();
  }
  function replan() {
    plan = planCSpace(cs, start, goal);
    paintSearched();
    anim = null;
    updateText();
    requestRender();
  }

  $("#cs-play").addEventListener("click", () => {
    if (!plan?.found) return;
    anim = { t0: performance.now() };
    requestRender();
  });

  // ---- Canvas sizing ----
  const sizes = new Map();
  const ro = new ResizeObserver((entries) => {
    for (const e of entries) {
      const c = e.target;
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      c.width = Math.round(c.clientWidth * dpr);
      c.height = Math.round(c.clientHeight * dpr);
      sizes.set(c, { w: c.clientWidth, dpr });
    }
    requestRender();
  });
  ro.observe(work);
  ro.observe(cmap);

  // ---- The room: drag obstacles ----
  const toWorld = (e) => {
    const r = work.getBoundingClientRect();
    return { x: ((e.clientX - r.left) / r.width) * 2 * VIEW - VIEW, y: VIEW - ((e.clientY - r.top) / r.height) * 2 * VIEW, px: (2 * VIEW) / r.width };
  };
  const inside = (o, p, slack) => (o.type === "circle"
    ? Math.hypot(p.x - o.x, p.y - o.y) <= o.r + slack
    : Math.abs(p.x - o.x) <= o.w / 2 + slack && Math.abs(p.y - o.y) <= o.h / 2 + slack);
  let grab = null;
  work.addEventListener("pointerdown", (e) => {
    const p = toWorld(e);
    // A finger needs a little room around a small obstacle.
    const o = [...obstacles].reverse().find((ob) => inside(ob, p, 10 * p.px));
    if (!o) return;
    grab = { o, dx: o.x - p.x, dy: o.y - p.y };
    work.setPointerCapture(e.pointerId);
    anim = null;
  });
  work.addEventListener("pointermove", (e) => {
    const p = toWorld(e);
    if (!grab) {
      work.style.cursor = obstacles.some((ob) => inside(ob, p, 10 * p.px)) ? "grab" : "";
      return;
    }
    const lim = VIEW - 0.1;
    grab.o.x = Math.max(-lim, Math.min(lim, p.x + grab.dx));
    grab.o.y = Math.max(-lim, Math.min(lim, p.y + grab.dy));
    rebuild();
  });
  const drop = () => (grab = null);
  work.addEventListener("pointerup", drop);
  work.addEventListener("pointercancel", drop);

  // ---- C-space: drag start and goal, point anywhere to see that pose ----
  const toQ = (e) => {
    const r = cmap.getBoundingClientRect();
    const u = Math.max(0, Math.min(0.9999, (e.clientX - r.left) / r.width));
    const v = Math.max(0, Math.min(0.9999, 1 - (e.clientY - r.top) / r.height));
    return { t1: -Math.PI + u * 2 * Math.PI, t2: -Math.PI + v * 2 * Math.PI, px: (2 * Math.PI) / r.width };
  };
  const wrapA = (a) => a - 2 * Math.PI * Math.floor((a + Math.PI) / (2 * Math.PI));
  const near = (q, m) => Math.hypot(wrapA(q.t1 - m.t1), wrapA(q.t2 - m.t2)) <= 14 * q.px;
  let moving = null;
  cmap.addEventListener("pointerdown", (e) => {
    const q = toQ(e);
    moving = near(q, start) ? "start" : near(q, goal) ? "goal" : "probe";
    cmap.setPointerCapture(e.pointerId);
    anim = null;
    pointerQ(q);
  });
  cmap.addEventListener("pointermove", (e) => {
    const q = toQ(e);
    if (moving) pointerQ(q);
    else {
      cmap.style.cursor = near(q, start) || near(q, goal) ? "grab" : "crosshair";
      if (e.pointerType === "mouse") {
        probe = q;
        requestRender();
      }
    }
  });
  const release = () => {
    moving = null;
    probe = null;
    requestRender();
  };
  cmap.addEventListener("pointerup", release);
  cmap.addEventListener("pointercancel", release);
  cmap.addEventListener("pointerleave", () => {
    if (!moving) release();
  });
  function pointerQ(q) {
    if (moving === "probe") {
      probe = q;
      requestRender();
      return;
    }
    const m = moving === "start" ? start : goal;
    m.t1 = q.t1;
    m.t2 = q.t2;
    probe = null;
    replan();
  }

  // ---- Drawing ----
  function paintCells() {
    const ctx = cells.getContext("2d");
    const img = ctx.createImageData(CS_N, CS_N);
    const rgb = (hex) => [1, 3, 5].map((i) => parseInt(hex.slice(i, i + 2), 16));
    const wall = rgb(colors.solid), free = rgb(colors.raised);
    for (let j = 0; j < CS_N; j++) {
      for (let i = 0; i < CS_N; i++) {
        const c = cs.cells[j * CS_N + i] ? wall : free;
        // Row 0 of the image is the top, which is the largest θ2.
        img.data.set([c[0], c[1], c[2], 255], ((CS_N - 1 - j) * CS_N + i) * 4);
      }
    }
    ctx.putImageData(img, 0, 0);
  }

  function paintSearched() {
    const ctx = searched.getContext("2d");
    const img = ctx.createImageData(CS_N, CS_N);
    const c = [1, 3, 5].map((i) => parseInt(colors.accent.slice(i, i + 2), 16));
    for (let k = 0; k < plan.closed.length; k++) {
      if (!plan.closed[k] || cs.cells[k]) continue;
      const i = k % CS_N, j = (k - i) / CS_N;
      img.data.set([c[0], c[1], c[2], 255], ((CS_N - 1 - j) * CS_N + i) * 4);
    }
    ctx.putImageData(img, 0, 0);
  }

  /** The pose to draw in the room right now. */
  function shownPose(now) {
    if (anim && plan?.found) {
      const total = plan.cost;
      const u = ((now - anim.t0) / 1000) * SPEED / total;
      if (u >= 1) {
        anim = null;
        return { ...goal, done: true };
      }
      return poseAlong(plan.path, u);
    }
    return probe ?? start;
  }

  function drawWork(pose) {
    const sz = sizes.get(work);
    if (!sz) return;
    const ctx = work.getContext("2d");
    const W = work.width, k = W / (2 * VIEW), dpr = sz.dpr;
    const X = (x) => (x + VIEW) * k, Y = (y) => (VIEW - y) * k;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = colors.panel;
    ctx.fillRect(0, 0, W, W);

    // Reach: the circle the tip can cover.
    ctx.strokeStyle = colors.line;
    ctx.lineWidth = dpr;
    ctx.setLineDash([4 * dpr, 4 * dpr]);
    ctx.beginPath();
    ctx.arc(X(0), Y(0), (ARM.l1 + ARM.l2) * k, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = colors.solid;
    for (const o of obstacles) {
      ctx.beginPath();
      if (o.type === "circle") ctx.arc(X(o.x), Y(o.y), o.r * k, 0, Math.PI * 2);
      else ctx.rect(X(o.x - o.w / 2), Y(o.y + o.h / 2), o.w * k, o.h * k);
      ctx.fill();
    }

    // Where the tip goes along the path.
    if (plan?.found) {
      ctx.strokeStyle = colors.accent;
      ctx.lineWidth = 2 * dpr;
      ctx.globalAlpha = 0.7;
      ctx.setLineDash([2 * dpr, 4 * dpr]);
      ctx.beginPath();
      for (let m = 0; m <= 200; m++) {
        const q = poseAlong(plan.path, m / 200);
        const { tip } = fk(ARM, q.t1, q.t2);
        if (m) ctx.lineTo(X(tip.x), Y(tip.y));
        else ctx.moveTo(X(tip.x), Y(tip.y));
      }
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.globalAlpha = 1;
    }

    const arm = (q, color, alpha, width) => {
      const { elbow, tip } = fk(ARM, q.t1, q.t2);
      ctx.globalAlpha = alpha;
      ctx.strokeStyle = color;
      ctx.lineCap = "round";
      ctx.lineWidth = width;
      ctx.beginPath();
      ctx.moveTo(X(0), Y(0));
      ctx.lineTo(X(elbow.x), Y(elbow.y));
      ctx.lineTo(X(tip.x), Y(tip.y));
      ctx.stroke();
      ctx.globalAlpha = 1;
      return { elbow, tip };
    };
    const thick = Math.max(2 * ARM.width * k, 5 * dpr);
    // Start and goal as faint outlines, marked like the grid's start and goal.
    const sTip = arm(start, colors.start, 0.35, thick).tip;
    const gTip = arm(goal, colors.goal, 0.35, thick).tip;
    marker(ctx, X(sTip.x), Y(sTip.y), 5 * dpr, "start");
    marker(ctx, X(gTip.x), Y(gTip.y), 5 * dpr, "goal");

    const hit = cs.cells[indexOf(pose.t2, CS_N) * CS_N + indexOf(pose.t1, CS_N)];
    const { elbow, tip } = arm(pose, hit ? colors.goal : colors.teal, 1, thick);
    ctx.fillStyle = colors.accent;
    for (const p of [{ x: 0, y: 0 }, elbow]) {
      ctx.beginPath();
      ctx.arc(X(p.x), Y(p.y), thick * 0.75, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.fillStyle = colors.highlight;
    ctx.beginPath();
    ctx.arc(X(tip.x), Y(tip.y), thick * 0.45, 0, Math.PI * 2);
    ctx.fill();
  }

  function marker(ctx, x, y, r, kind) {
    ctx.lineWidth = Math.max(1.5, r * 0.35);
    ctx.strokeStyle = colors.panel;
    ctx.fillStyle = kind === "start" ? colors.start : colors.goal;
    ctx.beginPath();
    if (kind === "start") ctx.arc(x, y, r, 0, Math.PI * 2);
    else {
      ctx.moveTo(x, y - r * 1.25);
      ctx.lineTo(x + r * 1.25, y);
      ctx.lineTo(x, y + r * 1.25);
      ctx.lineTo(x - r * 1.25, y);
      ctx.closePath();
    }
    ctx.fill();
    ctx.stroke();
  }

  function drawMap(pose) {
    const sz = sizes.get(cmap);
    if (!sz) return;
    const ctx = cmap.getContext("2d");
    const W = cmap.width, dpr = sz.dpr;
    const X = (t1) => ((wrapA(t1) + Math.PI) / (2 * Math.PI)) * W;
    const Y = (t2) => (1 - (wrapA(t2) + Math.PI) / (2 * Math.PI)) * W;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(cells, 0, 0, W, W);

    // Cells the search looked at, faintly. Drawn as one image so the
    // overlapping cell edges don't leave a pattern of seams.
    ctx.globalAlpha = 0.16;
    ctx.drawImage(searched, 0, 0, W, W);
    ctx.globalAlpha = 1;

    // Zero lines, so the picture has a center to read from.
    ctx.strokeStyle = colors.lineStrong;
    ctx.globalAlpha = 0.5;
    ctx.lineWidth = dpr;
    ctx.beginPath();
    ctx.moveTo(W / 2, 0);
    ctx.lineTo(W / 2, W);
    ctx.moveTo(0, W / 2);
    ctx.lineTo(W, W / 2);
    ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.fillStyle = colors.muted;
    ctx.font = `${11 * dpr}px ${getComputedStyle(cmap).fontFamily}`;
    ctx.fillText("θ₁ →", W - 34 * dpr, W / 2 - 5 * dpr);
    ctx.fillText("θ₂ ↑", W / 2 + 5 * dpr, 13 * dpr);

    // The path, broken wherever it wraps off one edge and back on the other.
    if (plan?.found) {
      ctx.strokeStyle = colors.accent;
      ctx.lineWidth = 3 * dpr;
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      ctx.beginPath();
      let prev = null;
      for (const q of plan.path) {
        const x = X(q.t1), y = Y(q.t2);
        if (!prev || Math.abs(x - prev.x) > W / 2 || Math.abs(y - prev.y) > W / 2) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
        prev = { x, y };
      }
      ctx.stroke();
    }

    marker(ctx, X(start.t1), Y(start.t2), 7 * dpr, "start");
    marker(ctx, X(goal.t1), Y(goal.t2), 7 * dpr, "goal");
    if (pose !== start) {
      ctx.fillStyle = colors.highlight;
      ctx.strokeStyle = colors.fg;
      ctx.lineWidth = 1.5 * dpr;
      ctx.beginPath();
      ctx.arc(X(pose.t1), Y(pose.t2), 5 * dpr, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
  }

  // ---- Readouts and live math ----
  function updateText() {
    const set = (id, text) => ($(id).textContent = text);
    set("#cs-m-blocked", `${num(cs.blockedShare * 100, 0)}%`);
    set("#cs-m-found", plan.found ? "yes" : "no");
    set("#cs-m-cost", plan.found ? `${num(plan.cost * DEG, 0)}°` : "n/a");
    set("#cs-m-expanded", String(plan.expanded));
    const status = $("#cs-status");
    status.className = plan.found ? "small" : "small is-bad";
    status.textContent = plan.blockedEnd === "start" ? "The start pose hits an obstacle. Drag the start, or the obstacle, somewhere free."
      : plan.blockedEnd === "goal" ? "The goal pose hits an obstacle. Drag the goal, or the obstacle, somewhere free."
      : !plan.found ? "No path. The shaded regions cut the start off from the goal, even going round the edges."
      : wrapsAround() ? "Found a path. It goes off one edge and comes back on the other: the arm turns the long way round, past ±180°."
      : "Found a path. Press Play to watch the arm follow it.";
    $("#cs-play").disabled = !plan.found;
    tex($("#cs-math-live"), plan.found
      ? `\\text{cost} = ${num(plan.cost)}\\,\\text{rad} = ${num(plan.cost * DEG, 0)}^\\circ`
      : "\\text{no path}");
  }
  const wrapsAround = () => plan.path.some((q) => Math.abs(q.t1) > Math.PI || Math.abs(q.t2) > Math.PI);

  tex($("#cs-math-q"), "q = (\\theta_1, \\theta_2),\\quad \\theta_i \\in [-\\pi, \\pi)");
  tex($("#cs-math-obs"), "C_\\text{obs} = \\{\\, q : A(q) \\cap O \\ne \\emptyset \\,\\}");
  tex($("#cs-math-free"), "C_\\text{free} = C \\setminus C_\\text{obs}");
  $("#cs-math-obs-note").innerHTML = `${inl("A(q)")} is the space the arm takes up in pose ${inl("q")}, and ${inl("O")} is the obstacles. A path for the arm is a line from ${inl("q_\\text{start}")} to ${inl("q_\\text{goal}")} that stays in ${inl("C_\\text{free}")}.`;
  tex($("#cs-math-fk"), "p_\\text{elbow} = \\ell_1 \\begin{bmatrix} \\cos\\theta_1 \\\\ \\sin\\theta_1 \\end{bmatrix}");
  tex($("#cs-math-fk2"), "p_\\text{tip} = p_\\text{elbow} + \\ell_2 \\begin{bmatrix} \\cos(\\theta_1 + \\theta_2) \\\\ \\sin(\\theta_1 + \\theta_2) \\end{bmatrix}");
  tex($("#cs-math-hit"), "\\text{hit} \\iff d(\\text{link}, O) \\le w + \\delta");
  tex($("#cs-math-pad"), "\\delta = 2(\\ell_1 + \\ell_2)\\,\\frac{\\pi}{n}");
  $("#cs-math-hit-note").innerHTML = `Each cell is checked once, at its center, with forward kinematics from Learn lesson 2. ${inl("w")} is half the link's thickness. ${inl("\\delta")} is the most any point on the arm can move while the joints stay within one cell, so padding by it means a path through free cells really is free, not just at the centers. Here ${inl(`\\ell_1 = ${num(ARM.l1, 1)}`)}, ${inl(`\\ell_2 = ${num(ARM.l2, 1)}`)} and ${inl(`n = ${CS_N}`)}, so each cell is ${inl("3^\\circ")} of each joint.`;
  tex($("#cs-math-search"), "\\text{cost} = \\sum_k \\sqrt{\\Delta\\theta_{1,k}^2 + \\Delta\\theta_{2,k}^2}");

  // ---- Loop ----
  function requestRender() {
    if (!raf && active) raf = requestAnimationFrame(frame);
  }
  function frame(now) {
    raf = 0;
    if (!active) return;
    const pose = shownPose(now);
    drawWork(pose);
    drawMap(pose);
    if (anim) requestRender();
  }

  load(preset);
  return {
    setActive(on) {
      active = on;
      if (on) requestRender();
    },
  };
}
