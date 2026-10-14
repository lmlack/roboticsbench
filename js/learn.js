// Learn: an eight-step class, from robot arm basics to planning and SLAM. Lessons 1–3 use a 2-link arm
// drawn flat on a canvas, where every quantity can be seen at once; 4–6 move
// to 3D for rotations, a 6-axis arm and numerical IK. Each lesson owns its
// controls, its drawing and its live math; this file only switches between
// them. The kinematics live in learn-math.js, where they are tested.

import * as THREE from "three";
import {
  ARM6,
  det3,
  fk2,
  fk6,
  ik2,
  ikStep,
  jacobian6,
  manipulability,
  matMul,
  matVec,
  norm,
  orthoError,
  reach6,
  rot2,
  rotX,
  rotY,
  rotZ,
  rotationAngleBetween,
  tf2,
  tfPos,
  tfRot,
} from "./learn-math.js";
import { matrixToQuat } from "./rotation-math.js";
import { LOOP, MGL, SWING_WEIGHTS, SlamLoop, SwingUp } from "./learn-sims.js";
import { axisTriad, createViewport, disposeObject } from "./three-common.js";
import { inl, mat, num, row, vec } from "./tex.js";
import { cssVar, el, segmented } from "./util.js";

const DEG = Math.PI / 180;
const L1 = 1;
const L2 = 0.7;
const STEP_KEY = "rb-learn-step";

export function start(root) {
  const $ = (sel) => root.querySelector(sel);
  const canvas = $("#learn-canvas");
  const wrap2d = $("#learn-2d");
  const wrap3d = $("#learn-3d");
  const controls = $("#learn-controls");
  const mathHost = $("#learn-math");
  const hint = $("#learn-hint");
  let active = true;

  /* ---------- shared state ---------- */

  const s2 = {
    base: { x: 0.5, y: 0.3, phi: 20 * DEG },
    pose: { t1: 30 * DEG, t2: 50 * DEG },   // lesson 1 keeps the arm still
    t1: 35 * DEG, t2: 60 * DEG,
    trace: false, trail: [],
    target: [1.15, 0.75], elbow: "up", ghost: true,
  };
  const s7 = { sim: new SwingUp(), iterations: [], trace: [], t0: 0, reveal: 1 };
  const s8 = { sim: new SlamLoop(), playing: true, speed: 2, showDR: true };
  const s3 = {
    ypr: [40 * DEG, 25 * DEG, 30 * DEG], other: false,
    q: [...ARM6.home], frames: true,
    target: [0.62, 0.2, 0.5], lambda: 0.08, solving: false, iters: 0, errors: [],
  };

  /* ---------- small UI builders ---------- */

  function slider({ label, min, max, step = 1, value, unit = "°", digits = 0, onInput }) {
    const out = el("span", { class: "val" });
    const input = el("input", { type: "range", min, max, step, value, "aria-label": label });
    const show = () => { out.textContent = `${Number(input.value).toFixed(digits)}${unit}`; };
    input.addEventListener("input", () => { show(); onInput(Number(input.value)); });
    show();
    const node = el("label", { class: "learn-slider" }, el("div", { class: "head" }, el("span", { text: label }), out), input);
    return { node, set(v) { input.value = v; show(); } };
  }
  const sliders = (...items) => el("div", { class: "learn-sliders" }, ...items.map((s) => s.node));
  function check(label, checked, onChange) {
    const input = el("input", { type: "checkbox" });
    input.checked = checked;
    input.addEventListener("change", () => onChange(input.checked));
    return el("label", { class: "check" }, input, label);
  }
  const button = (text, onclick, cls = "btn") => el("button", { type: "button", class: cls, text, onclick });
  function seg(label, options, value, onChange, { grid = false } = {}) {
    // grid: on a phone, lay the buttons out in even columns instead of a pill
    // that would wrap (see .seg-grid in style.css).
    const group = el("div", { class: grid ? "segmented seg-grid" : "segmented", role: "group", "aria-label": label },
      ...options.map(([v, text]) => el("button", { type: "button", "data-v": v, "aria-pressed": String(v === value), text })));
    segmented(group, onChange);
    return group;
  }

  /* ---------- 2D canvas ---------- */

  let size2 = null;
  new ResizeObserver(() => {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(canvas.clientWidth * dpr);
    canvas.height = Math.round(canvas.clientHeight * dpr);
    size2 = { w: canvas.clientWidth, h: canvas.clientHeight, dpr };
    if (lesson().dim === "2d") render();
  }).observe(canvas);

  function colors() {
    const c = (n) => cssVar(n);
    return {
      fg: c("--fg"), muted: c("--fg-muted"), subtle: c("--fg-subtle"), line: c("--line"), strong: c("--line-strong"),
      accent: c("--accent"), teal: c("--teal"), x: c("--axis-x"), y: c("--axis-y"), panel: c("--panel"),
      highlight: c("--highlight"), bad: c("--bad-fg"),
    };
  }

  /** World meters to canvas pixels for a lesson's view box, y up. */
  function view2(box) {
    const { w, h } = size2;
    const s = Math.min(w / (box.x1 - box.x0), h / (box.y1 - box.y0));
    const ox = (w - s * (box.x1 - box.x0)) / 2, oy = (h - s * (box.y1 - box.y0)) / 2;
    return {
      s,
      X: (x) => ox + (x - box.x0) * s,
      Y: (y) => oy + (box.y1 - y) * s,
      toWorld: (px, py) => [box.x0 + (px - ox) / s, box.y1 - (py - oy) / s],
    };
  }

  function begin2d(box, gridStep = 0.5, showLabel = true) {
    if (!size2?.w) return null;
    const ctx = canvas.getContext("2d");
    ctx.setTransform(size2.dpr, 0, 0, size2.dpr, 0, 0);
    ctx.clearRect(0, 0, size2.w, size2.h);
    const v = view2(box);
    const c = colors();
    // Half-meter grid, with a scale note so the numbers below mean something.
    ctx.strokeStyle = c.line;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil(box.x0 / gridStep) * gridStep; x <= box.x1; x += gridStep) { ctx.moveTo(v.X(x), 0); ctx.lineTo(v.X(x), size2.h); }
    for (let y = Math.ceil(box.y0 / gridStep) * gridStep; y <= box.y1; y += gridStep) { ctx.moveTo(0, v.Y(y)); ctx.lineTo(size2.w, v.Y(y)); }
    ctx.stroke();
    ctx.fillStyle = c.subtle;
    ctx.font = "13px Hyper, system-ui, sans-serif";
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    if (showLabel) ctx.fillText(`grid: ${gridStep} m`, 10, size2.h - 8);
    return { ctx, v, c };
  }

  function arrow(ctx, x0, y0, x1, y1, color, width = 2.5) {
    const a = Math.atan2(y1 - y0, x1 - x0);
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = width;
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(x1 - 7 * Math.cos(a), y1 - 7 * Math.sin(a));
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x1 - 11 * Math.cos(a - 0.4), y1 - 11 * Math.sin(a - 0.4));
    ctx.lineTo(x1 - 11 * Math.cos(a + 0.4), y1 - 11 * Math.sin(a + 0.4));
    ctx.closePath();
    ctx.fill();
  }

  /** An X (red) / Y (green) frame at a world point, turned by `angle`. */
  function triad2({ ctx, v, c }, x, y, angle, label, len = 0.35) {
    const px = v.X(x), py = v.Y(y);
    const ex = [Math.cos(angle), Math.sin(angle)], ey = [-Math.sin(angle), Math.cos(angle)];
    arrow(ctx, px, py, v.X(x + ex[0] * len), v.Y(y + ex[1] * len), c.x);
    arrow(ctx, px, py, v.X(x + ey[0] * len), v.Y(y + ey[1] * len), c.y);
    if (label) {
      ctx.fillStyle = c.fg;
      ctx.font = "600 15px Hyper, system-ui, sans-serif";
      ctx.textAlign = "right";
      ctx.textBaseline = "top";
      ctx.fillText(label, px - 6, py + 4);
    }
  }

  /** Draws the 2-link arm from a base pose; returns its points in world meters. */
  function arm2({ ctx, v, c }, base, t1, t2, { ghost = false } = {}) {
    const R = rot2(base.phi);
    const toW = (p) => { const r = matVec(R, p); return [r[0] + base.x, r[1] + base.y]; };
    const { elbow, tip, phi } = fk2(L1, L2, t1, t2);
    const pts = [[base.x, base.y], toW(elbow), toW(tip)];
    ctx.save();
    if (ghost) { ctx.globalAlpha = 0.35; ctx.setLineDash([8, 7]); }
    ctx.lineCap = "round";
    ctx.strokeStyle = c.muted;
    ctx.lineWidth = Math.max(6, v.s * 0.07);
    ctx.beginPath();
    ctx.moveTo(v.X(pts[0][0]), v.Y(pts[0][1]));
    for (const p of pts.slice(1)) ctx.lineTo(v.X(p[0]), v.Y(p[1]));
    ctx.stroke();
    ctx.setLineDash([]);
    for (const p of pts.slice(0, 2)) {
      ctx.beginPath();
      ctx.arc(v.X(p[0]), v.Y(p[1]), Math.max(6, v.s * 0.045), 0, Math.PI * 2);
      ctx.fillStyle = c.panel;
      ctx.fill();
      ctx.lineWidth = 2.5;
      ctx.strokeStyle = c.fg;
      ctx.stroke();
    }
    ctx.restore();
    return { base: pts[0], elbow: pts[1], tip: pts[2], phi: phi + base.phi, tipB: tip };
  }

  function basePlate({ ctx, v, c }, base) {
    ctx.save();
    ctx.translate(v.X(base.x), v.Y(base.y));
    ctx.rotate(-base.phi);
    const r = Math.max(14, v.s * 0.12);
    ctx.fillStyle = c.strong;
    ctx.globalAlpha = 0.35;
    ctx.fillRect(-r, -r * 0.55, r * 2, r * 1.1);
    ctx.restore();
  }

  function ring({ ctx, v, c }, x, y) {
    ctx.save();
    ctx.globalAlpha = 0.16;
    ctx.fillStyle = c.highlight;
    ctx.beginPath();
    ctx.arc(v.X(x), v.Y(y), (L1 + L2) * v.s, 0, Math.PI * 2);
    ctx.arc(v.X(x), v.Y(y), Math.abs(L1 - L2) * v.s, 0, Math.PI * 2, true);
    ctx.fill();
    ctx.restore();
  }

  function dot({ ctx, v }, x, y, color, r = 6) {
    ctx.beginPath();
    ctx.arc(v.X(x), v.Y(y), r, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
  }
  function dashed({ ctx, v }, a, b, color) {
    ctx.save();
    ctx.setLineDash([5, 5]);
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    ctx.moveTo(v.X(a[0]), v.Y(a[1]));
    ctx.lineTo(v.X(b[0]), v.Y(b[1]));
    ctx.stroke();
    ctx.restore();
  }
  function label({ ctx, v, c }, x, y, text, color = c.fg) {
    ctx.fillStyle = color;
    ctx.font = "600 15px Hyper, system-ui, sans-serif";
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    ctx.fillText(text, v.X(x) + 9, v.Y(y) - 6);
  }

  /* ---------- 3D viewport (built the first time a 3D lesson opens) ---------- */

  let vp = null;
  let scene3 = null;   // { root, world, turned, ghost, arm, target, line }

  function ensure3d() {
    if (vp) return;
    vp = createViewport(wrap3d, { distance: 2.6, target: [0, 0, 0.45] });
    build3d();
    window.addEventListener("rb:theme", () => { build3d(); render(); });
  }

  function build3d() {
    if (scene3) { vp.scene.remove(scene3.root); disposeObject(scene3.root); }
    const root = new THREE.Group();
    const color = (n) => new THREE.Color(cssVar(n));
    const material = (n, extra = {}) => new THREE.MeshStandardMaterial({ color: color(n), roughness: 0.55, metalness: 0.05, ...extra });

    const grid = new THREE.GridHelper(2, 8, color("--line-strong"), color("--line"));
    grid.rotation.x = Math.PI / 2;   // GridHelper lies in XZ; the scenes are Z-up
    const world = axisTriad(0.5, { thick: false, labels: true });

    // Lesson 4: a frame to turn, and the same angles in the other order.
    const turned = axisTriad(0.8, { thick: true, labels: false });
    const ghost = axisTriad(0.8, { thick: true, labels: false, opacity: 0.3 });
    const bigWorld = axisTriad(1, { thick: false, labels: true });

    // Lessons 5–6: the arm, as nested groups that mirror fk6 exactly:
    // each joint is offset in its parent's turned frame, then turns.
    const arm = new THREE.Group();
    const plate = new THREE.Mesh(new THREE.CylinderGeometry(0.13, 0.15, 0.04, 40), material("--line-strong"));
    plate.rotation.x = Math.PI / 2;
    plate.position.z = 0.02;
    arm.add(plate);
    const up = new THREE.Vector3(0, 1, 0);
    const spins = [];
    const jointFrames = [];
    let parent = arm;
    ARM6.joints.forEach((j, i) => {
      const holder = new THREE.Group();
      holder.position.set(...j.offset);
      parent.add(holder);
      const spin = new THREE.Group();
      holder.add(spin);
      const axis = new THREE.Vector3(...j.axis);
      const twist = j.axis[2] === 1;
      const body = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.09, 28), material(twist ? "--teal" : "--accent"));
      body.quaternion.setFromUnitVectors(up, axis);
      spin.add(body);
      const next = new THREE.Vector3(...(ARM6.joints[i + 1]?.offset ?? ARM6.tool));
      const link = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, next.length(), 20), material("--fg-muted"));
      link.position.copy(next.clone().multiplyScalar(0.5));
      link.quaternion.setFromUnitVectors(up, next.clone().normalize());
      spin.add(link);
      const frame = axisTriad(0.14, { thick: false });
      spin.add(frame);
      jointFrames.push(frame);
      spins.push({ spin, axis });
      parent = spin;
    });
    const toolHolder = new THREE.Group();
    toolHolder.position.set(...ARM6.tool);
    parent.add(toolHolder);
    toolHolder.add(axisTriad(0.18, { thick: true }));

    const target = new THREE.Mesh(new THREE.SphereGeometry(0.035, 24, 16), material("--highlight", { emissive: color("--highlight"), emissiveIntensity: 0.25 }));
    const line = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: color("--accent"), dashSize: 0.03, gapSize: 0.02 }));

    root.add(grid, world, bigWorld, turned, ghost, arm, target, line);
    vp.scene.add(root);
    scene3 = { root, grid, world, bigWorld, turned, ghost, arm, spins, jointFrames, target, line };
  }

  const m4 = (R) => new THREE.Matrix4().set(R[0][0], R[0][1], R[0][2], 0, R[1][0], R[1][1], R[1][2], 0, R[2][0], R[2][1], R[2][2], 0, 0, 0, 0, 1);

  function show3d(parts) {
    for (const k of ["grid", "world", "bigWorld", "turned", "ghost", "arm", "target", "line"]) scene3[k].visible = parts.includes(k);
  }
  function poseArm(q) {
    scene3.spins.forEach(({ spin, axis }, i) => spin.quaternion.setFromAxisAngle(axis, q[i]));
  }
  let cameraFor = null;
  function aimCamera(kind) {
    if (cameraFor === kind) return;
    cameraFor = kind;
    const [pos, target] = kind === "frame" ? [[2.1, 1.6, 1.3], [0, 0, 0]] : [[0.75, 1.75, 0.85], [0.25, 0, 0.4]];
    vp.camera.position.set(...pos);
    vp.controls.target.set(...target);
    vp.controls.update();
  }

  /* ---------- the lessons ---------- */

  const LESSONS = [
    {
      title: "Frames: where is anything?",
      part: "Part 1 · Where things are",
      short: "Frames",
      dim: "2d",
      box: { x0: -0.9, x1: 2.7, y0: -0.6, y1: 2.4 },
      hint: "Move the base with the sliders",
      body: () => `
        <p>Before a robot can move anything, it needs to know where things are. And "where" always depends on what you're measuring from.</p>
        <p>That reference is called a <b>frame</b>: a starting point plus a set of axes. The <b>world frame W</b> is fixed to the room. The robot's <b>base frame B</b> sits wherever the robot was bolted down, turned however it was installed.</p>
        <p>The tool tip <b>P</b> has an address in each one. Its base address ${inl("p_B")} stays the same when you move the base, because the arm moves with it. Its world address ${inl("p_W")} changes.</p>
        <p>To get from one to the other, turn by the base's angle, then add where the base is. A <b>transform</b> ${inl("T")} does both in a single matrix, using a trick called <b>homogeneous coordinates</b>: write the point with a 1 on the end, ${inl("(x, y, 1)")}, and put the shift in an extra column, ${inl("T = \\begin{bmatrix} R & t \\\\ 0 & 1 \\end{bmatrix}")}. A plain rotation matrix can only turn a point. With the extra 1, one multiplication turns it and shifts it.</p>
        <p>Two things make that worth it. Going through several frames is just multiplying their matrices, ${inl("T_{AC} = T_{AB}\\,T_{BC}")}, which is exactly how the arms in the next lessons are built. And going back is easy: ${inl("T^{-1} = \\begin{bmatrix} R^\\top & -R^\\top t \\\\ 0 & 1 \\end{bmatrix}")}. A direction, as opposed to a point, gets a 0 instead of the 1, so it turns but never shifts.</p>
        <div class="learn-try"><b>Try it</b><ul><li>Move the base with the sliders. Watch ${inl("p_W")} change while ${inl("p_B")} stays put.</li><li>Turn the base and see the whole arm turn with it.</li></ul></div>
        <p class="learn-up-next"><b>Up next:</b> an arm is just a chain of these frames, one per joint. Working out where that chain puts the tool is forward kinematics.</p>`,
      controls() {
        const b = s2.base;
        return [sliders(
          slider({ label: "Base x", min: 0, max: 1, step: 0.01, value: b.x, unit: " m", digits: 2, onInput: (v) => { b.x = v; render(); } }),
          slider({ label: "Base y", min: 0, max: 0.8, step: 0.01, value: b.y, unit: " m", digits: 2, onInput: (v) => { b.y = v; render(); } }),
          slider({ label: "Base angle φ", min: -45, max: 60, value: b.phi / DEG, onInput: (v) => { b.phi = v * DEG; render(); } }),
        )];
      },
      draw(g) {
        const b = s2.base;
        triad2(g, 0, 0, 0, "W", 0.45);
        basePlate(g, b);
        const a = arm2(g, b, s2.pose.t1, s2.pose.t2);
        triad2(g, b.x, b.y, b.phi, "B");
        dashed(g, [0, 0], a.tip, g.c.subtle);
        dashed(g, [b.x, b.y], a.tip, g.c.teal);
        dot(g, ...a.tip, g.c.accent, 7);
        label(g, ...a.tip, "P", g.c.accent);
        return a;
      },
      math(a) {
        const b = s2.base, c = Math.cos(b.phi), s = Math.sin(b.phi);
        const pB = a.tipB, pW = a.tip;
        return [
          `R(\\varphi) = \\begin{bmatrix} \\cos\\varphi & -\\sin\\varphi \\\\ \\sin\\varphi & \\cos\\varphi \\end{bmatrix} = ${mat([[c, -s], [s, c]], 3)}`,
          `p_B = ${vec(pB)}, \\quad t = ${vec([b.x, b.y])}`,
          `p_W = R\\,p_B + t = ${vec(pW)}`,
          { note: "The same thing as one matrix, with a 1 added under each point:" },
          `T_{WB} = ${mat([[c, -s, b.x], [s, c, b.y], [0, 0, 1]], 3)}, \\quad T_{WB} ${vec([...pB, 1])} = ${vec([...pW, 1])}`,
          { note: "And back again, with the inverse built from the transpose:" },
          `T_{WB}^{-1} = \\begin{bmatrix} R^\\top & -R^\\top t \\\\ 0 & 1 \\end{bmatrix} = ${mat([[c, s, -(c * b.x + s * b.y)], [-s, c, s * b.x - c * b.y], [0, 0, 1]], 3)}, \\quad T_{WB}^{-1} ${vec([...pW, 1])} = ${vec([...pB, 1])}`,
        ];
      },
    },
    {
      title: "Forward kinematics: angles in, position out",
      part: "Part 2 · A flat arm",
      short: "Forward kinematics",
      dim: "2d",
      box: { x0: -1.9, x1: 1.9, y0: -1.9, y1: 1.9 },
      hint: "Turn the joints with the sliders",
      body: () => `
        <p>Now let's put frames to work. An arm is a chain: base, joint, link, joint, link, tool. Each joint adds a frame.</p>
        <p><b>Forward kinematics</b> (FK) answers one question: if I know the joint angles, where's the tool? Turn by ${inl("\\theta_1")}, go ${inl("\\ell_1")} along the first link, turn by ${inl("\\theta_2")}, go ${inl("\\ell_2")}. Each step is a transform from lesson 1, and multiplying them in order gives the tool's <b>pose</b>: where it is and which way it points.</p>
        <p>There's always exactly one answer, and it's quick to work out. That's why FK is called the easy direction.</p>
        <p>The shaded ring is the <b>workspace</b>, every spot the tip can reach: as far as ${inl("\\ell_1 + \\ell_2")} with the arm straight, and as close as ${inl("|\\ell_1 - \\ell_2|")} with it folded back.</p>
        <div class="learn-try"><b>Try it</b><ul><li>Turn on <i>Trace the tip</i> and sweep joint 2. The tip draws a circle around the elbow.</li><li>Try to get the tip to the inner edge of the ring.</li></ul></div>
        <p class="learn-up-next"><b>Up next:</b> usually you know where you want the tool and need the angles. That's the other direction.</p>`,
      controls() {
        return [
          sliders(
            slider({ label: "Joint 1, θ₁", min: -180, max: 180, value: s2.t1 / DEG, onInput: (v) => { s2.t1 = v * DEG; render(); } }),
            slider({ label: "Joint 2, θ₂", min: -170, max: 170, value: s2.t2 / DEG, onInput: (v) => { s2.t2 = v * DEG; render(); } }),
          ),
          el("div", { class: "learn-row" }, check("Trace the tip", s2.trace, (on) => { s2.trace = on; s2.trail = []; render(); })),
        ];
      },
      draw(g) {
        ring(g, 0, 0);
        const a = arm2(g, { x: 0, y: 0, phi: 0 }, s2.t1, s2.t2);
        if (s2.trace) {
          const last = s2.trail[s2.trail.length - 1];
          if (!last || Math.hypot(last[0] - a.tip[0], last[1] - a.tip[1]) > 0.005) s2.trail.push(a.tip);
          g.ctx.strokeStyle = g.c.accent;
          g.ctx.lineWidth = 2;
          g.ctx.beginPath();
          s2.trail.forEach((p, i) => (i ? g.ctx.lineTo : g.ctx.moveTo).call(g.ctx, g.v.X(p[0]), g.v.Y(p[1])));
          g.ctx.stroke();
        }
        triad2(g, 0, 0, 0, "B");
        triad2(g, ...a.tip, a.phi, "T", 0.28);
        return a;
      },
      math(a) {
        const T = matMul(matMul(matMul(tf2(s2.t1, 0, 0), tf2(0, L1, 0)), tf2(s2.t2, 0, 0)), tf2(0, L2, 0));
        return [
          `x = \\ell_1\\cos\\theta_1 + \\ell_2\\cos(\\theta_1+\\theta_2) = ${num(a.tip[0], 3)}\\,\\text{m}`,
          `y = \\ell_1\\sin\\theta_1 + \\ell_2\\sin(\\theta_1+\\theta_2) = ${num(a.tip[1], 3)}\\,\\text{m}`,
          `\\varphi = \\theta_1 + \\theta_2 = ${num(a.phi / DEG, 1)}^\\circ`,
          { note: `As a chain of transforms, with ${inl(`\\ell_1 = ${L1}`)} m and ${inl(`\\ell_2 = ${L2}`)} m. The last column is the tip:` },
          `T_{BT} = \\text{Rot}(\\theta_1)\\,\\text{Trans}(\\ell_1)\\,\\text{Rot}(\\theta_2)\\,\\text{Trans}(\\ell_2) = ${mat(T, 3)}`,
        ];
      },
    },
    {
      title: "Inverse kinematics: position in, angles out",
      part: "Part 2 · A flat arm",
      short: "Inverse kinematics",
      dim: "2d",
      box: { x0: -1.9, x1: 1.9, y0: -1.9, y1: 1.9 },
      hint: "Drag anywhere to move the target",
      drag: true,
      body: () => `
        <p>Last lesson went from angles to a position. Most of the time you need the reverse: here's where the tool should go, so what angles get it there? That's <b>inverse kinematics</b> (IK).</p>
        <p>It's harder because the answer isn't always one thing. Usually there are <b>two</b>, elbow up and elbow down. The faded arm is the one you didn't pick. Outside the ring there are <b>none</b>, and the best the arm can do is point at the target.</p>
        <p>For two links there's a neat shortcut. Base, elbow and tip make a triangle, so the law of cosines gives the elbow angle, and then ${inl("\\theta_1")} just aims the arm.</p>
        <div class="learn-try"><b>Try it</b><ul><li>Drag the target around.</li><li>Flip between elbow up and elbow down: both reach the same spot.</li><li>Drag the target outside the ring and see what the arm does.</li></ul></div>
        <p class="learn-up-next"><b>Up next:</b> everything so far has been flat. Real arms work in 3D, and that starts with turning in 3D.</p>`,
      controls() {
        return [el("div", { class: "learn-row" },
          seg("Elbow", [["up", "Elbow up"], ["down", "Elbow down"]], s2.elbow, (v) => { s2.elbow = v; render(); }),
          check("Show the other answer", s2.ghost, (on) => { s2.ghost = on; render(); }))];
      },
      draw(g) {
        ring(g, 0, 0);
        const [x, y] = s2.target;
        const sol = ik2(L1, L2, x, y);
        const pick = sol[s2.elbow], other = sol[s2.elbow === "up" ? "down" : "up"];
        if (s2.ghost && sol.reachable && Math.abs(pick.t2) > 1e-3) arm2(g, { x: 0, y: 0, phi: 0 }, other.t1, other.t2, { ghost: true });
        const a = arm2(g, { x: 0, y: 0, phi: 0 }, pick.t1, pick.t2);
        triad2(g, 0, 0, 0, "B");
        if (!sol.reachable) dashed(g, a.tip, s2.target, g.c.bad);
        const px = g.v.X(x), py = g.v.Y(y);
        g.ctx.strokeStyle = sol.reachable ? g.c.accent : g.c.bad;
        g.ctx.lineWidth = 2.5;
        g.ctx.beginPath();
        g.ctx.arc(px, py, 11, 0, Math.PI * 2);
        g.ctx.moveTo(px - 17, py); g.ctx.lineTo(px + 17, py);
        g.ctx.moveTo(px, py - 17); g.ctx.lineTo(px, py + 17);
        g.ctx.stroke();
        hint.textContent = sol.reachable ? this.hint : "Out of reach: the arm can only point at it";
        return { sol, pick };
      },
      math({ sol, pick }) {
        const [x, y] = s2.target;
        const sign = s2.elbow === "up" ? "-" : "+";
        return [
          `r^2 = x^2 + y^2 = ${num(x * x + y * y, 3)}\\,\\text{m}^2`,
          `\\cos\\theta_2 = \\frac{r^2 - \\ell_1^2 - \\ell_2^2}{2\\,\\ell_1 \\ell_2} = ${num(sol.cosT2, 3)}`,
          sol.reachable
            ? `\\theta_2 = ${sign}\\arccos(${num(sol.cosT2, 3)}) = ${num(pick.t2 / DEG, 1)}^\\circ`
            : `|\\cos\\theta_2| > 1 \\Rightarrow \\text{no solution}`,
          `\\theta_1 = \\operatorname{atan2}(y, x) - \\operatorname{atan2}(\\ell_2 \\sin\\theta_2,\\ \\ell_1 + \\ell_2\\cos\\theta_2) = ${num(pick.t1 / DEG, 1)}^\\circ`,
          { note: sol.reachable
            ? "Two signs for θ₂, two elbows. Both put the tip exactly on the target."
            : "A cosine cannot pass 1, so no triangle closes: the target is out of reach." },
        ];
      },
    },
    {
      title: "Rotations in 3D, and SO(3)",
      part: "Part 3 · A real arm in 3D",
      short: "Rotations, SO(3)",
      dim: "3d",
      body: () => `
        <p>Moving to 3D, positions are easy: just add a third number. Turning is the tricky part. In 2D it was one angle. In 3D it takes three.</p>
        <p>Here they're yaw, pitch and roll: turn about Z, then about the new Y, then about the newer X. The thick axes are the turned frame, and the thin ones are the world.</p>
        <p>Write the turned axes down as the columns of a 3×3 matrix and you've got the <b>rotation matrix</b> ${inl("R")}. A real rotation always has columns of length 1 at right angles to each other, ${inl("R^\\top R = I")}, and ${inl("\\det R = +1")}, so it never mirrors anything. The set of all of them is called <b>SO(3)</b>, the <i>special orthogonal group</i> in 3D: orthogonal for ${inl("R^\\top R = I")}, special for the +1.</p>
        <p><b>Order matters.</b> The same three angles in a different order give a different result.</p>
        <div class="learn-try"><b>Try it</b><ul><li>Set all three sliders to something, then tick <i>Show the other order</i> and see how far apart the two frames end up.</li><li>Open it in the rotation converter, which translates between quaternions, Euler angles and matrices.</li></ul></div>
        <p class="learn-up-next"><b>Up next:</b> now we can describe how each joint of a 3D arm is turned. Put six of them together and you've got a real arm.</p>`,
      controls() {
        const set = (i) => (v) => { s3.ypr[i] = v * DEG; render(); };
        return [
          sliders(
            slider({ label: "Yaw ψ, about Z", min: -180, max: 180, value: s3.ypr[0] / DEG, onInput: set(0) }),
            slider({ label: "Pitch θ, about Y", min: -90, max: 90, value: s3.ypr[1] / DEG, onInput: set(1) }),
            slider({ label: "Roll φ, about X", min: -180, max: 180, value: s3.ypr[2] / DEG, onInput: set(2) }),
          ),
          el("div", { class: "learn-row" },
            check("Show the other order", s3.other, (on) => { s3.other = on; render(); }),
            button("Open in the rotation converter", () => openRotation(yprMatrix()))),
        ];
      },
      draw() {
        const R = yprMatrix();
        aimCamera("frame");
        show3d(s3.other ? ["bigWorld", "turned", "ghost"] : ["bigWorld", "turned"]);
        scene3.turned.quaternion.setFromRotationMatrix(m4(R));
        const Ro = otherOrder();
        scene3.ghost.quaternion.setFromRotationMatrix(m4(Ro));
        return { R, Ro };
      },
      math({ R, Ro }) {
        const rows = [
          `R = R_z(\\psi)\\,R_y(\\theta)\\,R_x(\\varphi) = ${mat(R, 3)}`,
          { note: "Column 1 is where X ends up, column 2 is Y, column 3 is Z." },
          `\\max\\,|R^\\top R - I| = ${num(orthoError(R), 2)}, \\quad \\det R = ${num(det3(R), 3)}`,
        ];
        if (s3.other) {
          rows.push(
            `R' = R_x(\\varphi)\\,R_y(\\theta)\\,R_z(\\psi) = ${mat(Ro, 3)}`,
            { note: `Same three angles, other order: the two frames differ by ${num(rotationAngleBetween(R, Ro) / DEG, 1)}°.` },
          );
        }
        return rows;
      },
    },
    {
      title: "Forward kinematics in 3D",
      part: "Part 3 · A real arm in 3D",
      short: "FK in 3D",
      dim: "3d",
      body: () => `
        <p>This is lesson 2's forward kinematics with lesson 4's 3D rotations. An industrial arm has <b>six joints</b>.</p>
        <p>Each joint adds a transform: go along its link, then turn about its axis. Multiply all six and you get where the tool is and which way it points, in one 4×4 matrix. It's the same homogeneous matrix as lesson 1, one size up: ${inl("R")} is 3×3, ${inl("p")} has three numbers, and the bottom row is ${inl("[0\\ 0\\ 0\\ 1]")}.</p>
        <p>Coral joints <b>bend</b> (about their own Y) and teal joints <b>twist</b> (about their own Z). The small axes show each joint's own frame, and the big ones at the end are the tool frame.</p>
        <p>The URDF viewer tab runs this same calculation on any robot you load.</p>
        <div class="learn-try"><b>Try it</b><ul><li>Move one joint at a time and watch which frames move: every frame after that joint, and none before it.</li><li>Press <i>Home pose</i>, then twist the last joint. The tool spins but doesn't move.</li></ul></div>
        <p class="learn-up-next"><b>Up next:</b> like before, the useful question is the reverse: how do you make this arm reach a point?</p>`,
      controls() {
        const items = ARM6.joints.map((j, i) => slider({
          label: `${i + 1}. ${j.name}`, min: Math.round(-j.limit / DEG), max: Math.round(j.limit / DEG), value: s3.q[i] / DEG,
          onInput: (v) => { s3.q[i] = v * DEG; render(); },
        }));
        armSliders = items;
        return [
          sliders(...items),
          el("div", { class: "learn-row" },
            check("Joint frames", s3.frames, (on) => { s3.frames = on; render(); }),
            button("Home pose", () => { s3.q = [...ARM6.home]; syncArmSliders(); render(); }),
            button("Open tool rotation in the converter", () => openRotation(tfRot(fk6(s3.q).tool)))),
        ];
      },
      draw() {
        aimCamera("arm");
        show3d(["grid", "world", "arm"]);
        poseArm(s3.q);
        for (const f of scene3.jointFrames) f.visible = s3.frames;
        return fk6(s3.q);
      },
      math({ tool }) {
        const R = tfRot(tool);
        return [
          `T_{WT} = T_1(q_1)\\,T_2(q_2)\\cdots T_6(q_6)\\,T_{\\text{tool}}`,
          { note: `Each ${inl("T_i = \\text{Trans}(\\ell_i)\\,\\text{Rot}(a_i, q_i)")}: shift along link ${inl("i")}, then turn about its axis ${inl("a_i")}.` },
          `T_{WT} = ${mat(tool, 3)} = \\begin{bmatrix} R & p \\\\ 0 & 1 \\end{bmatrix}`,
          `p = ${vec(tfPos(tool))}\\,\\text{m}, \\quad \\det R = ${num(det3(R), 3)}`,
        ];
      },
    },
    {
      title: "Inverse kinematics in 3D, and the Jacobian",
      part: "Part 3 · A real arm in 3D",
      short: "IK and the Jacobian",
      dim: "3d",
      body: () => `
        <p>Lesson 3 solved a two-link arm with one triangle. Six joints have no tidy formula like that, so arms are usually solved step by step instead.</p>
        <p>Start from the joints you have, nudge them so the tip moves a little toward the target, and repeat until it gets there. The yellow ball is the <b>target</b>, the point the tip should reach.</p>
        <p>The <b>Jacobian</b> ${inl("J")} is what tells you how to nudge. It says how fast the tip moves for each joint's speed. Turn it around and "move the tip this way" becomes "turn the joints this much". The damping ${inl("\\lambda")} keeps those nudges sensible when ${inl("J")} can't be turned around.</p>
        <p>That happens at a <b>singularity</b>: a pose where the arm loses a direction it can move in, like when it's stretched out straight or two wrist axes line up. <b>Manipulability</b> ${inl("w")} says how far you are from one, and it drops to zero there.</p>
        <div class="learn-try"><b>Try it</b><ul><li>Move the target with the sliders and watch the arm work its way there.</li><li>Press <i>Straight up (singular)</i> and <i>Stretch out (singular)</i> and watch ${inl("w")} go to zero.</li><li>Press <i>Out of reach</i>: the arm gets as close as it can.</li></ul></div>
        <p class="learn-up-next"><b>Up next:</b> reaching a point is one thing. Some moves need a whole plan, not just a nudge in the right direction.</p>`,
      controls() {
        const set = (i) => (v) => { s3.target[i] = v; s3.singular = null; startSolve(); };
        const items = [
          slider({ label: "Target x", min: -0.9, max: 0.9, step: 0.01, value: s3.target[0], unit: " m", digits: 2, onInput: set(0) }),
          slider({ label: "Target y", min: -0.9, max: 0.9, step: 0.01, value: s3.target[1], unit: " m", digits: 2, onInput: set(1) }),
          slider({ label: "Target z", min: 0, max: 1.3, step: 0.01, value: s3.target[2], unit: " m", digits: 2, onInput: set(2) }),
          slider({ label: "Damping λ", min: 0.01, max: 0.3, step: 0.01, value: s3.lambda, unit: "", digits: 2, onInput: (v) => { s3.lambda = v; startSolve(); } }),
        ];
        targetSliders = items;
        return [
          sliders(...items),
          el("div", { class: "learn-row" },
            button("Random reachable target", () => {
              const q = ARM6.home.map((v) => v + (Math.random() - 0.5) * 2);
              s3.q = [...ARM6.home];
              setTarget(tfPos(fk6(q).tool));
            }),
            button("Out of reach", () => setTarget([0.85, -0.6, 1.2])),
            // Two singular poses that stay put, so you can look at them. Moving
            // the target afterwards shows the solver finding its way out.
            button("Straight up (singular)", () => {
              s3.q = [0, 0, 0, 0, 0, 0];
              setTarget(tfPos(fk6(s3.q).tool), { solve: false, singular: "up" });
            }),
            button("Stretch out (singular)", () => {
              s3.q = [...ARM6.home];
              setTarget([reach6(), 0, ARM6.joints[1].offset[2]], { singular: "stretch" });
            })),
        ];
      },
      draw() {
        aimCamera("arm");
        show3d(["grid", "world", "arm", "target", "line"]);
        for (const f of scene3.jointFrames) f.visible = false;
        poseArm(s3.q);
        const tool = fk6(s3.q).tool;
        const p = tfPos(tool);
        scene3.target.position.set(...s3.target);
        scene3.line.geometry.setFromPoints([new THREE.Vector3(...p), new THREE.Vector3(...s3.target)]);
        scene3.line.computeLineDistances();
        const J = jacobian6(s3.q);
        return { p, J };
      },
      math({ p, J }) {
        const e = s3.target.map((v, i) => v - p[i]);
        const err = norm(e);
        const w = manipulability(J);
        const far = Math.hypot(s3.target[0], s3.target[1], s3.target[2] - 0.3) > reach6();
        const singular = w < 0.005;
        let status;
        if (s3.solving) status = `Solving: step ${s3.iters}, still ${num(err * 1000, 1)} mm off.`;
        else if (singular && s3.singular === "up") status = "At a singularity. Straight up, joints 1, 4 and 6 all turn about the same vertical line, which runs through the tip, so turning them does not move the tip at all, and the three bending joints can only push it along x. Sideways in y is impossible from here, and w is zero. Move the target to watch the solver work its way out.";
        else if (singular && s3.singular === "stretch") status = `At a singularity. Fully stretched, no combination of joints can move the tip any further out, so w is close to zero. Watch the step count: the last few millimetres took many small steps, because the damping holds each step back where J can no longer be inverted.`;
        else if (err < 1e-3) status = `Reached in ${s3.iters} step${s3.iters === 1 ? "" : "s"}.`;
        else if (far) status = `Out of reach: ${num(reach6(), 2)} m is the most the arm can stretch from its shoulder, so it stops as close as it can get.`;
        else status = `Stopped ${num(err * 1000, 1)} mm away after ${s3.iters} steps. A different start or less damping may get there.`;
        // The two singular presets explain themselves; anything else that ends
        // near a singularity gets a short note on the end.
        if (singular && !s3.singular && !s3.solving) status += " The arm is at or near a singularity, so some directions are barely reachable.";
        return [
          `e = p_{\\text{target}} - p_{\\text{tip}} = ${vec(e)}, \\quad \\|e\\| ${err < 1e-4 ? "< 0.1" : `= ${num(err * 1000, 1)}`}\\,\\text{mm}`,
          `\\Delta q = J_v^\\top \\left(J_v J_v^\\top + \\lambda^2 I\\right)^{-1} e, \\quad \\lambda = ${num(s3.lambda, 2)}`,
          { note: "With six joints and only three numbers to match, there are endless answers. The solver uses the spare freedom to drift back toward the home pose, so the arm doesn't end up in a knot." },
          { note: `${inl("J_v")}, the top three rows of ${inl("J")}: one column per joint, how the tip moves (m per radian) when only that joint turns.` },
          `J_v = ${mat(J.slice(0, 3), 2)}`,
          `w = \\sqrt{\\det\\left(J J^\\top\\right)} = ${num(w, 4)}${singular ? "\\quad \\text{(singular)}" : ""}`,
          { note: status, cls: "verdict" },
        ];
      },
    },
    {
      title: "Planning ahead: iLQR and MPC",
      part: "Part 4 · Beyond the arm",
      short: "iLQR and MPC",
      dim: "2d",
      box: { x0: -1.6, x1: 4.0, y0: -1.25, y1: 2.95 },
      gridLabel: false,
      hint: "",
      body: () => `
        <p>So far the question has been where the arm should be. Now it's how to get there, when you can't just push straight toward the goal.</p>
        <p>This arm's motor is too weak to lift it straight up. Holding it out level takes ${inl(`mgL = ${MGL}`)} N·m, and the motor only has a few. Push toward the top and it'll never get there. It has to swing back and forth to build up speed first, so it needs a plan.</p>
        <p><b>iLQR</b>, the <i>iterative linear quadratic regulator</i>, makes that plan. It starts from a guess and keeps improving it. Each pass treats the arm as simple (linear) around the current plan, finds the best fix for that simpler problem, and keeps the fix only if it actually lowers the cost. That inner step is the <b>LQR</b> part: a linear model with a quadratic cost has an exact best answer. The chart shows every pass, faint to bold.</p>
        <p><b>MPC</b>, <i>model predictive control</i>, uses the same solver in a loop. Every 100 ms it re-plans from wherever the arm actually is, follows the first bit, then plans again. That's what makes it hold up when something unexpected happens.</p>
        <div class="learn-try"><b>Try it</b><ul><li>Watch the plan come together, pass by pass, then let it run.</li><li>Press <i>Kick</i> partway through. Following the plan blindly, it misses.</li><li>Switch to <i>Re-plan (MPC)</i> and kick it again. It recovers.</li><li>Lower the motor strength and count the swings it needs.</li></ul></div>
        <p class="learn-up-next"><b>Up next:</b> everything so far assumed the robot knows where it is. Last step: what if it doesn't even have a map?</p>`,
      controls() {
        return [
          sliders(
            slider({ label: "Motor strength", min: 3, max: 8, step: 0.5, value: s7.sim.limit, unit: " N·m", digits: 1, onInput: (v) => { s7.sim.limit = v; restart7(); } }),
          ),
          el("div", { class: "learn-row" },
            seg("How to run it", [["follow", "Follow the plan"], ["mpc", "Re-plan (MPC)"]], s7.sim.mode, (v) => { s7.sim.mode = v; restart7(); }, { grid: true })),
          el("div", { class: "learn-row" },
            button("Kick", () => { s7.sim.kick(s7.sim.x[1] >= 0 ? 2.2 : -2.2); if (s7.sim.mode === "mpc") markChart7(); }, "btn solid"),
            button("Start over", () => restart7())),
        ];
      },
      tick(dt) {
        if (s7.reveal < 1) { s7.reveal = Math.min(1, s7.reveal + dt / 1.4); return; }
        s7.sim.advance(dt);
        const t = s7.sim.t - s7.t0;
        s7.trace.push([t, s7.sim.x[0]]);
        if (s7.trace.length > 2000) s7.trace.shift();
      },
      draw(g) {
        const { ctx, v, c } = g;
        const sim = s7.sim;
        const pivot = [0, 0.9];
        // The arm, θ measured from hanging straight down.
        const th = sim.x[0];
        const bob = [pivot[0] + Math.sin(th), pivot[1] - Math.cos(th)];
        ctx.save();
        ctx.setLineDash([5, 6]);
        ctx.strokeStyle = c.line;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(v.X(pivot[0]), v.Y(pivot[1]), v.s, 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
        dashed(g, pivot, [pivot[0], pivot[1] + 1], c.teal);
        label(g, pivot[0] + 0.02, pivot[1] + 1.02, "goal", c.teal);
        ctx.lineCap = "round";
        ctx.strokeStyle = c.muted;
        ctx.lineWidth = Math.max(6, v.s * 0.07);
        ctx.beginPath();
        ctx.moveTo(v.X(pivot[0]), v.Y(pivot[1]));
        ctx.lineTo(v.X(bob[0]), v.Y(bob[1]));
        ctx.stroke();
        dot(g, ...bob, c.highlight, Math.max(8, v.s * 0.09));
        dot(g, ...pivot, c.fg, 6);
        // Motor torque as an arc at the pivot, longer for more.
        const frac = sim.u / sim.limit;
        if (Math.abs(frac) > 0.02) {
          ctx.strokeStyle = c.accent;
          ctx.lineWidth = 4;
          ctx.beginPath();
          const r = v.s * 0.28, a0 = -Math.PI / 2;
          ctx.arc(v.X(pivot[0]), v.Y(pivot[1]), r, a0, a0 - frac * Math.PI * 0.9, frac > 0);
          ctx.stroke();
        }
        // Motor readout in the bottom-left corner, clear of the chart.
        ctx.fillStyle = c.muted;
        ctx.font = "13px Hyper, system-ui, sans-serif";
        ctx.textAlign = "left";
        ctx.textBaseline = "bottom";
        ctx.fillText(`motor ${sim.u < 0 ? "−" : ""}${Math.abs(sim.u).toFixed(1)} of ±${sim.limit} N·m`, 10, size2.h - 8);

        // Chart of θ against time: every iLQR pass, the arm's actual path, and MPC's forecast.
        const box = { x0: 1.35, x1: 3.85, y0: -1.0, y1: 2.6 };
        const T = sim.horizon + 1.5;
        const cx = (t) => box.x0 + (t / T) * (box.x1 - box.x0);
        const lo = -1.25 * Math.PI, hi = 1.25 * Math.PI;
        const cy = (a) => box.y0 + ((Math.max(lo, Math.min(hi, a)) - lo) / (hi - lo)) * (box.y1 - box.y0);
        ctx.strokeStyle = c.strong;
        ctx.lineWidth = 1;
        ctx.strokeRect(v.X(box.x0), v.Y(box.y1), (box.x1 - box.x0) * v.s, (box.y1 - box.y0) * v.s);
        for (const [a, txt] of [[Math.PI, "up"], [0, "down"], [-Math.PI, "up"]]) {
          dashed(g, [box.x0, cy(a)], [box.x1, cy(a)], a === 0 ? c.line : c.teal);
          ctx.fillStyle = c.subtle;
          ctx.font = "12px Hyper, system-ui, sans-serif";
          ctx.textAlign = "right";
          ctx.textBaseline = "middle";
          ctx.fillText(txt, v.X(box.x0) - 5, v.Y(cy(a)));
        }
        ctx.textAlign = "left";
        ctx.textBaseline = "bottom";
        ctx.fillText("angle over time", v.X(box.x0), v.Y(box.y1) - 4);
        const curve = (pts, color, width, alpha = 1, dash = null) => {
          if (pts.length < 2) return;
          ctx.save();
          ctx.globalAlpha = alpha;
          ctx.strokeStyle = color;
          ctx.lineWidth = width;
          if (dash) ctx.setLineDash(dash);
          ctx.beginPath();
          pts.forEach(([t, a], i) => (i ? ctx.lineTo : ctx.moveTo).call(ctx, v.X(cx(t)), v.Y(cy(a))));
          ctx.stroke();
          ctx.restore();
        };
        if (sim.mode === "follow" && s7.iterations.length) {
          const shown = Math.max(1, Math.ceil(s7.reveal * s7.iterations.length));
          s7.iterations.slice(0, shown).forEach((X, i) => {
            const last = i === shown - 1;
            curve(X.map((x, k) => [k * sim.dt, x[0]]), c.accent, last ? 3 : 1.2, last ? 1 : 0.15 + 0.5 * (i / s7.iterations.length));
          });
        }
        const fc = sim.forecast();
        if (fc) {
          const now = sim.t - s7.t0;
          curve(fc.map((x, k) => [now + k * 0.05, x[0]]), c.accent, 2.5, 0.9, [6, 5]);
        }
        curve(s7.trace, c.fg, 2);
        return sim;
      },
      math(sim) {
        const lines = [
          `\\tau_{\\max} = ${num(sim.limit, 1)}\\,\\text{N·m} < mgL = ${MGL}\\,\\text{N·m}`,
          { note: "So it can't lift the arm straight up. It has to swing back and forth to build up speed first." },
          `J = \\sum_k \\big( e_k^\\top Q\\, e_k + R\\,\\tau_k^2 \\big) + e_N^\\top Q_f\\, e_N`,
          `Q = \\operatorname{diag}(${SWING_WEIGHTS.Q.join(", ")}),\\quad Q_f = \\operatorname{diag}(${SWING_WEIGHTS.Qf.join(", ")}),\\quad R = ${SWING_WEIGHTS.R}`,
          { note: `It charges a little for being away from the top along the way, and a lot for not being there at the end. If it charged a lot the whole way, swinging away first would look too expensive, and the plan would get stuck partway up.` },
        ];
        if (sim.mode === "follow" && sim.plan) {
          const c = sim.plan.costs;
          lines.push(`J:\\ ${num(c[0], 0)} \\to ${num(c[c.length - 1], 0)}\\ \\text{in ${sim.plan.iters} passes}`);
          lines.push(`\\tau(t) = \\bar\\tau_k \\quad\\text{(the plan, exactly as computed)}`);
          lines.push({ note: "Followed blindly: nothing checks where the arm actually is until it reaches the top, where a steady LQR takes over to balance it.", cls: "eq-note" });
        } else {
          lines.push(`\\text{re-plans so far: } ${sim.solves},\\quad \\tau = ${num(sim.u, 2)}\\,\\text{N·m}`);
          lines.push({ note: "Every 100 ms it solves for the rest of the move from the measured angle and speed, toward a fixed finish time, and applies only the first step. If it reaches the finish without getting there, it sets a new one and tries again." });
        }
        lines.push({ note: sim.upright ? "Balanced at the top." : "Not at the top yet.", cls: "verdict" });
        return lines;
      },
    },
    {
      title: "SLAM: building the map as you go",
      part: "Part 4 · Beyond the arm",
      short: "SLAM",
      dim: "2d",
      box: { x0: -0.3, x1: 10.3, y0: -0.4, y1: 7.6 },
      grid: 1,
      hint: "",
      body: () => `
        <p>Lesson 1 asked "where is anything?" This last step asks it from the robot's side, with no map to go on.</p>
        <p>The Localization tab finds a robot on a map you hand it. <b>SLAM</b>, <i>simultaneous localization and mapping</i>, has to build the map and find itself on it at the same time.</p>
        <p>This robot drives a loop with a short-range sensor. The first time it sees a landmark, it adds it to its map where the measurement says, and that landmark picks up all the robot's uncertainty so far. The crosses are its map and the ellipses show how sure it is. Small wheel errors add up as it goes, so landmarks it maps later are less certain.</p>
        <p>Then it gets back to the start and sees its first landmarks again. That's a <b>loop closure</b>. The filter, an <b>EKF</b> (<i>extended Kalman filter</i>, like on the Localization tab), has kept track of how every landmark's error is tied to the robot's. So that one sighting fixes the robot and pulls the whole map back into place.</p>
        <div class="learn-try"><b>Try it</b><ul><li>Watch the chart below the map drop when the loop closes.</li><li>Shrink the sensor range or turn up wheel slip so it drifts more before it gets back.</li><li>Compare dead reckoning (dashed), which never looks at landmarks, so it just keeps drifting.</li></ul></div>
        <p class="learn-up-next"><b>That's the class.</b> You've gone from frames to a robot mapping a place it's never seen. Each tab up top goes deeper on one of these.</p>`,
      controls() {
        return [
          el("div", { class: "learn-row" },
            button(s8.playing ? "Pause" : "Play", (e) => { s8.playing = !s8.playing; e.currentTarget.textContent = s8.playing ? "Pause" : "Play"; }, "btn solid"),
            button("Start over", () => { s8.sim.reset(); render(); }),
            seg("Speed", [["1", "1×"], ["2", "2×"], ["4", "4×"]], String(s8.speed), (v) => (s8.speed = Number(v)))),
          sliders(
            slider({ label: "Sensor range", min: 1.5, max: 4, step: 0.1, value: s8.sim.range, unit: " m", digits: 1, onInput: (v) => { s8.sim.range = v; } }),
            slider({ label: "Wheel slip", min: 0.5, max: 5, step: 0.5, value: s8.sim.odomNoise, unit: "×", digits: 1, onInput: (v) => { s8.sim.odomNoise = v; } }),
          ),
          el("div", { class: "learn-row" }, check("Show dead reckoning", s8.showDR, (on) => { s8.showDR = on; render(); })),
        ];
      },
      tick(dt) {
        if (s8.playing) s8.sim.advance(dt * s8.speed);
      },
      draw(g) {
        const { ctx, v, c } = g;
        const sim = s8.sim;
        const tr = sim.truth;
        // The loop it drives, faint.
        ctx.save();
        ctx.setLineDash([4, 7]);
        ctx.strokeStyle = c.line;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.ellipse(v.X(LOOP.cx), v.Y(LOOP.cy), LOOP.rx * v.s, LOOP.ry * v.s, 0, 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
        // Sensor range and what it can see right now.
        ctx.save();
        ctx.fillStyle = c.teal;
        ctx.globalAlpha = 0.08;
        ctx.beginPath();
        ctx.arc(v.X(tr.x), v.Y(tr.y), sim.range * v.s, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
        for (const m of sim.z) dashed(g, [tr.x, tr.y], [sim.landmarks[m.j].x, sim.landmarks[m.j].y], c.teal);
        // True landmarks.
        sim.landmarks.forEach((lm) => {
          ctx.beginPath();
          ctx.arc(v.X(lm.x), v.Y(lm.y), 6, 0, Math.PI * 2);
          ctx.fillStyle = c.panel;
          ctx.fill();
          ctx.strokeStyle = c.strong;
          ctx.lineWidth = 2;
          ctx.stroke();
        });
        // The map SLAM has built.
        for (const m of sim.slam.map()) {
          const t = sim.landmarks[m.j];
          ctx.save();
          ctx.strokeStyle = c.accent;
          ctx.globalAlpha = 0.4;
          ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.moveTo(v.X(m.x), v.Y(m.y));
          ctx.lineTo(v.X(t.x), v.Y(t.y));
          ctx.stroke();
          ctx.globalAlpha = 0.13;
          ctx.fillStyle = c.accent;
          ctx.beginPath();
          ctx.ellipse(v.X(m.x), v.Y(m.y), Math.max(3, m.rx * v.s), Math.max(3, m.ry * v.s), -m.angle, 0, Math.PI * 2);
          ctx.fill();
          ctx.globalAlpha = 0.8;
          ctx.lineWidth = 1.5;
          ctx.stroke();
          ctx.globalAlpha = 1;
          ctx.lineWidth = 2.2;
          ctx.beginPath();
          ctx.moveTo(v.X(m.x) - 5, v.Y(m.y) - 5); ctx.lineTo(v.X(m.x) + 5, v.Y(m.y) + 5);
          ctx.moveTo(v.X(m.x) - 5, v.Y(m.y) + 5); ctx.lineTo(v.X(m.x) + 5, v.Y(m.y) - 5);
          ctx.stroke();
          ctx.restore();
        }
        // Trails: truth, SLAM, and dead reckoning if shown.
        const trail = (key, color, width, dash) => {
          if (sim.trail.length < 2) return;
          ctx.save();
          ctx.strokeStyle = color;
          ctx.lineWidth = width;
          if (dash) ctx.setLineDash(dash);
          ctx.beginPath();
          sim.trail.forEach((p, i) => (i ? ctx.lineTo : ctx.moveTo).call(ctx, v.X(p[key][0]), v.Y(p[key][1])));
          ctx.stroke();
          ctx.restore();
        };
        if (s8.showDR) trail("dr", c.subtle, 1.5, [5, 4]);
        trail("truth", c.fg, 2);
        trail("slam", c.accent, 2);
        const robotMark = (p, color, size) => {
          ctx.save();
          ctx.translate(v.X(p.x), v.Y(p.y));
          ctx.rotate(-p.th);
          ctx.fillStyle = color;
          ctx.beginPath();
          ctx.moveTo(size, 0); ctx.lineTo(-size * 0.7, size * 0.6); ctx.lineTo(-size * 0.7, -size * 0.6);
          ctx.closePath();
          ctx.fill();
          ctx.restore();
        };
        if (s8.showDR) robotMark(sim.dr.pose, c.subtle, 10);
        const e = sim.slam.estimate();
        const re = sim.slam.ellipse();
        ctx.save();
        ctx.globalAlpha = 0.15;
        ctx.fillStyle = c.accent;
        ctx.beginPath();
        ctx.ellipse(v.X(e.x), v.Y(e.y), Math.max(3, re.rx * v.s), Math.max(3, re.ry * v.s), -re.angle, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
        robotMark(e, c.accent, 13);
        robotMark(tr, c.fg, 11);
        ctx.fillStyle = c.muted;
        ctx.font = "13px Hyper, system-ui, sans-serif";
        ctx.textAlign = "right";
        ctx.textBaseline = "top";
        ctx.fillText(`${num(sim.t, 0)} s`, size2.w - 10, 8);
        return sim;
      },
      math(sim) {
        const map = sim.slam.map();
        const n = sim.slam.mu.length;
        const spread = sim.mapSpread();
        const lines = [
          `\\mu = (x,\\ y,\\ \\theta,\\ m_1,\\ \\dots,\\ m_{${map.length}}),\\quad ${n} \\text{ numbers},\\quad \\Sigma:\\ ${n} \\times ${n}`,
          { note: `${map.length} of ${sim.landmarks.length} landmarks mapped. ${inl("\\Sigma")} holds how sure it is of each one and, more importantly, how their errors are tied to the robot's and to each other.` },
          `\\text{robot off by } ${num(sim.error("slam") * 100, 0)}\\,\\text{cm},\\quad \\text{dead reckoning off by } ${num(sim.error("dr") * 100, 0)}\\,\\text{cm}`,
        ];
        if (spread !== null) lines.push(`\\text{map uncertainty (mean 95\\% radius)} = ${num(spread * 100, 0)}\\,\\text{cm}`);
        lines.push({ note: sparkline(sim), cls: "learn-spark" });
        const last = sim.closures[sim.closures.length - 1];
        lines.push({
          note: !last ? "No loop closed yet: every landmark so far has been seen only on this pass."
            : `Loop closed at ${num(last.t, 0)} s: it saw landmark ${last.j + 1} again after a long gap.`,
          cls: "verdict",
        });
        return lines;
      },
    },
  ];

  /* ---------- lesson helpers ---------- */

  function restart7() {
    s7.sim.reset();
    s7.trace = [];
    s7.t0 = 0;
    if (s7.sim.mode === "follow") {
      s7.sim.replan();
      s7.iterations = s7.sim.iterations;
      s7.reveal = 0; // the passes appear one by one before the arm moves
    } else {
      s7.iterations = [];
      s7.reveal = 1;
    }
    render();
  }
  function markChart7() {
    s7.t0 = s7.sim.t;
    s7.trace = [];
  }

  /** Map uncertainty over time, as a small inline chart with loop closures marked. */
  function sparkline(sim) {
    const h = sim.history;
    if (h.length < 2) return "";
    const W = 300, H = 56, t0 = h[0].t, t1 = h[h.length - 1].t;
    const max = Math.max(0.05, ...h.map((p) => p.spread ?? 0));
    const X = (t) => ((t - t0) / Math.max(1e-6, t1 - t0)) * W;
    const Y = (v) => H - 4 - (v / max) * (H - 10);
    const pts = h.filter((p) => p.spread !== null).map((p) => `${X(p.t).toFixed(1)},${Y(p.spread).toFixed(1)}`).join(" ");
    const marks = sim.closures.filter((c) => c.t >= t0).map((c) => `<line x1="${X(c.t).toFixed(1)}" x2="${X(c.t).toFixed(1)}" y1="2" y2="${H}" stroke="var(--teal)" stroke-dasharray="3 3"/>`).join("");
    return `<svg viewBox="0 0 ${W} ${H}" width="100%" height="${H}" role="img" aria-label="Map uncertainty over time">${marks}<polyline points="${pts}" fill="none" stroke="var(--accent)" stroke-width="2"/></svg><span class="small subtle">map uncertainty over time; dashed lines are loop closures</span>`;
  }

  let armSliders = [];
  let targetSliders = [];
  function syncArmSliders() { armSliders.forEach((s, i) => s.set(s3.q[i] / DEG)); }
  function setTarget(t, { solve = true, singular = null } = {}) {
    s3.target = solve ? t.map((v) => Math.round(v * 100) / 100) : [...t];
    s3.singular = singular;
    targetSliders.slice(0, 3).forEach((s, i) => s.set(s3.target[i]));
    if (solve) {
      startSolve();
    } else {
      s3.solving = false;
      s3.iters = 0;
      render();
    }
  }
  const yprMatrix = () => matMul(matMul(rotZ(s3.ypr[0]), rotY(s3.ypr[1])), rotX(s3.ypr[2]));
  const otherOrder = () => matMul(matMul(rotX(s3.ypr[2]), rotY(s3.ypr[1])), rotZ(s3.ypr[0]));
  function openRotation(R) {
    window.dispatchEvent(new CustomEvent("rb:open-rotation", { detail: { q: matrixToQuat(R.flat()) } }));
  }

  // Numerical IK runs one step per frame so the convergence is visible.
  let solveFrame = 0;
  function startSolve() {
    s3.solving = true;
    s3.iters = 0;
    if (!solveFrame) solveFrame = requestAnimationFrame(solveTick);
  }
  function solveTick() {
    solveFrame = 0;
    if (!active || lesson().short !== "IK and the Jacobian") { s3.solving = false; return; }
    const step = ikStep(s3.q, s3.target, { lambda: s3.lambda, rest: ARM6.home });
    if (step.error < 1e-4 || s3.iters >= 300 || norm(step.q.map((v, i) => v - s3.q[i])) < 1e-7) {
      s3.solving = false;
    } else {
      s3.q = step.q;
      s3.iters++;
      solveFrame = requestAnimationFrame(solveTick);
    }
    render();
  }

  /* ---------- navigation and rendering ---------- */

  let index = 0;
  try { index = Math.min(LESSONS.length - 1, Math.max(0, Number(localStorage.getItem(STEP_KEY)) || 0)); } catch { /* storage blocked */ }
  const lesson = () => LESSONS[index];

  const steps = $("#learn-steps");
  const stepButtons = LESSONS.map((l, i) => {
    const b = el("button", { type: "button", onclick: () => go(i) },
      el("span", { class: "dot", text: String(i + 1) }), el("span", { class: "label", text: l.short }));
    steps.append(b);
    return b;
  });
  $("#learn-prev").addEventListener("click", () => go(index - 1));
  // The last lesson's button finishes the class and returns to the home page.
  $("#learn-next").addEventListener("click", () => {
    if (index === LESSONS.length - 1) {
      location.hash = "#";
      window.scrollTo({ top: 0, behavior: "smooth" });
    } else {
      go(index + 1);
    }
  });

  function go(i) {
    if (i < 0 || i >= LESSONS.length) return;
    index = i;
    try { localStorage.setItem(STEP_KEY, String(i)); } catch { /* storage blocked */ }
    open();
    root.querySelector(".learn-steps").scrollIntoView({ block: "nearest", behavior: "smooth" });
  }

  function open() {
    const l = lesson();
    stepButtons.forEach((b, i) => {
      if (i === index) b.setAttribute("aria-current", "step");
      else b.removeAttribute("aria-current");
      b.classList.toggle("done", i < index);
      b.setAttribute("aria-label", `Lesson ${i + 1}: ${LESSONS[i].short}`);
    });
    $("#learn-title").textContent = l.title;
    $("#learn-count").textContent = `${l.part} · ${index + 1} of ${LESSONS.length}`;
    $("#learn-body").innerHTML = l.body();
    $("#learn-prev").disabled = index === 0;
    const next = $("#learn-next");
    // On a phone the lesson name drops out of the Next label (CSS), or the
    // button runs past the card. The last one finishes the class and goes home.
    if (index === LESSONS.length - 1) {
      next.textContent = "Finished!";
      next.title = "Back to the roboticsbench home page";
    } else {
      next.replaceChildren("Next", el("span", { class: "learn-next-name", text: `: ${LESSONS[index + 1].short}` }), " →");
      next.removeAttribute("title");
    }
    controls.replaceChildren(...l.controls());
    wrap2d.hidden = l.dim !== "2d";
    wrap3d.hidden = l.dim !== "3d";
    wrap2d.classList.toggle("draggable", !!l.drag);
    hint.textContent = l.hint ?? "";
    if (l.dim === "3d") {
      ensure3d();
      vp.setActive(active);
    } else if (vp) {
      vp.setActive(false);
    }
    // Each visit to the IK lesson starts from home, so the solve it shows is
    // the same one every time rather than whatever lesson 5 left behind.
    if (l.short === "IK and the Jacobian") { s3.q = [...ARM6.home]; s3.singular = null; startSolve(); }
    if (l.short === "iLQR and MPC") restart7();
    render();
  }

  // Lessons with a tick() animate: physics every frame, the math a few times a second.
  let animLast = 0, mathClock = 0;
  function animate(now) {
    const dt = animLast ? Math.min(0.05, (now - animLast) / 1000) : 0;
    animLast = now;
    const l = lesson();
    if (active && l.tick) {
      l.tick(dt);
      mathClock += dt;
      const math = mathClock > 0.2;
      if (math) mathClock = 0;
      render(math);
    }
    requestAnimationFrame(animate);
  }
  requestAnimationFrame(animate);

  function render(withMath = true) {
    const l = lesson();
    let result;
    if (l.dim === "2d") {
      const g = begin2d(l.box, l.grid ?? 0.5, l.gridLabel !== false);
      if (!g) return;
      result = l.draw(g);
    } else {
      if (!vp) return;
      result = l.draw();
      vp.requestRender();
    }
    if (!withMath) return;
    mathHost.innerHTML = l.math(result).map((m) => (typeof m === "string"
      ? `<div class="eq live">${row(m)}</div>`
      : `<p class="${m.cls ?? "eq-note"}">${m.note}</p>`)).join("");
  }

  // Lesson 3: press or drag anywhere on the canvas to place the target.
  let dragging = false;
  const placeTarget = (e) => {
    const r = canvas.getBoundingClientRect();
    s2.target = view2(lesson().box).toWorld(e.clientX - r.left, e.clientY - r.top);
    render();
  };
  canvas.addEventListener("pointerdown", (e) => {
    if (!lesson().drag) return;
    dragging = true;
    canvas.setPointerCapture(e.pointerId);
    placeTarget(e);
  });
  canvas.addEventListener("pointermove", (e) => { if (dragging) placeTarget(e); });
  const stop = () => { dragging = false; };
  canvas.addEventListener("pointerup", stop);
  canvas.addEventListener("pointercancel", stop);

  window.addEventListener("rb:theme", () => { if (lesson().dim === "2d") render(); });

  open();

  return {
    setActive(on) {
      active = on;
      if (vp) vp.setActive(on && lesson().dim === "3d");
      if (on && lesson().short === "IK and the Jacobian" && s3.solving) startSolve();
    },
  };
}
