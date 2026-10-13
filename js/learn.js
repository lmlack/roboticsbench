// Learn: a six-step class on robot arm basics. Lessons 1–3 use a 2-link arm
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
  function seg(label, options, value, onChange) {
    const group = el("div", { class: "segmented", role: "group", "aria-label": label },
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

  function begin2d(box) {
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
    for (let x = Math.ceil(box.x0 * 2) / 2; x <= box.x1; x += 0.5) { ctx.moveTo(v.X(x), 0); ctx.lineTo(v.X(x), size2.h); }
    for (let y = Math.ceil(box.y0 * 2) / 2; y <= box.y1; y += 0.5) { ctx.moveTo(0, v.Y(y)); ctx.lineTo(size2.w, v.Y(y)); }
    ctx.stroke();
    ctx.fillStyle = c.subtle;
    ctx.font = "13px Hyper, system-ui, sans-serif";
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    ctx.fillText("grid: 0.5 m", 10, size2.h - 8);
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
      short: "Frames",
      dim: "2d",
      box: { x0: -0.9, x1: 2.7, y0: -0.6, y1: 2.4 },
      hint: "Move the base with the sliders",
      body: () => `
        <p>A position only means something <b>relative to a frame</b>: an origin plus a set of axes. The <b>world frame W</b> is fixed to the room. The robot's <b>base frame B</b> sits wherever the robot was bolted down, turned however it was installed.</p>
        <p>Move the base. The tool tip <b>P</b> does not move relative to the base, so its base coordinates ${inl("p_B")} stay the same, but its world coordinates ${inl("p_W")} change. Same point, two addresses.</p>
        <p>To convert, turn by the base's angle, then add where the base is. A <b>transform</b> ${inl("T")} packs both steps into one matrix.</p>
        <p><b>Homogeneous coordinates</b> are what make that work. A rotation matrix can turn a point but cannot shift it, because multiplying a matrix by the origin always gives the origin. So write the point with a 1 on the end, ${inl("(x, y, 1)")}, and put the shift in an extra column: ${inl("T = \\begin{bmatrix} R & t \\\\ 0 & 1 \\end{bmatrix}")}. Multiplying ${inl("T")} by ${inl("(x, y, 1)")} turns the point and adds ${inl("t")} in one go, and the bottom row keeps the 1 in place for the next matrix.</p>
        <p>That buys two things. <b>Chaining</b>: going through several frames is just multiplying their matrices, ${inl("T_{AC} = T_{AB}\\,T_{BC}")}, which is how the arm in the next lessons is built. <b>Going back</b>: the inverse is ${inl("T^{-1} = \\begin{bmatrix} R^\\top & -R^\\top t \\\\ 0 & 1 \\end{bmatrix}")}, no general matrix inversion needed. A direction rather than a point gets a 0 instead of the 1, so it turns but never shifts. In 3D it is the same idea with a 4×4 matrix.</p>`,
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
      title: "Forward kinematics: joints in, pose out",
      short: "Forward kinematics",
      dim: "2d",
      box: { x0: -1.9, x1: 1.9, y0: -1.9, y1: 1.9 },
      hint: "Turn the joints with the sliders",
      body: () => `
        <p><b>Forward kinematics (FK)</b> answers: given the joint angles, where is the tool? Follow the chain: turn by ${inl("\\theta_1")}, walk ${inl("\\ell_1")} along the first link, turn by ${inl("\\theta_2")}, walk ${inl("\\ell_2")}. Each step is a transform, and multiplying them in order gives the tool's pose.</p>
        <p>There is always exactly one answer, and it is cheap to compute. That is why FK is called the easy direction.</p>
        <p>The shaded ring is the <b>workspace</b>, every point the tip can reach. Its outer edge is the arm stretched straight, ${inl("\\ell_1 + \\ell_2")}; the inner edge is the arm folded back, ${inl("|\\ell_1 - \\ell_2|")}.</p>`,
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
      title: "Inverse kinematics: pose in, joints out",
      short: "Inverse kinematics",
      dim: "2d",
      box: { x0: -1.9, x1: 1.9, y0: -1.9, y1: 1.9 },
      hint: "Drag anywhere to move the target",
      drag: true,
      body: () => `
        <p><b>Inverse kinematics (IK)</b> runs the other way: here is where the tool should be, find the joint angles. Drag the target around.</p>
        <p>Usually there are <b>two answers</b>, elbow up and elbow down; the faded arm is the one not chosen. Outside the ring there are <b>none</b>, and the arm can only point at the target. Real arms have more ways to have many answers or none, which is why IK is the hard direction.</p>
        <p>For two links, the triangle from base to elbow to tip gives a closed form: the law of cosines fixes the elbow angle, then ${inl("\\theta_1")} aims the arm.</p>`,
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
      short: "Rotations, SO(3)",
      dim: "3d",
      body: () => `
        <p>In 3D, an orientation takes <b>three numbers</b>. Here they are yaw, pitch and roll: turn about Z, then about the new Y, then about the newer X. The thick axes are the turned frame; the thin ones are the world.</p>
        <p>Write the turned axes down as the columns of a 3×3 matrix and you have the <b>rotation matrix</b> ${inl("R")}. Every valid ${inl("R")} has columns of length one at right angles to each other, ${inl("R^\\top R = I")}, and ${inl("\\det R = +1")}, so it never mirrors. All such matrices together are called <b>SO(3)</b>, the special orthogonal group: orthogonal for ${inl("R^\\top R = I")}, special for the +1.</p>
        <p><b>Order matters.</b> Tick the box to see the same three angles applied in the opposite order. The rotation converter tab takes any of these between quaternions, Euler angles and matrices.</p>`,
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
      short: "FK in 3D",
      dim: "3d",
      body: () => `
        <p>An industrial arm chains <b>six joints</b>. Each adds one transform: shift along its link, then turn about its axis. Multiply all six and you have the tool's pose in the world, a rotation ${inl("R")} and a position ${inl("p")} together in one 4×4 matrix. It is the homogeneous matrix from lesson 1 with one more dimension: ${inl("R")} is now 3×3, ${inl("p")} has three entries, and the bottom row is ${inl("[0\\ 0\\ 0\\ 1]")}.</p>
        <p>Coral joints <b>bend</b> (about their Y); teal joints <b>twist</b> (about their Z). The small axes at each joint show that joint's own frame, and the big ones at the end are the tool frame.</p>
        <p>This is the same calculation the URDF viewer tab runs on any robot you load.</p>`,
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
      short: "IK and the Jacobian",
      dim: "3d",
      body: () => `
        <p>Six joints have no tidy closed form in general, so arms are usually solved <b>numerically</b>: from the current joints, take a small step that moves the tip toward the target, and repeat. The yellow ball is the <b>target</b>, the point the tip should reach; move it with the sliders and watch the arm converge on it.</p>
        <p>The <b>Jacobian</b> ${inl("J")} says how fast the tip moves for each joint's speed. Turning it around converts "move the tip this way" into "turn the joints this much". The damping ${inl("\\lambda")} keeps the step sensible where ${inl("J")} cannot be inverted.</p>
        <p>That happens at a <b>singularity</b>, where the arm loses a direction it can move in: fully stretched, or with two wrist axes in line. <b>Manipulability</b> ${inl("w")} measures how far the arm is from one, and drops to zero there. Try the buttons.</p>`,
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
          { note: "With six joints and only three position numbers to match, there are endless answers. The solver uses the spare freedom to drift back toward the home pose, so the arm does not end up in a knot." },
          { note: `${inl("J_v")}, the top three rows of ${inl("J")}: one column per joint, how the tip moves (m per radian) when only that joint turns.` },
          `J_v = ${mat(J.slice(0, 3), 2)}`,
          `w = \\sqrt{\\det\\left(J J^\\top\\right)} = ${num(w, 4)}${singular ? "\\quad \\text{(singular)}" : ""}`,
          { note: status, cls: "verdict" },
        ];
      },
    },
  ];

  /* ---------- lesson helpers ---------- */

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
    $("#learn-count").textContent = `${index + 1} of ${LESSONS.length}`;
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
    render();
  }

  function render() {
    const l = lesson();
    let result;
    if (l.dim === "2d") {
      const g = begin2d(l.box);
      if (!g) return;
      result = l.draw(g);
    } else {
      if (!vp) return;
      result = l.draw();
      vp.requestRender();
    }
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
