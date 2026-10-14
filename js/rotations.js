// Rotation converter UI. One canonical quaternion; every panel renders from it
// except the one being typed in, so a half-typed value is never overwritten.

import {
  EULER_ORDERS,
  IDENTITY_Q,
  axisAngleToQuat,
  eulerToQuat,
  matrixCheck,
  matrixToQuat,
  parsePasted,
  qNormalize,
  quatToAxisAngle,
  quatToEuler,
  quatToMatrix,
  quatToRotvec,
  randomQuat,
  rotvecToQuat,
} from "./rotation-math.js";
import { axisTriad, createViewport, disposeObject, themeColors } from "./three-common.js";
import { copyText, el, evalNumber, fmt, segmented } from "./util.js";

const DEG = Math.PI / 180;

export function start(root) {
  const $ = (sel) => root.querySelector(sel);
  let q = [...IDENTITY_Q];
  let codeLang = "scipy";

  // ---- Settings ----
  const qOrder = segmented($("#rot-qorder"), () => { buildQuatFields(); render(); });
  const eFrame = segmented($("#rot-eframe"), () => { labelEuler(); render(); });
  const unit = segmented($("#rot-unit"), () => { labelEuler(); render(); });
  segmented($("#rot-code-tabs"), (v) => { codeLang = v; renderCode(); });

  const orderSelect = $("#rot-eorder");
  for (const o of EULER_ORDERS) {
    const proper = o[0] === o[2];
    orderSelect.append(el("option", { value: o, text: `${[...o].join("-")}${proper ? "  (proper Euler)" : ""}` }));
  }
  orderSelect.value = "ZYX";
  orderSelect.addEventListener("change", () => { labelEuler(); render(); });

  const deg = () => unit.get() === "deg";
  const intrinsic = () => eFrame.get() === "intrinsic";

  // ---- Fields ----
  function numField(label, labelClass) {
    const input = el("input", { class: "field", inputmode: "decimal", spellcheck: "false", "aria-label": label });
    const wrap = el("div", { class: "num" }, el("label", { class: labelClass ?? "", text: label }), input);
    return { wrap, input, label: wrap.firstChild };
  }

  const groups = {};
  function makeGroup(name, container, labels, classes = []) {
    container.replaceChildren();
    const fields = labels.map((l, i) => numField(l, classes[i]));
    for (const f of fields) {
      container.append(f.wrap);
      f.input.addEventListener("input", () => onEdit(name));
      f.input.addEventListener("blur", () => render());
    }
    groups[name] = fields;
  }

  function buildQuatFields() {
    const order = qOrder.get() === "wxyz" ? ["w", "x", "y", "z"] : ["x", "y", "z", "w"];
    const cls = { w: "", x: "axis-x", y: "axis-y", z: "axis-z" };
    makeGroup("quat", $("#rot-quat"), order, order.map((k) => cls[k]));
  }
  buildQuatFields();
  makeGroup("euler", $("#rot-euler"), ["", "", ""]);
  makeGroup("axis", $("#rot-axis"), ["axis x", "axis y", "axis z", "angle"], ["axis-x", "axis-y", "axis-z", ""]);
  makeGroup("rotvec", $("#rot-rotvec"), ["x", "y", "z"], ["axis-x", "axis-y", "axis-z"]);
  {
    const box = $("#rot-matrix");
    box.replaceChildren();
    const fields = [];
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        const input = el("input", { class: "field", inputmode: "decimal", spellcheck: "false", "aria-label": `Row ${r + 1}, column ${c + 1}` });
        input.addEventListener("input", () => onEdit("matrix"));
        input.addEventListener("blur", () => render());
        box.append(input);
        fields.push({ input });
      }
    }
    groups.matrix = fields;
  }

  function labelEuler() {
    const order = orderSelect.value;
    const ord = ["1st", "2nd", "3rd"];
    const cls = { X: "axis-x", Y: "axis-y", Z: "axis-z" };
    groups.euler.forEach((f, i) => {
      f.label.textContent = `${order[i]} (${ord[i]}, ${deg() ? "deg" : "rad"})`;
      f.label.className = cls[order[i]];
      f.input.setAttribute("aria-label", `${order[i]} angle`);
    });
    const [a, b, c] = order;
    $("#rot-euler-desc").textContent = intrinsic()
      ? `Rotate about ${a}, then about the new ${b}, then the newer ${c}. Same as extrinsic ${c}-${b}-${a} with the angles reversed.`
      : `Rotate about world ${a}, then world ${b}, then world ${c}. Same as intrinsic ${c}-${b}-${a} with the angles reversed.`;
    $("#rot-axis-unit").textContent = `angle in ${deg() ? "deg" : "rad"}`;
  }
  labelEuler();

  // ---- Edit and render ----
  function read(name) {
    const vals = groups[name].map((f) => evalNumber(f.input.value));
    groups[name].forEach((f, i) => f.input.classList.toggle("invalid", Number.isNaN(vals[i])));
    return vals.every((v) => !Number.isNaN(v)) ? vals : null;
  }

  function onEdit(name) {
    const v = read(name);
    if (!v) return;
    const angle = (a) => (deg() ? a * DEG : a);
    let next = null;
    const status = $("#rot-matrix-status");
    if (name === "quat") {
      const [w, x, y, z] = qOrder.get() === "wxyz" ? v : [v[3], v[0], v[1], v[2]];
      if (Math.hypot(w, x, y, z) < 1e-12) return;
      next = [w, x, y, z];
    } else if (name === "euler") {
      next = eulerToQuat(v.map(angle), orderSelect.value, intrinsic());
    } else if (name === "axis") {
      if (Math.hypot(v[0], v[1], v[2]) < 1e-12) return;
      next = axisAngleToQuat(v.slice(0, 3), angle(v[3]));
    } else if (name === "rotvec") {
      next = rotvecToQuat(v);
    } else if (name === "matrix") {
      const { orthoError, det } = matrixCheck(v);
      if (det <= 0) {
        status.textContent = `Not a rotation: det = ${fmt(det, 3)}. A reflection or a singular matrix cannot be converted.`;
        status.className = "note status";
        return;
      }
      next = matrixToQuat(v);
      if (orthoError > 1e-4) {
        status.textContent = `Not orthonormal (off by ${orthoError.toExponential(1)}, det ${fmt(det, 4)}). Converted anyway; other forms show the closest rotation found.`;
        status.className = "note status";
      } else {
        status.textContent = "Orthonormal, det = 1.";
        status.className = "small subtle status";
      }
    }
    setQ(next, name);
  }

  function setQ(next, source) {
    q = qNormalize(next);
    render(source);
  }

  function writeGroup(name, values, digits) {
    groups[name].forEach((f, i) => {
      f.input.value = fmt(values[i], digits);
      f.input.classList.remove("invalid");
    });
  }

  function render(skip) {
    const active = root.contains(document.activeElement) ? document.activeElement : null;
    const editing = Object.entries(groups).find(([, fs]) => fs.some((f) => f.input === active))?.[0];
    const keep = skip ?? editing;
    const outAngle = (a) => (deg() ? a / DEG : a);

    if (keep !== "quat") {
      const [w, x, y, z] = q;
      writeGroup("quat", qOrder.get() === "wxyz" ? [w, x, y, z] : [x, y, z, w], 6);
    }
    const eul = quatToEuler(q, orderSelect.value, intrinsic());
    if (keep !== "euler") writeGroup("euler", eul.angles.map(outAngle), deg() ? 4 : 6);
    const gimbal = $("#rot-gimbal");
    gimbal.hidden = !eul.gimbalLock;
    if (eul.gimbalLock) {
      gimbal.textContent = "Gimbal lock: the middle angle makes the outer two rotate about the same axis. Only their combination is defined, so the last angle is shown as 0.";
    }
    const aa = quatToAxisAngle(q);
    if (keep !== "axis") writeGroup("axis", [...aa.axis, outAngle(aa.angle)], 6);
    if (keep !== "rotvec") writeGroup("rotvec", quatToRotvec(q), 6);
    const m = quatToMatrix(q);
    if (keep !== "matrix") {
      writeGroup("matrix", m, 6);
      const status = $("#rot-matrix-status");
      status.className = "small subtle status";
      status.textContent = "Orthonormal, det = 1.";
    }
    renderCode();
    updatePreview();
  }

  // ---- Paste ----
  const pasteBox = $("#rot-paste");
  const pasteStatus = $("#rot-paste-status");
  pasteBox.addEventListener("input", () => {
    const r = parsePasted(pasteBox.value);
    pasteStatus.hidden = !r;
    if (!r) return;
    const angle = (a) => (deg() ? a * DEG : a);
    let next = null;
    if (r.kind === "quat") next = r.q;
    else if (r.kind === "quat4") next = qOrder.get() === "wxyz" ? r.v : [r.v[3], r.v[0], r.v[1], r.v[2]];
    else if (r.kind === "matrix" && matrixCheck(r.m).det > 0) next = matrixToQuat(r.m);
    else if (r.kind === "vec3") {
      const conv = `${intrinsic() ? "intrinsic" : "extrinsic"} ${orderSelect.value}, ${unit.get()}`;
      r.why = `${r.why} (${conv})`;
      next = eulerToQuat(r.v.map(angle), orderSelect.value, intrinsic());
    }
    const ok = next && Math.hypot(...next) > 1e-12;
    pasteStatus.className = `note status${ok ? " ok" : ""}`;
    pasteStatus.textContent = ok ? r.why : r.kind === "matrix" ? "Nine numbers, but not a rotation matrix (det ≤ 0)." : r.why;
    if (ok) setQ(next, null);
  });

  $("#rot-reset").addEventListener("click", () => setQ(IDENTITY_Q, null));
  $("#rot-random").addEventListener("click", () => setQ(randomQuat(), null));

  // ---- Code ----
  function renderCode() {
    const [w, x, y, z] = q.map((v) => fmt(v, 6));
    const m = quatToMatrix(q).map((v) => fmt(v, 6));
    const order = orderSelect.value;
    const eul = quatToEuler(q, order, intrinsic()).angles.map((a) => fmt(a / DEG, 4));
    const seq = intrinsic() ? order : order.toLowerCase();
    const code = {
      scipy:
        `from scipy.spatial.transform import Rotation as R\n\n` +
        `# SciPy quaternions are scalar-last: x, y, z, w\n` +
        `r = R.from_quat([${x}, ${y}, ${z}, ${w}])\n` +
        `# Same rotation as Euler angles. Uppercase = intrinsic, lowercase = extrinsic.\n` +
        `r = R.from_euler("${seq}", [${eul.join(", ")}], degrees=True)`,
      eigen:
        `#include <Eigen/Geometry>\n\n` +
        `// Constructor order is w, x, y, z (coeffs() returns x, y, z, w)\n` +
        `Eigen::Quaterniond q(${w}, ${x}, ${y}, ${z});\n` +
        `Eigen::Matrix3d R = q.toRotationMatrix();`,
      ros:
        `# geometry_msgs/Quaternion\n` +
        `x: ${x}\ny: ${y}\nz: ${z}\nw: ${w}`,
      numpy:
        `import numpy as np\n\n` +
        `R = np.array([\n` +
        `    [${m[0]}, ${m[1]}, ${m[2]}],\n` +
        `    [${m[3]}, ${m[4]}, ${m[5]}],\n` +
        `    [${m[6]}, ${m[7]}, ${m[8]}],\n` +
        `])`,
    };
    $("#rot-code").textContent = code[codeLang];
  }
  $("#rot-copy").addEventListener("click", (e) => copyText($("#rot-code").textContent, e.currentTarget));

  // ---- 3D preview ----
  const vp = createViewport($("#rot-viewport"), { distance: 3.6, target: [0, 0, 0] });
  const { THREE, scene } = vp;
  let world, body, grid;

  function buildScene() {
    for (const o of [world, body, grid]) {
      if (o) {
        scene.remove(o);
        disposeObject(o);
      }
    }
    const c = themeColors();
    world = axisTriad(1.25, { thick: false, labels: true, opacity: 0.6 });
    grid = new THREE.GridHelper(3, 6, c.grid, c.gridFaint);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -0.9;

    body = new THREE.Group();
    body.add(axisTriad(1, { thick: true, labels: true }));
    // A flat plate whose leading edge comes to a point, so the orientation reads
    // even without axes. It is one extruded outline rather than a plate plus a
    // cone, because a separate cone sat on the +X arm and read as a second
    // arrowhead on that axis.
    const outline = new THREE.Shape();
    outline.moveTo(-0.45, 0.275);
    outline.lineTo(0.45, 0.275);
    outline.lineTo(0.67, 0);
    outline.lineTo(0.45, -0.275);
    outline.lineTo(-0.45, -0.275);
    outline.closePath();
    const plate = new THREE.Mesh(
      new THREE.ExtrudeGeometry(outline, { depth: 0.08, bevelEnabled: false }),
      new THREE.MeshStandardMaterial({ color: c.highlight, roughness: 0.6, transparent: true, opacity: 0.35 }),
    );
    plate.position.z = -0.04;
    body.add(plate);
    scene.add(grid, world, body);
    showQuat(q);
    showAxisArrow();
  }

  function updatePreview() {
    if (!body) return;
    if (playing) {
      // A new rotation mid-play restarts the animation toward it.
      startPlay();
      return;
    }
    showQuat(q);
    showAxisArrow();
  }

  // three stores quaternions as (x, y, z, w).
  function showQuat(p) {
    body.quaternion.set(p[1], p[2], p[3], p[0]);
    vp.requestRender();
  }

  // ---- Play ----
  // "Shortest path" turns about the axis-angle axis, the one rotation that
  // gets there directly. "Euler steps" plays the three elementary rotations
  // of the selected convention in order, which is where intrinsic and
  // extrinsic visibly differ.
  const playBtn = $("#rot-play");
  const caption = $("#rot-caption");
  const loop = $("#rot-loop");
  const playMode = segmented($("#rot-play-mode"), () => playing && startPlay());
  let playing = false;
  let playStart = 0;
  let axisLine = null;
  let active = true;
  const STEP_S = 1.1;
  const HOLD_S = 0.8;

  function setPlayLabel() {
    playBtn.querySelector("span").textContent = playing ? "Stop" : "Play";
    playBtn.querySelector("svg").innerHTML = playing
      ? '<rect x="4" y="4" width="8" height="8" rx="1" fill="currentColor"/>'
      : '<path d="M4.5 3v10l8.5-5z" fill="currentColor"/>';
  }

  function clearAxisLine() {
    if (axisLine) {
      scene.remove(axisLine);
      disposeObject(axisLine);
      axisLine = null;
    }
  }

  /**
   * The axis-angle axis as a see-through arrow through the origin, pointing
   * the way the right-hand rule turns. It shows while stopped and during a
   * shortest-path play, and hides for Euler steps, which turn about other axes.
   */
  function showAxisArrow() {
    clearAxisLine();
    if (!body || (playing && playMode.get() !== "path")) return;
    const { axis, angle } = quatToAxisAngle(q);
    if (angle < 1e-6) return;
    const mat = new THREE.MeshBasicMaterial({ color: themeColors().spin, transparent: true, opacity: 0.45, depthWrite: false });
    const L = 1.45, head = 0.2;
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 2 * L - head, 12), mat);
    shaft.position.y = -head / 2;
    const tip = new THREE.Mesh(new THREE.ConeGeometry(0.07, head, 20), mat);
    tip.position.y = L - head / 2;
    axisLine = new THREE.Group();
    axisLine.add(shaft, tip);
    axisLine.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(...axis));
    axisLine.renderOrder = 2;
    scene.add(axisLine);
    vp.requestRender();
  }

  function startPlay() {
    playing = true;
    playStart = performance.now();
    showAxisArrow();
    setPlayLabel();
    requestAnimationFrame(tick);
  }

  function stopPlay() {
    playing = false;
    caption.hidden = true;
    showAxisArrow();
    setPlayLabel();
    showQuat(q);
  }

  playBtn.addEventListener("click", () => (playing ? stopPlay() : startPlay()));

  function tick(now) {
    if (!playing || !active) return;
    const t = (now - playStart) / 1000;
    const deg = (a) => fmt(Math.abs(a) / DEG, 1);
    let total;
    if (playMode.get() === "path") {
      const { axis, angle } = quatToAxisAngle(q);
      total = STEP_S * 2;
      const f = ease(Math.min(1, t / total));
      showQuat(axisAngleToQuat(axis, angle * f));
      caption.hidden = false;
      caption.textContent = angle < 1e-6
        ? "Identity: nothing to turn"
        : `${deg(angle * f)}° of ${deg(angle)}° about the see-through arrow`;
    } else {
      const order = orderSelect.value;
      const { angles } = quatToEuler(q, order, intrinsic());
      total = STEP_S * 3;
      const k = Math.min(2, Math.floor(t / STEP_S));
      const partial = angles.map((a, i) => a * ease(Math.min(1, Math.max(0, t / STEP_S - i))));
      showQuat(eulerToQuat(partial, order, intrinsic()));
      const which = ["1st", "2nd", "3rd"][k];
      const about = intrinsic() ? `the frame's own ${order[k]}` : `world ${order[k]}`;
      caption.hidden = false;
      caption.textContent = t >= total
        ? `Done: ${intrinsic() ? "intrinsic" : "extrinsic"} ${[...order].join("-")}`
        : `${which}: about ${about}, ${deg(partial[k])}°`;
    }
    if (t >= total + HOLD_S) {
      if (loop.checked) playStart = now;
      else return stopPlay();
    }
    requestAnimationFrame(tick);
  }

  const ease = (x) => (x < 0.5 ? 2 * x * x : 1 - (-2 * x + 2) ** 2 / 2);

  window.addEventListener("rb:theme", buildScene);
  buildScene();
  render();

  return {
    setActive(on) {
      active = on;
      vp.setActive(on);
      if (on && playing) requestAnimationFrame(tick);
    },
    setQuaternion(next) {
      setQ(next, null);
    },
  };
}
