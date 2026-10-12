// URDF viewer: load a robot (sample, files, folder, pasted text), pose it with
// joint sliders, and read forward kinematics and the geometric Jacobian.

import URDFLoader from "urdf-loader";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { ColladaLoader } from "three/examples/jsm/loaders/ColladaLoader.js";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import { createViewport, disposeObject, themeColors } from "./three-common.js";
import { SAMPLES } from "./urdf-samples.js";
import { quatToEuler } from "./rotation-math.js";
import { copyText, el, fmt, segmented } from "./util.js";

const MOVABLE = new Set(["revolute", "continuous", "prismatic"]);
const DEG = Math.PI / 180;

export function start(root) {
  const $ = (sel) => root.querySelector(sel);
  const vp = createViewport($("#urdf-viewport"), { distance: 2.2, target: [0, 0, 0.4] });
  const { THREE, scene } = vp;

  let grid;
  const buildGrid = () => {
    if (grid) {
      scene.remove(grid);
      disposeObject(grid);
    }
    const c = themeColors();
    grid = new THREE.GridHelper(4, 40, c.grid, c.gridFaint);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);
    vp.requestRender();
  };
  buildGrid();
  window.addEventListener("rb:theme", () => {
    buildGrid();
    if (robot) refreshHelpers();
  });

  let robot = null;
  let joints = [];
  // Link frames and joint arrows, parented to links and joints so they move
  // with the robot. Kept here only so they can be removed.
  let helperObjs = [];
  let blobUrls = [];

  const status = $("#urdf-status");
  function setStatus(text, warn = false, list = []) {
    status.hidden = !text;
    status.className = `note status${warn ? "" : " ok"}`;
    status.replaceChildren(text);
    if (list.length) {
      status.append(el("ul", { class: "warn-list" }, ...list.slice(0, 8).map((t) => el("li", { text: t }))));
      if (list.length > 8) status.append(el("div", { text: `and ${list.length - 8} more` }));
    }
  }

  // ---- Loading ----

  /** files: array of { path, file }. Paths are relative, '/'-separated. */
  function makeResolver(files) {
    const entries = files.map(({ path, file }) => ({ path: path.toLowerCase(), file }));
    const cache = new Map();
    return (rawPath) => {
      const clean = decodeURIComponent(rawPath)
        .replace(/^package:\/\//, "")
        .replace(/^(\.\.?\/)+/, "")
        .replace(/^\/+/, "")
        .toLowerCase();
      if (cache.has(clean)) return cache.get(clean);
      // Longest suffix match first, so meshes/visual/base.stl beats another base.stl.
      const parts = clean.split("/");
      let hit = null;
      for (let k = 0; k < parts.length && !hit; k++) {
        const suffix = parts.slice(k).join("/");
        hit = entries.find((e) => e.path === suffix || e.path.endsWith(`/${suffix}`));
      }
      const url = hit ? URL.createObjectURL(hit.file) : null;
      if (url) blobUrls.push(url);
      cache.set(clean, url ? { url, name: hit.path } : null);
      return cache.get(clean);
    };
  }

  function loadRobot(text, files = [], pose = {}, label = "") {
    for (const u of blobUrls) URL.revokeObjectURL(u);
    blobUrls = [];
    const resolve = makeResolver(files);
    const missing = [];
    const manager = new THREE.LoadingManager();
    // Textures referenced from inside DAE files go through here too.
    manager.setURLModifier((url) => (url.startsWith("blob:") || url.startsWith("data:") ? url : resolve(url)?.url ?? url));

    const loader = new URDFLoader(manager);
    loader.packages = (pkg) => pkg;
    loader.parseCollision = true;
    loader.loadMeshCb = (path, mgr, done) => {
      const hit = resolve(path);
      if (!hit) {
        missing.push(path);
        done(null);
        return;
      }
      const ext = hit.name.split(".").pop();
      const fail = (err) => {
        missing.push(`${path} (${err?.message ?? "could not parse"})`);
        done(null);
      };
      if (ext === "stl") {
        new STLLoader(mgr).load(hit.url, (geom) => done(new THREE.Mesh(geom, new THREE.MeshStandardMaterial({ color: 0xc9cfd8 }))), undefined, fail);
      } else if (ext === "dae") {
        new ColladaLoader(mgr).load(hit.url, (dae) => {
          // ColladaLoader turns Z_UP assets to Y-up for three's default
          // scene. This scene is Z-up like ROS, so undo that, and turn Y_UP
          // assets the way RViz does.
          const s = dae.scene;
          s.rotation.set(Math.abs(s.rotation.x + Math.PI / 2) < 1e-6 ? 0 : Math.PI / 2, 0, 0);
          done(s);
        }, undefined, fail);
      } else if (ext === "obj") {
        new OBJLoader(mgr).load(hit.url, done, undefined, fail);
      } else {
        fail({ message: `.${ext} meshes are not supported` });
      }
    };

    let next;
    try {
      next = loader.parse(text);
    } catch (e) {
      setStatus(`Could not parse the URDF: ${e.message}`, true);
      return;
    }
    if (!next || !Object.keys(next.links ?? {}).length) {
      setStatus("That does not look like a URDF: no <link> elements found.", true);
      return;
    }

    if (robot) {
      scene.remove(robot);
      disposeObject(robot);
    }
    robot = next;
    scene.add(robot);
    applyCollisionVisibility();

    joints = Object.values(robot.joints).filter((j) => MOVABLE.has(j.jointType) && !j.isURDFMimicJoint);
    for (const j of joints) {
      if (j.name in pose) j.setJointValue(pose[j.name]);
    }
    buildJointList();
    buildLinkSelects();
    refreshHelpers();
    update();
    fitCamera();

    const done = () => {
      fitCamera();
      update();
      const counts = `${Object.keys(robot.links).length} links, ${joints.length} movable joints`;
      if (missing.length) setStatus(`Loaded ${robot.robotName || label} (${counts}), but ${missing.length} mesh${missing.length > 1 ? "es were" : " was"} not found:`, true, missing);
      else setStatus(`Loaded ${robot.robotName || label}: ${counts}.`);
    };
    // The manager only fires onLoad if something was queued.
    let queued = false;
    manager.onStart = () => (queued = true);
    manager.onLoad = done;
    setTimeout(() => {
      if (!queued) done();
    }, 0);
  }

  function fitCamera() {
    if (!robot) return;
    robot.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(robot);
    if (box.isEmpty()) return;
    const size = box.getSize(new THREE.Vector3()).length() || 1;
    const center = box.getCenter(new THREE.Vector3());
    vp.controls.target.copy(center);
    const dir = new THREE.Vector3(0.8, 0.7, 0.45).normalize();
    vp.camera.position.copy(center).addScaledVector(dir, size * 1.5);
    vp.camera.near = size / 100;
    vp.camera.far = size * 100;
    vp.camera.updateProjectionMatrix();
    vp.controls.update();
    vp.requestRender();
  }

  // ---- Sample / file inputs ----

  const sampleSelect = $("#urdf-sample");
  for (const s of SAMPLES) sampleSelect.append(el("option", { value: s.id, text: s.name }));
  sampleSelect.append(el("option", { value: "custom", text: "Your robot", disabled: true }));
  const loadSample = (id) => {
    const s = SAMPLES.find((x) => x.id === id);
    loadRobot(s.urdf, [], s.pose, s.name);
  };
  sampleSelect.addEventListener("change", () => loadSample(sampleSelect.value));

  async function loadFiles(files) {
    const urdfs = files.filter((f) => /\.(urdf|xml)$/i.test(f.path));
    const xacro = files.find((f) => /\.xacro$/i.test(f.path));
    if (!urdfs.length) {
      setStatus(
        xacro
          ? "Found a .xacro file but no .urdf. Expand it first: ros2 run xacro xacro robot.urdf.xacro > robot.urdf"
          : "No .urdf file in what was opened. Include the URDF along with its meshes.",
        true,
      );
      return;
    }
    // Prefer a file named .urdf over generic .xml, then the shortest path.
    urdfs.sort((a, b) => /\.urdf$/i.test(b.path) - /\.urdf$/i.test(a.path) || a.path.length - b.path.length);
    const main = urdfs[0];
    const text = await main.file.text();
    sampleSelect.value = "custom";
    loadRobot(text, files, {}, main.path.split("/").pop());
  }

  const fromInput = (input) => [...input.files].map((file) => ({ file, path: file.webkitRelativePath || file.name }));
  $("#urdf-files").addEventListener("change", (e) => loadFiles(fromInput(e.target)));
  $("#urdf-folder").addEventListener("change", (e) => loadFiles(fromInput(e.target)));
  $("#urdf-text-load").addEventListener("click", () => {
    const text = $("#urdf-text").value.trim();
    if (!text) return;
    sampleSelect.value = "custom";
    loadRobot(text, [], {}, "pasted URDF");
  });

  // Drag and drop, including whole folders.
  const viewport = $("#urdf-viewport");
  const dropHint = $("#urdf-drop");
  let dragDepth = 0;
  viewport.addEventListener("dragenter", (e) => {
    e.preventDefault();
    dragDepth++;
    viewport.classList.add("drag");
    dropHint.hidden = false;
  });
  viewport.addEventListener("dragover", (e) => e.preventDefault());
  viewport.addEventListener("dragleave", () => {
    if (--dragDepth <= 0) {
      dragDepth = 0;
      viewport.classList.remove("drag");
      dropHint.hidden = true;
    }
  });
  viewport.addEventListener("drop", async (e) => {
    e.preventDefault();
    dragDepth = 0;
    viewport.classList.remove("drag");
    dropHint.hidden = true;
    const entries = [...e.dataTransfer.items].map((i) => i.webkitGetAsEntry?.()).filter(Boolean);
    const files = [];
    const walk = async (entry, prefix) => {
      if (entry.isFile) {
        const file = await new Promise((res, rej) => entry.file(res, rej));
        files.push({ file, path: prefix + entry.name });
      } else if (entry.isDirectory) {
        const reader = entry.createReader();
        // readEntries returns batches; keep reading until it comes back empty.
        for (;;) {
          const batch = await new Promise((res, rej) => reader.readEntries(res, rej));
          if (!batch.length) break;
          for (const child of batch) await walk(child, `${prefix}${entry.name}/`);
        }
      }
    };
    if (entries.length) for (const entry of entries) await walk(entry, "");
    else for (const file of e.dataTransfer.files) files.push({ file, path: file.name });
    loadFiles(files);
  });

  // ---- Joints ----

  const jointList = $("#urdf-joints");
  function range(j) {
    if (j.jointType === "continuous") return [-Math.PI, Math.PI];
    const lo = Number(j.limit?.lower ?? 0);
    const hi = Number(j.limit?.upper ?? 0);
    if (lo === 0 && hi === 0) return j.jointType === "prismatic" ? [-0.5, 0.5] : [-Math.PI, Math.PI];
    return [lo, hi];
  }
  const show = (j, v) => (j.jointType === "prismatic" ? `${fmt(v, 3)} m` : `${fmt(v / DEG, 1)}°`);

  function buildJointList() {
    jointList.replaceChildren();
    if (!joints.length) {
      jointList.append(el("p", { class: "small subtle", text: "No movable joints." }));
      return;
    }
    for (const j of joints) {
      const [lo, hi] = range(j);
      const val = el("span", { class: "val" });
      const slider = el("input", { type: "range", min: lo, max: hi, step: "any", "aria-label": j.name });
      slider.value = j.angle;
      const sync = () => (val.textContent = show(j, j.angle));
      slider.addEventListener("input", () => {
        j.setJointValue(Number(slider.value));
        sync();
        update();
      });
      j.__slider = slider;
      j.__sync = sync;
      sync();
      jointList.append(
        el("div", { class: "joint" },
          el("div", { class: "joint-head" },
            el("span", { class: "name", text: j.name, title: j.name }),
            el("span", { class: "chip", text: j.jointType }),
            val),
          slider,
          el("div", { class: "limits" }, el("span", { text: show(j, lo) }), el("span", { text: show(j, hi) }))),
      );
    }
  }

  function setAll(fn) {
    for (const j of joints) {
      j.setJointValue(fn(j));
      j.__slider.value = j.angle;
      j.__sync();
    }
    update();
  }
  $("#urdf-zero").addEventListener("click", () => setAll((j) => Math.min(Math.max(0, range(j)[0]), range(j)[1])));
  $("#urdf-random").addEventListener("click", () => setAll((j) => {
    const [lo, hi] = range(j);
    return lo + Math.random() * (hi - lo);
  }));

  // ---- Helpers: link frames and joint axes ----

  function robotScale() {
    const box = new THREE.Box3().setFromObject(robot);
    return box.isEmpty() ? 1 : box.getSize(new THREE.Vector3()).length();
  }

  function refreshHelpers() {
    for (const h of helperObjs) {
      h.removeFromParent();
      disposeObject(h);
    }
    helperObjs = [];
    if (!robot) return;
    robot.updateMatrixWorld(true);
    const s = robotScale();
    const c = themeColors();
    if ($("#urdf-frames").checked) {
      for (const link of Object.values(robot.links)) {
        const ax = new THREE.AxesHelper(s * 0.06);
        ax.setColors(c.x, c.y, c.z);
        ax.material.depthTest = false;
        ax.renderOrder = 5;
        link.add(ax);
        helperObjs.push(ax);
      }
    }
    if ($("#urdf-axes").checked) {
      for (const j of joints) {
        const dir = j.axis.clone().normalize();
        const arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(), s * 0.12, c.accent, s * 0.03, s * 0.018);
        arrow.traverse((o) => {
          if (o.material) {
            o.material.depthTest = false;
            o.renderOrder = 6;
          }
        });
        j.add(arrow);
        helperObjs.push(arrow);
      }
    }
    vp.requestRender();
  }
  $("#urdf-frames").addEventListener("change", refreshHelpers);
  $("#urdf-axes").addEventListener("change", refreshHelpers);

  function applyCollisionVisibility() {
    if (!robot) return;
    const on = $("#urdf-collision").checked;
    robot.traverse((o) => {
      if (o.isURDFCollider) {
        o.visible = on;
        o.traverse((m) => {
          if (m.isMesh) {
            m.material = new THREE.MeshBasicMaterial({ color: 0xd97706, transparent: true, opacity: 0.35, depthWrite: false });
          }
        });
      }
    });
    vp.requestRender();
  }
  $("#urdf-collision").addEventListener("change", applyCollisionVisibility);

  // ---- FK and Jacobian ----

  const baseSel = $("#urdf-base");
  const tipSel = $("#urdf-tip");
  function buildLinkSelects() {
    const names = Object.keys(robot.links);
    for (const sel of [baseSel, tipSel]) {
      sel.replaceChildren(...names.map((n) => el("option", { value: n, text: n })));
    }
    baseSel.value = robot.name in robot.links ? robot.name : names[0];
    // Default tip: the deepest leaf link.
    let best = names[0], bestDepth = -1;
    for (const n of names) {
      const link = robot.links[n];
      const isLeaf = !link.children.some((c) => c.isURDFJoint);
      let d = 0;
      for (let p = link; p && p !== robot; p = p.parent) d++;
      if (isLeaf && d > bestDepth) {
        best = n;
        bestDepth = d;
      }
    }
    tipSel.value = best;
  }
  baseSel.addEventListener("change", update);
  tipSel.addEventListener("change", update);

  let lastT = null;
  let lastQ = null;

  function update() {
    if (!robot) return;
    robot.updateMatrixWorld(true);
    const base = robot.links[baseSel.value];
    const tip = robot.links[tipSel.value];
    const baseInv = base.matrixWorld.clone().invert();
    const T = baseInv.clone().multiply(tip.matrixWorld);
    const p = new THREE.Vector3();
    const q3 = new THREE.Quaternion();
    T.decompose(p, q3, new THREE.Vector3());
    const q = [q3.w, q3.x, q3.y, q3.z];
    if (q[0] < 0) q.forEach((v, i) => (q[i] = -v));
    const rpy = quatToEuler(q, "XYZ", false).angles;
    $("#fk-pos").textContent = [p.x, p.y, p.z].map((v) => fmt(v, 4)).join("  ");
    $("#fk-quat").textContent = q.map((v) => fmt(v, 4)).join("  ");
    $("#fk-rpy").textContent = rpy.map((v) => fmt(v / DEG, 2)).join("  ");
    lastT = T;
    lastQ = q;
    renderJacobian(base, tip, baseInv);
    vp.requestRender();
  }

  function renderJacobian(base, tip, baseInv) {
    const table = $("#jac-table");
    const meta = $("#jac-meta");
    // Joints between base and tip, walking up from the tip.
    const chain = [];
    let node = tip;
    while (node && node !== base) {
      if (node.isURDFJoint) chain.unshift(node);
      node = node.parent;
    }
    if (node !== base) {
      lastJ = null;
      table.replaceChildren();
      meta.textContent = "";
      table.append(el("tr", {}, el("td", { class: "small", style: "text-align:left;font-family:var(--font-sans)", text: "Pick a base link that is an ancestor of the tip." })));
      return;
    }

    const rotInv = new THREE.Matrix3().setFromMatrix4(baseInv);
    const pTip = new THREE.Vector3().setFromMatrixPosition(tip.matrixWorld).applyMatrix4(baseInv);
    const cols = new Map(); // joint name -> column [vx vy vz wx wy wz]
    for (const j of chain) {
      if (!MOVABLE.has(j.jointType)) continue;
      const z = j.axis.clone().normalize().transformDirection(j.matrixWorld).applyMatrix3(rotInv).normalize();
      const pj = new THREE.Vector3().setFromMatrixPosition(j.matrixWorld).applyMatrix4(baseInv);
      const col = j.jointType === "prismatic"
        ? [z.x, z.y, z.z, 0, 0, 0]
        : (() => {
            const v = z.clone().cross(pTip.clone().sub(pj));
            return [v.x, v.y, v.z, z.x, z.y, z.z];
          })();
      // A mimic joint moves with its master: fold its column into the master's.
      const key = j.isURDFMimicJoint ? j.mimicJoint : j.name;
      const k = j.isURDFMimicJoint ? Number(j.multiplier ?? 1) : 1;
      const acc = cols.get(key) ?? [0, 0, 0, 0, 0, 0];
      cols.set(key, acc.map((a, i) => a + k * col[i]));
    }

    const names = [...cols.keys()];
    const J = names.map((n) => cols.get(n));
    lastJ = { names, J, base: base.name, tip: tip.name };
    table.replaceChildren();
    if (!names.length) {
      lastJ = null;
      meta.textContent = "";
      table.append(el("tr", {}, el("td", { class: "small", style: "text-align:left;font-family:var(--font-sans)", text: "No movable joints between these links." })));
      return;
    }
    table.append(el("tr", {}, el("th", { class: "row", text: "" }), ...names.map((n) => el("th", { text: n }))));
    const rows = ["vx", "vy", "vz", "ωx", "ωy", "ωz"];
    rows.forEach((r, i) => {
      table.append(el("tr", {}, el("th", { class: "row", text: r }), ...J.map((c) => el("td", { text: fmt(c[i], 3) }))));
    });

    // Yoshikawa manipulability: sqrt(det(J Jᵀ)) for 6+ joints, else sqrt(det(Jᵀ J)).
    const n = names.length;
    const M = n >= 6
      ? rows.map((_, a) => rows.map((_, b) => J.reduce((s, c) => s + c[a] * c[b], 0)))
      : J.map((ca) => J.map((cb) => ca.reduce((s, v, i) => s + v * cb[i], 0)));
    const w = Math.sqrt(Math.max(0, det(M)));
    meta.textContent = `${n} joint${n > 1 ? "s" : ""} · manipulability ${w < 1e-4 ? "≈ 0, singular" : fmt(w, 4)}`;
  }

  function det(A) {
    const m = A.map((r) => [...r]);
    const n = m.length;
    let d = 1;
    for (let c = 0; c < n; c++) {
      let piv = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(m[r][c]) > Math.abs(m[piv][c])) piv = r;
      if (Math.abs(m[piv][c]) < 1e-14) return 0;
      if (piv !== c) {
        [m[piv], m[c]] = [m[c], m[piv]];
        d = -d;
      }
      d *= m[c][c];
      for (let r = c + 1; r < n; r++) {
        const f = m[r][c] / m[c][c];
        for (let k = c; k < n; k++) m[r][k] -= f * m[c][k];
      }
    }
    return d;
  }

  // ---- Copy the Jacobian ----
  // Full precision, rows v then ω, columns in chain order, so it pastes
  // straight into a solver. The comment line keeps the frame and joints with it.
  let lastJ = null;
  const jacFormat = segmented($("#jac-format"));
  function jacobianText({ names, J, base, tip }, format) {
    const rows = [0, 1, 2, 3, 4, 5].map((r) => J.map((col) => fmt(col[r], 6)));
    const about = `Geometric Jacobian of ${tip} in ${base} frame. Rows: vx vy vz wx wy wz. Columns: ${names.join(", ")}`;
    if (format === "matlab") {
      return `% ${about}\nJ = [\n${rows.map((r) => `  ${r.join(", ")}`).join(";\n")}\n];`;
    }
    if (format === "csv") {
      const labels = ["vx", "vy", "vz", "wx", "wy", "wz"];
      return [["row", ...names].join(","), ...rows.map((r, i) => [labels[i], ...r].join(","))].join("\n");
    }
    return `import numpy as np\n\n# ${about}\nJ = np.array([\n${rows.map((r) => `    [${r.join(", ")}],`).join("\n")}\n])`;
  }
  $("#jac-copy").addEventListener("click", (e) => {
    if (lastJ) copyText(jacobianText(lastJ, jacFormat.get()), e.currentTarget);
  });

  $("#fk-send").addEventListener("click", () => {
    if (lastQ) window.dispatchEvent(new CustomEvent("rb:open-rotation", { detail: { q: lastQ } }));
  });
  $("#fk-copy").addEventListener("click", (e) => {
    if (!lastT) return;
    const m = lastT.elements; // column-major
    const rows = [0, 1, 2, 3].map((r) => [0, 1, 2, 3].map((c) => fmt(m[c * 4 + r], 6)).join(", "));
    copyText(`[[${rows.join("],\n [")}]]`, e.currentTarget);
  });

  loadSample(SAMPLES[0].id);

  return { setActive: vp.setActive };
}
