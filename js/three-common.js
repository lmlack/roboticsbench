// Shared three.js setup for the two 3D viewports. Scenes are Z-up, as in ROS
// and URDF, by setting the camera's up vector rather than rotating the robot.

import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { cssVar } from "./util.js";

export function themeColors() {
  return {
    bg: new THREE.Color(cssVar("--raised")),
    grid: new THREE.Color(cssVar("--line-strong")),
    gridFaint: new THREE.Color(cssVar("--line")),
    fg: cssVar("--fg"),
    x: new THREE.Color(cssVar("--axis-x")),
    y: new THREE.Color(cssVar("--axis-y")),
    z: new THREE.Color(cssVar("--axis-z")),
    accent: new THREE.Color(cssVar("--accent")),
    highlight: new THREE.Color(cssVar("--highlight")),
    spin: new THREE.Color(cssVar("--spin-axis")),
  };
}

/**
 * Renderer, camera and orbit controls in a container that sizes itself.
 * Renders on demand: call `requestRender()` after changing the scene.
 */
export function createViewport(container, { distance = 3, target = [0, 0, 0.3] } = {}) {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  container.prepend(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 200);
  camera.up.set(0, 0, 1);
  const d = distance;
  camera.position.set(d * 0.8, d * 0.7, d * 0.55);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(...target);
  controls.enableDamping = true;
  controls.update();

  scene.add(new THREE.HemisphereLight(0xffffff, 0x5a6578, 2.2));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(3, 2, 5);
  scene.add(sun);

  let pending = false;
  let active = true;
  const frame = () => {
    pending = false;
    if (!active) return;
    // Damping needs a few frames to coast to a stop after a drag.
    if (controls.update()) requestRender();
    renderer.render(scene, camera);
  };
  function requestRender() {
    if (!pending) {
      pending = true;
      requestAnimationFrame(frame);
    }
  }
  controls.addEventListener("change", requestRender);

  const resize = () => {
    const { clientWidth: w, clientHeight: h } = container;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    requestRender();
  };
  new ResizeObserver(resize).observe(container);

  const applyTheme = () => {
    scene.background = themeColors().bg;
    requestRender();
  };
  applyTheme();
  window.addEventListener("rb:theme", applyTheme);

  return {
    THREE, scene, camera, controls, renderer, requestRender,
    setActive(on) {
      active = on;
      if (on) resize();
    },
  };
}

/** A text label that always faces the camera. */
export function textSprite(text, color, size = 0.12) {
  const canvas = document.createElement("canvas");
  const px = 96;
  canvas.width = px * 2;
  canvas.height = px;
  const ctx = canvas.getContext("2d");
  ctx.font = `600 ${px * 0.7}px Hyper, system-ui, sans-serif`;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillStyle = color;
  ctx.fillText(text, px, px / 2);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  sprite.scale.set(size * 2, size, 1);
  sprite.renderOrder = 10;
  return sprite;
}

/** RGB triad. `thick` draws solid arrows, otherwise thin lines. */
export function axisTriad(length = 1, { thick = true, labels = false, opacity = 1 } = {}) {
  const group = new THREE.Group();
  const c = themeColors();
  const dirs = [
    [new THREE.Vector3(1, 0, 0), c.x, "X"],
    [new THREE.Vector3(0, 1, 0), c.y, "Y"],
    [new THREE.Vector3(0, 0, 1), c.z, "Z"],
  ];
  for (const [dir, color, name] of dirs) {
    if (thick) {
      const r = length * 0.025;
      const mat = new THREE.MeshStandardMaterial({ color, roughness: 0.5, transparent: opacity < 1, opacity });
      const shaft = new THREE.Mesh(new THREE.CylinderGeometry(r, r, length * 0.82, 16), mat);
      const head = new THREE.Mesh(new THREE.ConeGeometry(r * 2.6, length * 0.18, 20), mat);
      // Cylinders are built along +Y; turn them onto the axis direction.
      const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
      shaft.quaternion.copy(q);
      head.quaternion.copy(q);
      shaft.position.copy(dir.clone().multiplyScalar(length * 0.41));
      head.position.copy(dir.clone().multiplyScalar(length * 0.91));
      group.add(shaft, head);
    } else {
      const geom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), dir.clone().multiplyScalar(length)]);
      group.add(new THREE.Line(geom, new THREE.LineBasicMaterial({ color, transparent: true, opacity })));
    }
    if (labels) {
      const s = textSprite(name, `#${color.getHexString()}`, length * 0.12);
      s.position.copy(dir.clone().multiplyScalar(length * 1.12));
      group.add(s);
    }
  }
  return group;
}

export function disposeObject(obj) {
  obj.traverse((o) => {
    o.geometry?.dispose();
    const mats = Array.isArray(o.material) ? o.material : o.material ? [o.material] : [];
    for (const m of mats) {
      m.map?.dispose();
      m.dispose();
    }
  });
}
