import { test } from "node:test";
import assert from "node:assert/strict";
import {
  randomQuat,
  EULER_ORDERS,
  axisAngleToQuat,
  eulerToQuat,
  matrixCheck,
  matrixToQuat,
  parsePasted,
  quatDistance,
  quatToAxisAngle,
  quatToEuler,
  quatToMatrix,
  quatToRotvec,
  rotvecToQuat,
} from "../js/rotation-math.js";

// Deterministic pseudo-random so failures reproduce.
let seed = 12345;
const rand = () => ((seed = (seed * 1103515245 + 12345) % 2 ** 31) / 2 ** 31);
const randQuat = () => {
  const u = [rand(), rand(), rand()];
  return [
    Math.sqrt(1 - u[0]) * Math.sin(2 * Math.PI * u[1]),
    Math.sqrt(1 - u[0]) * Math.cos(2 * Math.PI * u[1]),
    Math.sqrt(u[0]) * Math.sin(2 * Math.PI * u[2]),
    Math.sqrt(u[0]) * Math.cos(2 * Math.PI * u[2]),
  ];
};
const near = (a, b, tol = 1e-9) => assert.ok(quatDistance(a, b) < tol, `rotations differ by ${quatDistance(a, b)}`);
const deg = (d) => (d * Math.PI) / 180;

test("matrix round trip", () => {
  for (let n = 0; n < 500; n++) {
    const q = randQuat();
    near(matrixToQuat(quatToMatrix(q)), q);
    const { orthoError, det } = matrixCheck(quatToMatrix(q));
    assert.ok(orthoError < 1e-12 && Math.abs(det - 1) < 1e-12);
  }
});

test("matrix to quat near 180 degrees", () => {
  for (const axis of [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, -2, 3]]) {
    for (const a of [Math.PI, Math.PI - 1e-9, Math.PI - 1e-4]) {
      const q = axisAngleToQuat(axis, a);
      near(matrixToQuat(quatToMatrix(q)), q, 1e-6);
    }
  }
});

test("known values", () => {
  // 90 degrees about Z sends +X to +Y.
  const m = quatToMatrix(axisAngleToQuat([0, 0, 1], deg(90)));
  assert.ok(Math.abs(m[3] - 1) < 1e-12 && Math.abs(m[0]) < 1e-12);
  // Intrinsic ZYX yaw-pitch-roll equals extrinsic XYZ roll-pitch-yaw reversed.
  const a = eulerToQuat([deg(30), deg(20), deg(10)], "ZYX", true);
  const b = eulerToQuat([deg(10), deg(20), deg(30)], "XYZ", false);
  near(a, b);
  // ROS RPY (fixed-axis roll, pitch, yaw) of (0, 0, 90deg) is +90 about Z.
  near(eulerToQuat([0, 0, deg(90)], "XYZ", false), axisAngleToQuat([0, 0, 1], deg(90)));
});

test("euler round trip, all 24 conventions", () => {
  for (const order of EULER_ORDERS) {
    for (const intrinsic of [true, false]) {
      for (let n = 0; n < 300; n++) {
        const q = randQuat();
        const { angles } = quatToEuler(q, order, intrinsic);
        near(eulerToQuat(angles, order, intrinsic), q, 1e-8);
      }
    }
  }
});

test("euler angles come back unchanged away from singularities", () => {
  for (const order of EULER_ORDERS) {
    const proper = order[0] === order[2];
    for (const intrinsic of [true, false]) {
      const input = proper ? [deg(40), deg(70), deg(-25)] : [deg(40), deg(-35), deg(-25)];
      const { angles } = quatToEuler(eulerToQuat(input, order, intrinsic), order, intrinsic);
      angles.forEach((a, i) => assert.ok(Math.abs(a - input[i]) < 1e-9, `${order} ${intrinsic} ${angles} vs ${input}`));
    }
  }
});

test("gimbal lock is flagged and still reconstructs", () => {
  for (const intrinsic of [true, false]) {
    for (const mid of [deg(90), deg(-90)]) {
      const q = eulerToQuat([deg(25), mid, deg(15)], "ZYX", intrinsic);
      const r = quatToEuler(q, "ZYX", intrinsic);
      assert.ok(r.gimbalLock);
      near(eulerToQuat(r.angles, "ZYX", intrinsic), q, 1e-7);
    }
    const q = eulerToQuat([deg(25), 0, deg(15)], "ZXZ", intrinsic);
    const r = quatToEuler(q, "ZXZ", intrinsic);
    assert.ok(r.gimbalLock);
    near(eulerToQuat(r.angles, "ZXZ", intrinsic), q, 1e-7);
  }
});

test("axis-angle and rotation vector", () => {
  for (let n = 0; n < 200; n++) {
    const q = randQuat();
    const { axis, angle } = quatToAxisAngle(q);
    assert.ok(angle >= 0 && angle <= Math.PI + 1e-12);
    near(axisAngleToQuat(axis, angle), q);
    near(rotvecToQuat(quatToRotvec(q)), q);
  }
});

test("paste parsing", () => {
  const tf = parsePasted("- Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]");
  assert.equal(tf.kind, "quat");
  assert.deepEqual(tf.q, [0.707, 0, 0, 0.707]);
  const ros = parsePasted("orientation:\n  x: 0.1\n  y: 0.2\n  z: 0.3\n  w: 0.9");
  assert.deepEqual(ros.q, [0.9, 0.1, 0.2, 0.3]);
  assert.equal(parsePasted("array([[1., 0., 0.],\n [0., 1., 0.],\n [0., 0., 1.]])").kind, "matrix");
  assert.equal(parsePasted("[0.1, 0.2, 0.3, 0.9]").kind, "quat4");
  assert.equal(parsePasted("1e-3 2 3").kind, "vec3");
  assert.equal(parsePasted("1 2").kind, "unknown");
});

test("random rotations are unit quaternions spread over every angle", () => {
  let seed = 7;
  const rand = () => ((seed = (seed * 16807) % 2147483647) - 1) / 2147483646;
  const angles = [];
  for (let i = 0; i < 2000; i++) {
    const q = randomQuat(rand);
    assert.ok(Math.abs(Math.hypot(...q) - 1) < 1e-12);
    assert.ok(q[0] >= 0);
    angles.push(2 * Math.acos(Math.min(1, q[0])));
  }
  // Uniform rotations have mean angle pi/2 + 2/pi, about 2.207 rad.
  const mean = angles.reduce((a, b) => a + b, 0) / angles.length;
  assert.ok(Math.abs(mean - (Math.PI / 2 + 2 / Math.PI)) < 0.05, `mean ${mean}`);
});
