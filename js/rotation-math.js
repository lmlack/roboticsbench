// Rotation conversions. Pure functions, no DOM, so they run under node --test.
//
// Conventions, stated once so nothing downstream has to guess:
// - Rotations are active (they rotate vectors, v' = R v) and right-handed.
// - Quaternions are stored scalar-first internally: [w, x, y, z]. The UI
//   converts to and from scalar-last at the edges.
// - Matrices are row-major arrays of 9 numbers.
// - Euler orders are written as three axis letters. "intrinsic" means each
//   rotation is about the already-rotated (body) axes; "extrinsic" means
//   about the fixed world axes. Intrinsic XYZ and extrinsic ZYX with the
//   angles reversed are the same rotation.

const AXIS = { X: 0, Y: 1, Z: 2 };
export const EULER_ORDERS = [
  "XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX", // Tait-Bryan
  "XYX", "XZX", "YXY", "YZY", "ZXZ", "ZYZ", // proper Euler
];

export const IDENTITY_Q = [1, 0, 0, 0];

export function qNormalize(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (!(n > 1e-12)) return [...IDENTITY_Q];
  // Keep w >= 0 so the same rotation always prints the same way.
  const s = q[0] < 0 ? -1 / n : 1 / n;
  return q.map((v) => v * s + 0);
}

export function qMul(a, b) {
  const [aw, ax, ay, az] = a;
  const [bw, bx, by, bz] = b;
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ];
}

function qAxis(axis, angle) {
  const q = [Math.cos(angle / 2), 0, 0, 0];
  q[1 + axis] = Math.sin(angle / 2);
  return q;
}

export function quatToMatrix(q) {
  const [w, x, y, z] = qNormalize(q);
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
  ];
}

// Shepperd's method: picks the largest diagonal term to divide by, so it
// stays accurate near 180 degrees where the naive trace formula falls apart.
export function matrixToQuat(m) {
  const [m00, m01, m02, m10, m11, m12, m20, m21, m22] = m;
  const tr = m00 + m11 + m22;
  let q;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    q = [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s];
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
    q = [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s];
  } else if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
    q = [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s];
  } else {
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
    q = [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s];
  }
  return qNormalize(q);
}

/** How far a 3x3 is from a proper rotation: max |R^T R - I| and det(R). */
export function matrixCheck(m) {
  let err = 0;
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      let d = 0;
      for (let k = 0; k < 3; k++) d += m[k * 3 + i] * m[k * 3 + j];
      err = Math.max(err, Math.abs(d - (i === j ? 1 : 0)));
    }
  }
  const det =
    m[0] * (m[4] * m[8] - m[5] * m[7]) -
    m[1] * (m[3] * m[8] - m[5] * m[6]) +
    m[2] * (m[3] * m[7] - m[4] * m[6]);
  return { orthoError: err, det };
}

export function eulerToQuat(angles, order, intrinsic) {
  const axes = [...order].map((c) => AXIS[c]);
  const [q0, q1, q2] = axes.map((a, i) => qAxis(a, angles[i]));
  // Intrinsic: R = R0 R1 R2. Extrinsic: the first rotation is applied first
  // about fixed axes, so it ends up rightmost: R = R2 R1 R0.
  return qNormalize(intrinsic ? qMul(qMul(q0, q1), q2) : qMul(qMul(q2, q1), q0));
}

const wrap = (a) => {
  let r = ((a + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
  if (Math.abs(r + Math.PI) < 1e-12) r = Math.PI;
  return r;
};

/**
 * Quaternion to Euler angles for any of the 12 orders, after Bernardes and
 * Viollet, "Quaternion to Euler angles conversion: A direct, general and
 * computationally efficient method" (PLOS ONE, 2022). It works on extrinsic
 * sequences; an intrinsic sequence is the reversed extrinsic one with the
 * angles reversed.
 *
 * Returns { angles, gimbalLock }. At gimbal lock only the sum or difference
 * of the outer angles is defined; the third angle is set to 0.
 */
export function quatToEuler(q, order, intrinsic) {
  const seq = intrinsic ? [...order].reverse().join("") : order;
  let [i, j, k] = [...seq].map((c) => AXIS[c]);
  const symmetric = i === k;
  if (symmetric) k = 3 - i - j;
  const sign = ((i - j) * (j - k) * (k - i)) / 2; // +1 even permutation, -1 odd

  const [w, x, y, z] = qNormalize(q);
  const v = [x, y, z];
  let a, b, c, d;
  if (symmetric) {
    a = w; b = v[i]; c = v[j]; d = v[k] * sign;
  } else {
    a = w - v[j]; b = v[i] + v[k] * sign; c = v[j] + w; d = v[k] * sign - v[i];
  }

  const angles = [0, 0, 0];
  angles[1] = 2 * Math.atan2(Math.hypot(c, d), Math.hypot(a, b));
  const eps = 1e-7;
  const lockLow = Math.abs(angles[1]) <= eps;
  const lockHigh = Math.abs(angles[1] - Math.PI) <= eps;
  const halfSum = Math.atan2(b, a);
  const halfDiff = Math.atan2(d, c);

  if (!lockLow && !lockHigh) {
    angles[0] = halfSum - halfDiff;
    angles[2] = halfSum + halfDiff;
  } else {
    // Only one combination of the outer angles is defined. Put it all in the
    // angle the user reads first (index 0 after the intrinsic reversal below)
    // and zero the other.
    const first = intrinsic ? 2 : 0;
    angles[2 - first] = 0;
    angles[first] = lockLow ? 2 * halfSum : 2 * halfDiff * (intrinsic ? 1 : -1);
  }

  if (!symmetric) {
    angles[2] *= sign;
    angles[1] -= Math.PI / 2;
  }

  let out = angles.map(wrap);
  if (intrinsic) out = out.reverse();
  return { angles: out, gimbalLock: lockLow || lockHigh };
}

/** Axis-angle with the angle in [0, pi]. Identity returns axis +Z, angle 0. */
export function quatToAxisAngle(q) {
  const [w, x, y, z] = qNormalize(q);
  const s = Math.hypot(x, y, z);
  if (s < 1e-12) return { axis: [0, 0, 1], angle: 0 };
  return { axis: [x / s, y / s, z / s], angle: 2 * Math.atan2(s, w) };
}

export function axisAngleToQuat(axis, angle) {
  const n = Math.hypot(...axis);
  if (!(n > 1e-12)) return [...IDENTITY_Q];
  const s = Math.sin(angle / 2) / n;
  return qNormalize([Math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s]);
}

export function rotvecToQuat(rv) {
  const angle = Math.hypot(...rv);
  return angle < 1e-12 ? [...IDENTITY_Q] : axisAngleToQuat(rv, angle);
}

export function quatToRotvec(q) {
  const { axis, angle } = quatToAxisAngle(q);
  return axis.map((a) => a * angle);
}

/** Angle in radians between two rotations, 0 when they match. */
export function quatDistance(a, b) {
  const qa = qNormalize(a);
  const qb = qNormalize(b);
  // atan2 of the relative rotation, not acos of a dot product: acos cannot
  // resolve angles below about 1e-8 because its slope is infinite at 1.
  const [w, x, y, z] = qMul([qa[0], -qa[1], -qa[2], -qa[3]], qb);
  return 2 * Math.atan2(Math.hypot(x, y, z), Math.abs(w));
}

/**
 * Pulls numbers out of whatever was pasted: tf2_echo output, a ROS message,
 * a NumPy or Eigen print, a bare list. Returns a guess with its reason.
 */
export function parsePasted(text) {
  const src = text.trim();
  if (!src) return null;

  // Named fields win: "x: 0.1 y: ... w: ..." from a ROS message or YAML.
  const named = {};
  for (const m of src.matchAll(/\b([wxyz])\s*[:=]\s*(-?\d*\.?\d+(?:e[-+]?\d+)?)/gi)) {
    named[m[1].toLowerCase()] = Number(m[2]);
  }
  if (["w", "x", "y", "z"].every((k) => k in named)) {
    return {
      kind: "quat",
      q: [named.w, named.x, named.y, named.z],
      why: "Named w, x, y, z fields",
    };
  }

  const nums = [...src.matchAll(/-?\d*\.?\d+(?:e[-+]?\d+)?/gi)].map((m) => Number(m[0]));

  // tf2_echo prints "Quaternion [x, y, z, w]", always scalar-last.
  const tf = src.match(/Quaternion\s*\[([^\]]+)\]/i);
  if (tf) {
    const v = tf[1].split(/[,\s]+/).filter(Boolean).map(Number);
    if (v.length === 4 && v.every(Number.isFinite)) {
      return { kind: "quat", q: [v[3], v[0], v[1], v[2]], why: "tf2_echo quaternion, x y z w" };
    }
  }

  if (nums.length === 9) return { kind: "matrix", m: nums, why: "Nine numbers, read as a 3x3 matrix, row by row" };
  if (nums.length === 4) return { kind: "quat4", v: nums, why: "Four numbers, read as a quaternion in the selected order" };
  if (nums.length === 3) return { kind: "vec3", v: nums, why: "Three numbers, read as Euler angles with the current settings" };
  return { kind: "unknown", why: `Found ${nums.length} numbers. Expected 3, 4 or 9.` };
}
