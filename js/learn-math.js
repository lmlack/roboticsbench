// Kinematics for the Learn lessons: a 2-link planar arm with closed-form IK,
// rotation matrices, and a 6-axis arm with forward kinematics, the geometric
// Jacobian and damped least-squares IK. Plain arrays, no three.js, so all of
// it is testable on its own.

/* ---------- 3×3 and 4×4 helpers (row-major nested arrays) ---------- */

export const matMul = (a, b) => a.map((row) => b[0].map((_, j) => row.reduce((s, v, k) => s + v * b[k][j], 0)));
export const transpose = (m) => m[0].map((_, j) => m.map((row) => row[j]));
export const matVec = (m, v) => m.map((row) => row.reduce((s, x, k) => s + x * v[k], 0));
export const identity = (n) => Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
const sub = (a, b) => a.map((v, i) => v - b[i]);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const norm = (v) => Math.hypot(...v);

export function det3(m) {
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
    - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
    + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

/** Determinant of any square matrix, by elimination with partial pivoting. */
export function det(m) {
  const a = m.map((row) => [...row]);
  const n = a.length;
  let d = 1;
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
    if (Math.abs(a[p][c]) < 1e-15) return 0;
    if (p !== c) { [a[p], a[c]] = [a[c], a[p]]; d = -d; }
    d *= a[c][c];
    for (let r = c + 1; r < n; r++) {
      const f = a[r][c] / a[c][c];
      for (let k = c; k < n; k++) a[r][k] -= f * a[c][k];
    }
  }
  return d;
}

/** Solves A x = b for a small square system. */
export function solve(A, b) {
  const n = A.length;
  const a = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
    [a[p], a[c]] = [a[c], a[p]];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = a[r][c] / a[c][c];
      for (let k = c; k <= n; k++) a[r][k] -= f * a[c][k];
    }
  }
  return a.map((row, i) => row[n] / row[i]);
}

/* ---------- Rotations ---------- */

export const rotX = (t) => [[1, 0, 0], [0, Math.cos(t), -Math.sin(t)], [0, Math.sin(t), Math.cos(t)]];
export const rotY = (t) => [[Math.cos(t), 0, Math.sin(t)], [0, 1, 0], [-Math.sin(t), 0, Math.cos(t)]];
export const rotZ = (t) => [[Math.cos(t), -Math.sin(t), 0], [Math.sin(t), Math.cos(t), 0], [0, 0, 1]];

/** Rotation by `angle` about a unit `axis` (Rodrigues). */
export function rotAxis([x, y, z], angle) {
  const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
  return [
    [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
    [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
    [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
  ];
}

/** Largest entry of RᵀR − I: zero for a true rotation. */
export function orthoError(R) {
  const P = matMul(transpose(R), R);
  let e = 0;
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) e = Math.max(e, Math.abs(P[i][j] - (i === j ? 1 : 0)));
  return e;
}

/** Angle in radians between two rotation matrices: the size of R_aᵀ R_b. */
export function rotationAngleBetween(Ra, Rb) {
  const M = matMul(transpose(Ra), Rb);
  const c = (M[0][0] + M[1][1] + M[2][2] - 1) / 2;
  return Math.acos(Math.min(1, Math.max(-1, c)));
}

/** 2D rotation and homogeneous transform, for the planar lessons. */
export const rot2 = (t) => [[Math.cos(t), -Math.sin(t)], [Math.sin(t), Math.cos(t)]];
export const tf2 = (t, x, y) => [[Math.cos(t), -Math.sin(t), x], [Math.sin(t), Math.cos(t), y], [0, 0, 1]];

/** 4×4 homogeneous transform from a 3×3 rotation and a translation. */
export const tf3 = (R, p) => [[...R[0], p[0]], [...R[1], p[1]], [...R[2], p[2]], [0, 0, 0, 1]];
export const tfRot = (T) => T.slice(0, 3).map((row) => row.slice(0, 3));
export const tfPos = (T) => [T[0][3], T[1][3], T[2][3]];

/* ---------- 2-link planar arm ---------- */

/** Joint, elbow and tip positions for link lengths l1, l2 and joint angles t1, t2. */
export function fk2(l1, l2, t1, t2) {
  const elbow = [l1 * Math.cos(t1), l1 * Math.sin(t1)];
  const tip = [elbow[0] + l2 * Math.cos(t1 + t2), elbow[1] + l2 * Math.sin(t1 + t2)];
  return { elbow, tip, phi: t1 + t2 };
}

/**
 * Closed-form IK by the law of cosines. Returns both solutions. Outside the
 * reachable ring, the arm is aimed at the target as close as it can get and
 * `reachable` is false.
 */
export function ik2(l1, l2, x, y) {
  const r2 = x * x + y * y;
  const raw = (r2 - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  const c2 = Math.min(1, Math.max(-1, raw));
  const reachable = raw >= -1 - 1e-9 && raw <= 1 + 1e-9;
  const solve2 = (t2) => ({ t1: Math.atan2(y, x) - Math.atan2(l2 * Math.sin(t2), l1 + l2 * Math.cos(t2)), t2 });
  // With y up, a negative elbow angle puts the elbow on the left of the line
  // from base to target: above it when the target is to the right.
  const t2 = Math.acos(c2);
  return { reachable, cosT2: raw, up: solve2(-t2), down: solve2(t2) };
}

/* ---------- 6-axis arm ---------- */

/**
 * Each joint sits at `offset` from the previous joint's frame and turns about
 * `axis` in its own frame. Zero pose is straight up, which is also a
 * singularity: joints 4 and 6 line up.
 */
export const ARM6 = {
  joints: [
    { name: "base", axis: [0, 0, 1], offset: [0, 0, 0], limit: Math.PI },
    { name: "shoulder", axis: [0, 1, 0], offset: [0, 0, 0.3], limit: 2.2 },
    { name: "elbow", axis: [0, 1, 0], offset: [0, 0, 0.4], limit: 2.6 },
    { name: "forearm roll", axis: [0, 0, 1], offset: [0, 0, 0.15], limit: Math.PI },
    { name: "wrist pitch", axis: [0, 1, 0], offset: [0, 0, 0.2], limit: 2.2 },
    { name: "flange roll", axis: [0, 0, 1], offset: [0, 0, 0.08], limit: Math.PI },
  ],
  tool: [0, 0, 0.07],
  home: [0, 0.5, 1.0, 0, 0.6, 0],
};

/**
 * World frames of every joint (after it turns) and of the tool.
 * T_i = T_{i−1} · Trans(offset_i) · Rot(axis_i, q_i).
 */
export function fk6(q, arm = ARM6) {
  let T = identity(4);
  const frames = [];
  arm.joints.forEach((j, i) => {
    T = matMul(T, tf3(identity(3), j.offset));
    T = matMul(T, tf3(rotAxis(j.axis, q[i]), [0, 0, 0]));
    frames.push(T);
  });
  const tool = matMul(T, tf3(identity(3), arm.tool));
  return { frames, tool };
}

/**
 * Geometric Jacobian, 6 × n: rows vx vy vz wx wy wz. Column i is
 * [z_i × (p_tool − p_i); z_i], with z_i the joint axis in world coordinates.
 */
export function jacobian6(q, arm = ARM6) {
  const { frames, tool } = fk6(q, arm);
  const pe = tfPos(tool);
  const cols = frames.map((T, i) => {
    const z = matVec(tfRot(T), arm.joints[i].axis);
    return [...cross(z, sub(pe, tfPos(T))), ...z];
  });
  return transpose(cols);
}

/** Yoshikawa manipulability, sqrt(det(J Jᵀ)). Zero at a singularity. */
export function manipulability(J) {
  return Math.sqrt(Math.max(0, det(matMul(J, transpose(J)))));
}

/**
 * One damped least-squares step toward a target position:
 * Δq = Jᵥᵀ (Jᵥ Jᵥᵀ + λ² I)⁻¹ e. The damping keeps the step finite near a
 * singularity, where a plain inverse would blow up.
 */
export function ikStep(q, target, { lambda = 0.08, maxStep = 0.25, rest = null, restGain = 0.1, arm = ARM6 } = {}) {
  const J = jacobian6(q, arm).slice(0, 3);
  const e = sub(target, tfPos(fk6(q, arm).tool));
  const A = matMul(J, transpose(J)).map((row, i) => row.map((v, j) => v + (i === j ? lambda * lambda : 0)));
  const y = solve(A, e);
  let dq = matVec(transpose(J), y);
  // Six joints for three position numbers leaves three to spare. Spend them
  // drifting toward a rest pose, projected so the drift never moves the tip:
  // dq += (I − J⁺J)·k·(rest − q). The projector uses a nearly undamped J⁺:
  // with the full λ it is only roughly a null-space projector, and the pull
  // leaks into the tip as a small error that never goes away.
  if (rest) {
    const A0 = matMul(J, transpose(J)).map((row, i) => row.map((v, j) => v + (i === j ? 1e-6 : 0)));
    const Ainv = transpose([0, 1, 2].map((k) => solve(A0, [0, 1, 2].map((r) => (r === k ? 1 : 0)))));
    const Jpinv = matMul(transpose(J), Ainv);
    const N = matMul(Jpinv, J).map((row, i) => row.map((v, j) => (i === j ? 1 : 0) - v));
    const pull = matVec(N, q.map((v, i) => restGain * (rest[i] - v)));
    dq = dq.map((v, i) => v + pull[i]);
  }
  const size = norm(dq);
  if (size > maxStep) dq = dq.map((v) => (v * maxStep) / size);
  const next = q.map((v, i) => {
    const lim = arm.joints[i].limit;
    return Math.min(lim, Math.max(-lim, v + dq[i]));
  });
  return { q: next, error: norm(e) };
}

/** Iterates ikStep until the tip is within `tol` or `maxIter` runs out. */
export function solveIk(q0, target, { tol = 1e-4, maxIter = 200, ...opts } = {}) {
  let q = [...q0];
  const errors = [];
  for (let k = 0; k < maxIter; k++) {
    const step = ikStep(q, target, opts);
    errors.push(step.error);
    if (step.error < tol) return { q, errors, converged: true };
    q = step.q;
  }
  const error = norm(sub(target, tfPos(fk6(q, opts.arm).tool)));
  errors.push(error);
  return { q, errors, converged: error < tol };
}

/** Farthest the tool can be from the shoulder: every link straight. */
export function reach6(arm = ARM6) {
  return arm.joints.slice(2).reduce((s, j) => s + norm(j.offset), 0) + norm(arm.tool);
}
