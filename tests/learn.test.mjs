import { test } from "node:test";
import assert from "node:assert/strict";
import {
  ARM6,
  det,
  det3,
  fk2,
  fk6,
  ik2,
  jacobian6,
  manipulability,
  matMul,
  orthoError,
  reach6,
  rotAxis,
  rotX,
  rotY,
  rotZ,
  rotationAngleBetween,
  solveIk,
  tfPos,
} from "../js/learn-math.js";

// Deterministic pseudo-random so failures reproduce.
let seed = 4242;
const rand = () => ((seed = (seed * 1103515245 + 12345) % 2 ** 31) / 2 ** 31);
const close = (a, b, tol, what) => assert.ok(Math.abs(a - b) <= tol, `${what}: got ${a}, wanted ${b} ± ${tol}`);

test("rotation matrices are in SO(3): orthonormal, det +1", () => {
  for (let k = 0; k < 50; k++) {
    const R = matMul(matMul(rotZ(rand() * 6), rotY(rand() * 6)), rotX(rand() * 6));
    assert.ok(orthoError(R) < 1e-12);
    close(det3(R), 1, 1e-12, "det");
  }
});

test("rotating about a coordinate axis matches the named matrices", () => {
  const t = 0.7;
  assert.ok(rotationAngleBetween(rotAxis([1, 0, 0], t), rotX(t)) < 1e-9);
  assert.ok(rotationAngleBetween(rotAxis([0, 1, 0], t), rotY(t)) < 1e-9);
  assert.ok(rotationAngleBetween(rotAxis([0, 0, 1], t), rotZ(t)) < 1e-9);
});

test("rotation order matters: 90° about X then Z is not Z then X", () => {
  const q = Math.PI / 2;
  const a = matMul(rotZ(q), rotX(q));
  const b = matMul(rotX(q), rotZ(q));
  close(rotationAngleBetween(a, b), (2 * Math.PI) / 3, 1e-9, "angle between the two orders");
});

test("det handles larger matrices and singular ones", () => {
  close(det([[2, 0, 0, 0], [0, 3, 0, 0], [0, 0, 4, 0], [0, 0, 0, 5]]), 120, 1e-12, "diagonal");
  close(det([[1, 2], [2, 4]]), 0, 1e-12, "singular");
  close(det([[0, 1], [1, 0]]), -1, 1e-12, "swap");
});

test("2-link IK puts the tip back on the target, both elbows", () => {
  const l1 = 1, l2 = 0.7;
  for (let k = 0; k < 200; k++) {
    const t1 = (rand() - 0.5) * 6, t2 = (rand() - 0.5) * 6;
    const { tip } = fk2(l1, l2, t1, t2);
    const sol = ik2(l1, l2, ...tip);
    assert.ok(sol.reachable);
    for (const s of [sol.up, sol.down]) {
      const back = fk2(l1, l2, s.t1, s.t2).tip;
      close(back[0], tip[0], 1e-9, "x");
      close(back[1], tip[1], 1e-9, "y");
    }
  }
});

test("2-link IK: elbow up has the elbow above the line to a target on the right", () => {
  const { up, down } = ik2(1, 1, 1.4, 0);
  assert.ok(fk2(1, 1, up.t1, up.t2).elbow[1] > 0);
  assert.ok(fk2(1, 1, down.t1, down.t2).elbow[1] < 0);
});

test("2-link IK outside the ring says so and still aims at the target", () => {
  const sol = ik2(1, 0.7, 3, 0);
  assert.equal(sol.reachable, false);
  const tip = fk2(1, 0.7, sol.up.t1, sol.up.t2).tip;
  close(tip[0], 1.7, 1e-9, "stretched toward the target");
  assert.equal(ik2(1, 0.7, 0.1, 0).reachable, false, "inside the hole is unreachable too");
});

test("6-axis FK at zero is straight up", () => {
  const p = tfPos(fk6([0, 0, 0, 0, 0, 0]).tool);
  close(p[0], 0, 1e-12, "x");
  close(p[1], 0, 1e-12, "y");
  close(p[2], 0.3 + reach6(), 1e-12, "z");
});

test("the Jacobian's velocity rows match finite differences of FK", () => {
  for (let k = 0; k < 20; k++) {
    const q = ARM6.home.map(() => (rand() - 0.5) * 3);
    const J = jacobian6(q);
    const p0 = tfPos(fk6(q).tool);
    const h = 1e-6;
    for (let i = 0; i < 6; i++) {
      const qh = [...q];
      qh[i] += h;
      const p1 = tfPos(fk6(qh).tool);
      for (let r = 0; r < 3; r++) close(J[r][i], (p1[r] - p0[r]) / h, 1e-4, `J[${r}][${i}]`);
    }
  }
});

test("manipulability is zero at the straight-up singularity and positive away from it", () => {
  close(manipulability(jacobian6([0, 0, 0, 0, 0, 0])), 0, 1e-9, "straight up");
  assert.ok(manipulability(jacobian6(ARM6.home)) > 1e-3);
});

test("damped least-squares IK reaches reachable targets from home", () => {
  let solved = 0;
  for (let k = 0; k < 40; k++) {
    const q = ARM6.home.map((v) => v + (rand() - 0.5) * 1.6);
    const target = tfPos(fk6(q).tool);
    const out = solveIk(ARM6.home, target, { maxIter: 400 });
    if (out.converged) solved++;
    const p = tfPos(fk6(out.q).tool);
    if (out.converged) close(Math.hypot(p[0] - target[0], p[1] - target[1], p[2] - target[2]), 0, 1e-4, "tip error");
  }
  assert.ok(solved >= 36, `only ${solved} of 40 reachable targets solved`);
});

test("a rest pose keeps IK solutions closer to home without costing accuracy", () => {
  let tidier = 0;
  for (let k = 0; k < 20; k++) {
    const target = tfPos(fk6(ARM6.home.map((v) => v + (rand() - 0.5) * 1.6)).tool);
    const plain = solveIk(ARM6.home, target, { maxIter: 400 });
    const rested = solveIk(ARM6.home, target, { maxIter: 400, rest: ARM6.home });
    assert.ok(rested.converged, "rest pose stopped IK from converging");
    const dist = (q) => Math.hypot(...q.map((v, i) => v - ARM6.home[i]));
    if (dist(rested.q) <= dist(plain.q) + 1e-6) tidier++;
  }
  assert.ok(tidier >= 16, `rest pose helped on only ${tidier} of 20`);
});

test("IK on an out-of-reach target stops short, stretched toward it", () => {
  const out = solveIk(ARM6.home, [2, 0, 0.3], { maxIter: 300 });
  assert.equal(out.converged, false);
  const p = tfPos(fk6(out.q).tool);
  close(Math.hypot(p[0], p[1], p[2] - 0.3), reach6(), 0.02, "fully stretched");
});
