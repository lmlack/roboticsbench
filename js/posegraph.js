// Graph SLAM: the robot's path as a chain of keyframe poses, the landmarks it
// has seen, and an edge for every piece of evidence tying them together.
// Odometry links each pose to the next; a sighting links a pose to a
// landmark. Every so often the whole graph is re-solved by least squares, so
// seeing an old landmark again (a loop closure) pulls the entire path back
// into shape, not just the robot's current pose. Pure JS, no DOM.

import { FLOOR, GATE, NEW_LANDMARK, ellipseOf, noiseVar, wrap } from "./localization-sim.js";

export const KEYFRAME = { dist: 0.4, turn: 0.4 }; // m, rad: a new pose once it has moved this far
export const MAX_POSES = 240; // past this, every other pose in the older half is merged away
export const CLOSURE_GAP = 8; // s out of sight before a sighting counts as a loop closure
const MAX_ITERS = 6;

/** a ⊕ b: pose b, given relative to pose a, in a's parent frame. */
export function compose(a, b) {
  const c = Math.cos(a.th), s = Math.sin(a.th);
  return { x: a.x + c * b.x - s * b.y, y: a.y + s * b.x + c * b.y, th: wrap(a.th + b.th) };
}

/** Jacobians of a ⊕ b with respect to a and to b. */
function composeJ(a, b) {
  const c = Math.cos(a.th), s = Math.sin(a.th);
  return [
    [[1, 0, -s * b.x - c * b.y], [0, 1, c * b.x - s * b.y], [0, 0, 1]],
    [[c, -s, 0], [s, c, 0], [0, 0, 1]],
  ];
}

const mul = (A, B) => A.map((row) => B[0].map((_, j) => row.reduce((s, a, k) => s + a * B[k][j], 0)));
const tr = (A) => A[0].map((_, j) => A.map((row) => row[j]));
const add = (A, B) => A.map((row, i) => row.map((v, j) => v + B[i][j]));
const sandwich = (J, P) => mul(mul(J, P), tr(J));

/** Inverse of a symmetric positive definite matrix by Gauss-Jordan. Small sizes only. */
export function invSPD(A) {
  const n = A.length;
  const M = A.map((row, i) => [...row, ...Array.from({ length: n }, (_, j) => (i === j ? 1 : 0))]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    const d = M[c][c];
    for (let j = 0; j < 2 * n; j++) M[c][j] /= d;
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c];
      if (f) for (let j = 0; j < 2 * n; j++) M[r][j] -= f * M[c][j];
    }
  }
  return M.map((row) => row.slice(n));
}

/** Range-bearing error from a pose to a landmark, and its Jacobians. */
function sightError(p, l, r, b) {
  const dx = l.x - p.x, dy = l.y - p.y;
  const q = Math.max(dx * dx + dy * dy, 1e-9);
  const sq = Math.sqrt(q);
  return {
    e: [sq - r, wrap(Math.atan2(dy, dx) - p.th - b)],
    Jp: [[-dx / sq, -dy / sq, 0], [dy / q, -dx / q, -1]],
    Jl: [[dx / sq, dy / sq], [-dy / q, dx / q]],
  };
}

/**
 * The normal equations of the graph, H δ = −g, stored the way they are
 * shaped: poses first, landmarks last. Odometry only links neighboring poses,
 * so the pose block is banded (a pose's three numbers touch at most the next
 * pose's three). Sightings fill the border between poses and landmarks. That
 * arrow shape solves in about poses × landmarks² work instead of the cube of
 * everything.
 */
const BW = 6; // stored band width: each row keeps H[i][i-5 .. i]

export class ArrowSystem {
  constructor(n, m) {
    this.n = n;
    this.m = m;
    this.B = new Float64Array(n * BW); // B[i*BW + k] = H[i][i-k]
    this.C = new Float64Array(n * m); // C[i*m + a] = H[i][n + a]
    this.D = new Float64Array(m * m);
    this.g = new Float64Array(n + m);
  }

  /** Add v to H[a][b] (and H[b][a]); a and b are indices into the full system. */
  addH(a, b, v) {
    const { n, m } = this;
    if (a < b) [a, b] = [b, a];
    if (a < n) this.B[a * BW + (a - b)] += v;
    else if (b < n) this.C[b * m + (a - n)] += v;
    else {
      this.D[(a - n) * m + (b - n)] += v;
      if (a !== b) this.D[(b - n) * m + (a - n)] += v;
    }
  }

  /** Add an edge's Jᵀ Ω J and Jᵀ Ω e: idx are the variables J's columns belong to. */
  addEdge(idx, J, info, e) {
    const k = idx.length, d = e.length;
    // W = Ω J  (d × k)
    const W = info.map((row) => Array.from({ length: k }, (_, c) => row.reduce((s, o, t) => s + o * J[t][c], 0)));
    for (let a = 0; a < k; a++) {
      let ga = 0;
      for (let t = 0; t < d; t++) ga += W[t][a] * e[t];
      this.g[idx[a]] += ga;
      for (let b = 0; b <= a; b++) {
        let h = 0;
        for (let t = 0; t < d; t++) h += J[t][a] * W[t][b];
        this.addH(idx[a], idx[b], h);
      }
    }
  }

  /** Banded Cholesky of the pose block, in place. False if not positive definite. */
  factor() {
    const { n, B } = this;
    for (let i = 0; i < n; i++) {
      for (let j = Math.max(0, i - BW + 1); j <= i; j++) {
        let s = B[i * BW + (i - j)];
        for (let k = Math.max(0, i - BW + 1); k < j; k++) s -= B[i * BW + (i - k)] * B[j * BW + (j - k)];
        if (i === j) {
          if (!(s > 0)) return false;
          B[i * BW] = Math.sqrt(s);
        } else {
          B[i * BW + (i - j)] = s / B[j * BW];
        }
      }
    }
    return true;
  }

  /** Solve (L Lᵀ) x = b in place with the factored pose block. */
  bandSolve(x) {
    const { n, B } = this;
    for (let i = 0; i < n; i++) {
      let s = x[i];
      for (let k = Math.max(0, i - BW + 1); k < i; k++) s -= B[i * BW + (i - k)] * x[k];
      x[i] = s / B[i * BW];
    }
    for (let i = n - 1; i >= 0; i--) {
      let s = x[i];
      for (let k = i + 1; k < Math.min(n, i + BW); k++) s -= B[k * BW + (k - i)] * x[k];
      x[i] = s / B[i * BW];
    }
    return x;
  }

  /**
   * Solve H δ = −g by eliminating the poses: S = D − Cᵀ B⁻¹ C is the
   * landmarks' own small system. Returns δ, plus S⁻¹ (the landmarks'
   * covariance) and Y = B⁻¹ C for the marginals, or null if H is singular.
   */
  solve() {
    const { n, m, C, D, g } = this;
    if (!this.factor()) return null;
    const Y = new Float64Array(n * m);
    const col = new Float64Array(n);
    for (let a = 0; a < m; a++) {
      for (let i = 0; i < n; i++) col[i] = C[i * m + a];
      this.bandSolve(col);
      for (let i = 0; i < n; i++) Y[i * m + a] = col[i];
    }
    const S = Array.from({ length: m }, (_, a) => Array.from({ length: m }, (_, b) => {
      let s = D[a * m + b];
      for (let i = 0; i < n; i++) s -= C[i * m + a] * Y[i * m + b];
      return s;
    }));
    const r1 = new Float64Array(n);
    for (let i = 0; i < n; i++) r1[i] = -g[i];
    this.bandSolve(r1); // B⁻¹ r1
    const r2 = Array.from({ length: m }, (_, a) => {
      let s = -g[n + a];
      for (let i = 0; i < n; i++) s -= C[i * m + a] * r1[i];
      return s;
    });
    const Si = m ? invSPD(S) : [];
    if (Si.some((row) => row.some((v) => !Number.isFinite(v)))) return null;
    const dl = Si.map((row) => row.reduce((s, v, b) => s + v * r2[b], 0));
    const dp = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      let s = r1[i];
      for (let a = 0; a < m; a++) s -= Y[i * m + a] * dl[a];
      dp[i] = s;
    }
    return { dp, dl, Si, Y };
  }

  /** 3 × 3 covariance of the pose starting at variable i, after solve(). */
  poseCov(i, Y, Si) {
    const { n, m } = this;
    const out = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let c = 0; c < 3; c++) {
      const x = new Float64Array(n);
      x[i + c] = 1;
      this.bandSolve(x);
      for (let r = 0; r < 3; r++) out[r][c] = x[i + r];
    }
    // Plus the part that comes through the landmarks: Y_i S⁻¹ Y_iᵀ.
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        let s = 0;
        for (let a = 0; a < m; a++) for (let b = 0; b < m; b++) s += Y[(i + r) * m + a] * Si[a][b] * Y[(i + c) * m + b];
        out[r][c] += s;
      }
    }
    return out;
  }
}

export class PoseGraphSLAM {
  constructor() {
    this.init({ x: 2, y: 2, th: 0 }, 0.1, 0.1, 0.05);
  }

  /** Start at a pose. keepMap holds on to the landmarks, each pinned where it was with its own covariance. */
  init(pose, sx, sy, sth, keepMap = false) {
    const P0 = [[sx * sx, 0, 0], [0, sy * sy, 0], [0, 0, sth * sth]];
    this.poses = [{ ...pose }];
    this.prior = { mean: { ...pose }, info: invSPD(P0) };
    if (keepMap && this.landmarks?.length) {
      for (const l of this.landmarks) l.prior = { x: l.x, y: l.y, info: invSPD(l.P) };
    } else {
      this.landmarks = []; // { x, y, P, j, lastSeen, prior }
    }
    this.odo = []; // { z: {x, y, th}, Sigma, info } from pose k to k + 1
    this.sights = []; // { i, l, r, b, info, closure, t }
    this.rel = { x: 0, y: 0, th: 0 };
    this.relP = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    this.lastP = P0;
    this.P = P0;
    this.mu = [pose.x, pose.y, pose.th];
    this.t = 0;
    this.fresh = true; // the first pose has not had its sightings attached yet
    this.closures = [];
    this.before = null; // the chain just before the last loop closure, for drawing
    this.last = null;
    this.rejected = 0;
    this.added = 0;
    this.wrong = 0;
    this.iters = 0;
    this.solveMs = 0;
    this.cost = 0;
  }

  /** Same midpoint motion model as the EKF, kept relative to the newest pose. */
  predict(v, w, dt, k) {
    const r = this.rel;
    const tm = r.th + (w * dt) / 2;
    const c = Math.cos(tm), s = Math.sin(tm);
    const G = [[1, 0, -v * dt * s], [0, 1, v * dt * c], [0, 0, 1]];
    const V = [[dt * c, (-v * dt * dt * s) / 2], [dt * s, (v * dt * dt * c) / 2], [0, dt]];
    const [vv, vw] = noiseVar(v, w, k);
    const M = [[vv + FLOOR.v * FLOOR.v, 0], [0, vw + FLOOR.w * FLOOR.w]];
    this.relP = add(sandwich(G, this.relP), sandwich(V, M));
    this.rel = { x: r.x + v * dt * c, y: r.y + v * dt * s, th: wrap(r.th + w * dt) };
    this.t += dt;
    this.refreshCurrent();
  }

  /** The current pose: the newest keyframe plus the odometry since, with both uncertainties. */
  refreshCurrent() {
    const a = this.poses[this.poses.length - 1];
    const p = compose(a, this.rel);
    const [Ja, Jb] = composeJ(a, this.rel);
    this.P = add(sandwich(Ja, this.lastP), sandwich(Jb, this.relP));
    this.mu = [p.x, p.y, p.th];
  }

  estimate() {
    return { x: this.mu[0], y: this.mu[1], th: this.mu[2] };
  }

  ellipse() {
    return ellipseOf(this.P);
  }

  map() {
    return this.landmarks.map((l) => ({ j: l.j, x: l.x, y: l.y, ...ellipseOf(l.P) }));
  }

  /**
   * Sightings are only used at keyframes: once the robot has moved
   * KEYFRAME.dist or turned KEYFRAME.turn, the current pose becomes a node,
   * the odometry since the last one an edge, and each sighting an edge to its
   * landmark. Then the graph is re-solved.
   */
  update(z, sr, sb, known = true) {
    this.added = 0;
    const moved = Math.hypot(this.rel.x, this.rel.y) >= KEYFRAME.dist || Math.abs(this.rel.th) >= KEYFRAME.turn;
    if (!moved && !this.fresh) return false;
    if (!this.fresh) {
      const a = this.poses[this.poses.length - 1];
      const Sigma = this.relP.map((row, i) => row.map((v, j) => v + (i === j ? 1e-8 : 0)));
      this.odo.push({ z: { ...this.rel }, Sigma, info: invSPD(Sigma) });
      this.poses.push(compose(a, this.rel));
      this.rel = { x: 0, y: 0, th: 0 };
      this.relP = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
      this.lastP = this.P;
    }
    this.fresh = false;
    const i = this.poses.length - 1;
    const pose = this.poses[i];
    const R = [[sr * sr, 0], [0, sb * sb]];
    const info = [[1 / (sr * sr), 0], [0, 1 / (sb * sb)]];
    this.rejected = 0;
    this.wrong = 0;
    let closed = false;
    for (const m of z) {
      const pick = this.associate(pose, m, R, known);
      this.last = pick.last;
      if (pick.l === -2) {
        this.rejected++;
        continue;
      }
      let l = pick.l;
      if (l < 0) {
        // Placed where the sighting says, with the robot's uncertainty plus the sensor's.
        const a = pose.th + m.b, c = Math.cos(a), sn = Math.sin(a);
        const Gr = [[1, 0, -m.r * sn], [0, 1, m.r * c]];
        const Gz = [[c, -m.r * sn], [sn, m.r * c]];
        const P = add(sandwich(Gr, this.lastP), sandwich(Gz, R));
        this.landmarks.push({ x: pose.x + m.r * c, y: pose.y + m.r * sn, P, j: m.j, lastSeen: this.t, prior: null });
        l = this.landmarks.length - 1;
        this.added++;
      } else if (this.landmarks[l].j !== m.j) {
        this.wrong++;
      }
      const lm = this.landmarks[l];
      const closure = pick.l >= 0 && this.t - lm.lastSeen > CLOSURE_GAP;
      if (closure) {
        closed = true;
        this.closures.push({ t: this.t, l, i });
      }
      lm.lastSeen = this.t;
      this.sights.push({ i, l, r: m.r, b: m.b, info, closure, t: this.t });
    }
    if (this.poses.length > MAX_POSES) this.thin();
    if (closed) this.before = { t: this.t, poses: this.poses.map((p) => ({ ...p })) };
    this.optimize();
    return true;
  }

  /**
   * Which landmark a sighting is. Known: the sighting says so, and only the
   * gate is checked. Look-alike: the mapped landmark with the smallest
   * Mahalanobis distance, d² = νᵀ S⁻¹ ν. Returns l ≥ 0 to match, −1 for a
   * new landmark, −2 to drop it.
   */
  associate(pose, m, R, known) {
    const cand = known ? this.landmarks.map((l, k) => (l.j === m.j ? k : -1)).filter((k) => k >= 0) : this.landmarks.map((_, k) => k);
    let best = -1, bestD = Infinity, bestNu = null;
    for (const k of cand) {
      const lm = this.landmarks[k];
      const { e, Jp, Jl } = sightError(pose, lm, m.r, m.b);
      const S = add(add(sandwich(Jp, this.lastP), sandwich(Jl, lm.P)), R);
      const Si = invSPD(S);
      const nu = [-e[0], -e[1]];
      const d2 = nu[0] * (Si[0][0] * nu[0] + Si[0][1] * nu[1]) + nu[1] * (Si[1][0] * nu[0] + Si[1][1] * nu[1]);
      if (d2 < bestD) {
        bestD = d2;
        best = k;
        bestNu = nu;
      }
    }
    const last = { j: m.j, l: best, nu: bestNu, d2: bestD, accepted: bestD <= GATE };
    if (best >= 0 && bestD <= GATE) return { l: best, last };
    if (best < 0 || (!known ? bestD > NEW_LANDMARK : false)) return { l: -1, last: { ...last, accepted: true, isNew: true } };
    return { l: -2, last };
  }

  /** Merge every other pose in the older half into its neighbors, composing their odometry. */
  thin() {
    const half = Math.floor(this.poses.length / 2);
    const keep = this.poses.map((_, k) => k === 0 || k >= half || k % 2 === 0);
    const newIndex = [];
    const poses = [], odo = [];
    let pending = null; // odometry from the last kept pose up to here
    this.poses.forEach((p, k) => {
      if (k > 0) {
        const e = this.odo[k - 1];
        if (!pending) pending = { z: { ...e.z }, Sigma: e.Sigma };
        else {
          const [J1, J2] = composeJ(pending.z, e.z);
          pending = { z: compose(pending.z, e.z), Sigma: add(sandwich(J1, pending.Sigma), sandwich(J2, e.Sigma)) };
        }
      }
      if (keep[k]) {
        if (pending) odo.push({ ...pending, info: invSPD(pending.Sigma) });
        pending = null;
        newIndex[k] = poses.length;
        poses.push(p);
      } else {
        newIndex[k] = -1;
      }
    });
    this.poses = poses;
    this.odo = odo;
    this.sights = this.sights.filter((s) => newIndex[s.i] >= 0).map((s) => ({ ...s, i: newIndex[s.i] }));
    this.closures = this.closures.filter((c) => newIndex[c.i] >= 0).map((c) => ({ ...c, i: newIndex[c.i] }));
    // A landmark whose every sighting just went keeps its place as a prior.
    const seen = new Set(this.sights.map((s) => s.l));
    this.landmarks.forEach((l, k) => {
      if (!seen.has(k) && !l.prior) l.prior = { x: l.x, y: l.y, info: invSPD(l.P) };
    });
  }

  /** Total weighted squared error, Σ eᵀ Ω e, over every edge. */
  totalCost(poses = this.poses, landmarks = this.landmarks) {
    let c = 0;
    const quad = (e, I) => e.reduce((s, ei, a) => s + ei * e.reduce((t, ej, b) => t + I[a][b] * ej, 0), 0);
    const pe = [poses[0].x - this.prior.mean.x, poses[0].y - this.prior.mean.y, wrap(poses[0].th - this.prior.mean.th)];
    c += quad(pe, this.prior.info);
    this.odo.forEach((o, k) => (c += quad(this.odoError(poses[k], poses[k + 1], o.z).e, o.info)));
    for (const s of this.sights) c += quad(sightError(poses[s.i], landmarks[s.l], s.r, s.b).e, s.info);
    for (const l of landmarks) if (l.prior) c += quad([l.x - l.prior.x, l.y - l.prior.y], l.prior.info);
    return c;
  }

  /** How far pose b sits from where odometry z says it should be, seen from pose a. */
  odoError(a, b, z) {
    const c = Math.cos(a.th), s = Math.sin(a.th);
    const dx = b.x - a.x, dy = b.y - a.y;
    const e = [c * dx + s * dy - z.x, -s * dx + c * dy - z.y, wrap(b.th - a.th - z.th)];
    // ∂e/∂a and ∂e/∂b
    const Ja = [[-c, -s, -s * dx + c * dy], [s, -c, -c * dx - s * dy], [0, 0, -1]];
    const Jb = [[c, s, 0], [-s, c, 0], [0, 0, 1]];
    return { e, Ja, Jb };
  }

  /** Gauss-Newton with Levenberg-Marquardt damping, over the whole graph. */
  optimize(maxIters = MAX_ITERS) {
    const t0 = typeof performance !== "undefined" ? performance.now() : 0;
    const K = this.poses.length, L = this.landmarks.length;
    const n = 3 * K, m = 2 * L;
    let lambda = 1e-6;
    let cost = this.totalCost();
    let sol = null, sys = null;
    this.iters = 0;
    for (let it = 0; it < maxIters; it++) {
      sys = new ArrowSystem(n, m);
      const pe = [this.poses[0].x - this.prior.mean.x, this.poses[0].y - this.prior.mean.y, wrap(this.poses[0].th - this.prior.mean.th)];
      sys.addEdge([0, 1, 2], [[1, 0, 0], [0, 1, 0], [0, 0, 1]], this.prior.info, pe);
      this.odo.forEach((o, k) => {
        const { e, Ja, Jb } = this.odoError(this.poses[k], this.poses[k + 1], o.z);
        const idx = [3 * k, 3 * k + 1, 3 * k + 2, 3 * k + 3, 3 * k + 4, 3 * k + 5];
        sys.addEdge(idx, Ja.map((row, r) => [...row, ...Jb[r]]), o.info, e);
      });
      for (const s of this.sights) {
        const { e, Jp, Jl } = sightError(this.poses[s.i], this.landmarks[s.l], s.r, s.b);
        const idx = [3 * s.i, 3 * s.i + 1, 3 * s.i + 2, n + 2 * s.l, n + 2 * s.l + 1];
        sys.addEdge(idx, Jp.map((row, r) => [...row, ...Jl[r]]), s.info, e);
      }
      this.landmarks.forEach((l, k) => {
        if (l.prior) sys.addEdge([n + 2 * k, n + 2 * k + 1], [[1, 0], [0, 1]], l.prior.info, [l.x - l.prior.x, l.y - l.prior.y]);
      });
      // Damping on the diagonal: a small λ is Gauss-Newton, a large one a short gradient step.
      for (let v = 0; v < n + m; v++) sys.addH(v, v, lambda * (v < n ? sys.B[v * BW] : sys.D[(v - n) * m + (v - n)]) + 1e-9);
      sol = sys.solve();
      if (!sol) {
        lambda *= 10;
        continue;
      }
      const poses = this.poses.map((p, k) => ({ x: p.x + sol.dp[3 * k], y: p.y + sol.dp[3 * k + 1], th: wrap(p.th + sol.dp[3 * k + 2]) }));
      const landmarks = this.landmarks.map((l, k) => ({ ...l, x: l.x + sol.dl[2 * k], y: l.y + sol.dl[2 * k + 1] }));
      const next = this.totalCost(poses, landmarks);
      this.iters++;
      if (next <= cost) {
        this.poses = poses;
        this.landmarks = landmarks;
        const gain = cost - next;
        cost = next;
        lambda = Math.max(1e-7, lambda / 10);
        if (gain < 1e-4 * Math.max(1, cost)) break;
      } else {
        lambda *= 10;
        if (lambda > 1e4) break;
      }
    }
    this.cost = cost;
    // Covariances at the solution: the landmarks' from S⁻¹, the newest pose's
    // from the full inverse. One more factorization, undamped.
    if (sys) this.marginals(n, m);
    this.refreshCurrent();
    this.solveMs = typeof performance !== "undefined" ? performance.now() - t0 : 0;
  }

  marginals(n, m) {
    const sys = new ArrowSystem(n, m);
    sys.addEdge([0, 1, 2], [[1, 0, 0], [0, 1, 0], [0, 0, 1]], this.prior.info, [0, 0, 0]);
    this.odo.forEach((o, k) => {
      const { Ja, Jb } = this.odoError(this.poses[k], this.poses[k + 1], o.z);
      sys.addEdge([3 * k, 3 * k + 1, 3 * k + 2, 3 * k + 3, 3 * k + 4, 3 * k + 5], Ja.map((row, r) => [...row, ...Jb[r]]), o.info, [0, 0, 0]);
    });
    for (const s of this.sights) {
      const { Jp, Jl } = sightError(this.poses[s.i], this.landmarks[s.l], s.r, s.b);
      sys.addEdge([3 * s.i, 3 * s.i + 1, 3 * s.i + 2, n + 2 * s.l, n + 2 * s.l + 1], Jp.map((row, r) => [...row, ...Jl[r]]), s.info, [0, 0]);
    }
    this.landmarks.forEach((l, k) => {
      if (l.prior) sys.addEdge([n + 2 * k, n + 2 * k + 1], [[1, 0], [0, 1]], l.prior.info, [0, 0]);
    });
    for (let v = 0; v < n + m; v++) sys.addH(v, v, 1e-9);
    const sol = sys.solve();
    if (!sol) return;
    this.landmarks.forEach((l, k) => {
      l.P = [[sol.Si[2 * k][2 * k], sol.Si[2 * k][2 * k + 1]], [sol.Si[2 * k + 1][2 * k], sol.Si[2 * k + 1][2 * k + 1]]];
    });
    this.lastP = sys.poseCov(n - 3, sol.Y, sol.Si);
  }

  /** Drop world landmark j, and shift the indices above it down. */
  forget(j) {
    const gone = new Set(this.landmarks.map((l, k) => (l.j === j ? k : -1)).filter((k) => k >= 0));
    if (gone.size) {
      const newIndex = [];
      let c = 0;
      this.landmarks.forEach((_, k) => (newIndex[k] = gone.has(k) ? -1 : c++));
      this.landmarks = this.landmarks.filter((_, k) => !gone.has(k));
      this.sights = this.sights.filter((s) => !gone.has(s.l)).map((s) => ({ ...s, l: newIndex[s.l] }));
      this.closures = this.closures.filter((x) => !gone.has(x.l)).map((x) => ({ ...x, l: newIndex[x.l] }));
    }
    for (const l of this.landmarks) if (l.j > j) l.j--;
    if (this.last && this.last.j === j) this.last = null;
  }
}
