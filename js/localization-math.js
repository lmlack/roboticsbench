// "The math" panel for the Localization tool: the motion and measurement
// models, then each estimator's update with live numbers substituted in.
// Equations are LaTeX typeset by KaTeX (see tex.js); the sentences around
// them are plain text. Everything shown is built from numbers, never user text.

import {
  A_FAST, A_SLOW, ALPHA, DT, ELLIPSE_K2, GATE, GRID, INJECT_RATIO, MAX_INJECT, MATCHERS, NEW_LANDMARK, UKF_PARAMS,
  expectedMeasurement, noiseVar,
} from "./localization-sim.js";
import { CLOSURE_GAP, KEYFRAME, MAX_POSES } from "./posegraph.js";
import { inl, mat, num, row, tex, unit, vec } from "./tex.js";
import { fmt } from "./util.js";

const DEG = Math.PI / 180;

/** An equation row followed by an optional one-line note. */
const step = (latex, note) => `<div class="eq">${row(latex)}</div>${note ? `<p class="eq-note">${note}</p>` : ""}`;

/**
 * A matrix whose entries are all tiny reads better with the power of ten
 * pulled out front: 10⁻⁴ [1.23 0.05; ...] instead of a grid of exponents.
 */
function scaled(rows, d = 2) {
  const max = Math.max(...rows.flat().map(Math.abs));
  const e = max > 0 ? Math.floor(Math.log10(max)) : 0;
  if (e >= -1) return mat(rows, 3);
  return `10^{${e}} ${mat(rows.map((r) => r.map((v) => v / 10 ** e)), d)}`;
}

/** Comma-grouped integer for LaTeX, 17920 as 17{,}920. */
const big = (n) => n.toLocaleString("en-US").replace(/,/g, "{,}");

const G_SYM = "\\begin{bmatrix} 1 & 0 & -v\\,\\Delta t\\sin\\theta_m \\\\ 0 & 1 & v\\,\\Delta t\\cos\\theta_m \\\\ 0 & 0 & 1 \\end{bmatrix}";
const H_SYM = "\\begin{bmatrix} -\\dfrac{\\Delta x}{\\sqrt q} & -\\dfrac{\\Delta y}{\\sqrt q} & 0 \\\\[6pt] \\dfrac{\\Delta y}{q} & -\\dfrac{\\Delta x}{q} & -1 \\end{bmatrix}";

export function createMathPanel(root, sim) {
  const $ = (sel) => root.querySelector(sel);
  // The note paragraph that follows an equation container in its block.
  const noteOf = (id) => $(id).parentElement.querySelector("p.small.subtle");

  // Live rows: each container holds one KaTeX row per line.
  function live(id, lines) {
    const el = $(id);
    if (el.className !== "loc-live") el.className = "loc-live";
    while (el.children.length < lines.length) el.append(Object.assign(document.createElement("div"), { className: "eq live" }));
    while (el.children.length > lines.length) el.lastChild.remove();
    lines.forEach((l, i) => tex(el.children[i], l));
  }

  // ---- The equations themselves, written once ----
  $("#loc-math-motion").innerHTML = [
    step("x' = x + \\hat v\\,\\Delta t\\cos\\!\\big(\\theta + \\tfrac12\\hat\\omega\\,\\Delta t\\big)"),
    step("y' = y + \\hat v\\,\\Delta t\\sin\\!\\big(\\theta + \\tfrac12\\hat\\omega\\,\\Delta t\\big)"),
    step("\\theta' = \\theta + \\hat\\omega\\,\\Delta t"),
    step("\\hat v = v + \\varepsilon_v,\\quad \\varepsilon_v \\sim \\mathcal N\\big(0,\\ \\alpha_1 v^2 + \\alpha_2\\omega^2\\big)"),
    step("\\hat\\omega = \\omega + \\varepsilon_\\omega,\\quad \\varepsilon_\\omega \\sim \\mathcal N\\big(0,\\ \\alpha_3 v^2 + \\alpha_4\\omega^2\\big)"),
  ].join("");
  noteOf("#loc-math-motion").innerHTML =
    `The estimators get the commanded ${inl("v")} and ${inl("\\omega")}. The real robot executes them with this noise added, and a wall can stop it while the command says it moved.`;

  $("#loc-math-meas").innerHTML = [
    step("r = \\sqrt{(m_x - x)^2 + (m_y - y)^2} + \\varepsilon_r"),
    step("\\varphi = \\operatorname{atan2}(m_y - y,\\ m_x - x) - \\theta + \\varepsilon_\\varphi"),
    step("\\varepsilon_r \\sim \\mathcal N(0, \\sigma_r^2),\\quad \\varepsilon_\\varphi \\sim \\mathcal N(0, \\sigma_\\varphi^2)"),
    step("\\lVert m_j - x \\rVert \\le r_{\\max}",
      `Landmark ${inl("j")} is measured only when it is within range and the straight line from the robot to it crosses no wall.`),
  ].join("");
  const measNote = () => (sim.params.lookAlike
    ? `Landmarks look alike: a measurement doesn't say which landmark it came from, so each estimator has to match it (see data association below). The estimators are not told about hidden landmarks; they just get fewer measurements.`
    : `Each measurement says which landmark it came from. The estimators are not told about hidden landmarks; they just get fewer measurements.`);

  $("#loc-math-dr").innerHTML = step("\\hat x_k = g(u_k,\\ \\hat x_{k-1}),\\quad \\hat x_0 = x_\\text{start}");

  $("#loc-math-pf").className = "";
  $("#loc-math-pf").innerHTML = [
    step("x_i \\leftarrow g(u + \\varepsilon_i,\\ x_i)", "Predict: each particle draws its own noise."),
    step("\\log w_i \\mathrel{+}= -\\frac12 \\sum_j \\left[\\frac{(r_j - \\hat r_{ij})^2}{\\sigma_r^2} + \\frac{(\\varphi_j - \\hat\\varphi_{ij})^2}{\\sigma_\\varphi^2}\\right]",
      `Weight: ${inl("w_i \\propto p(z \\mid x_i)")}, kept as a sum of logs.`),
    step("w_i \\leftarrow \\frac{w_i}{\\sum_k w_k}", "Normalize, in log space so nothing underflows."),
    step("N_\\text{eff} = \\frac{1}{\\sum_i w_i^2},\\qquad \\text{resample if } N_\\text{eff} < \\frac N2",
      `Systematic: one random offset, then ${inl("N")} evenly spaced pointers.`),
    step(`w_\\text{slow} \\mathrel{+}= ${A_SLOW}\\,(w_\\text{avg} - w_\\text{slow}),\\quad w_\\text{fast} \\mathrel{+}= ${A_FAST}\\,(w_\\text{avg} - w_\\text{fast})`,
      "How well the whole set explains each scan, slow and fast."),
    step(`p_\\text{inject} = \\min\\!\\left(${MAX_INJECT},\\ 1 - \\frac{w_\\text{fast}}{${INJECT_RATIO}\\,w_\\text{slow}}\\right)`,
      "The share of resampled particles placed at random."),
  ].join("");

  $("#loc-math-grid").className = "";
  $("#loc-math-grid").innerHTML = [
    step("\\overline{bel}(x) = \\sum_{x'} p(x \\mid x', u)\\, bel(x')",
      `Predict: shift every cell by the odometry summed over ${GRID.every} steps, then blur by ${inl("[\\,b,\\ 1 - 2b,\\ b\\,]")} along ${inl("x")}, ${inl("y")} and ${inl("\\theta")}. Fractional shifts are split between neighboring cells.`),
    step("bel(x) = \\eta\\; p(z \\mid x)\\; \\overline{bel}(x)",
      "Update: the particle filter's likelihood, with the noise widened by half a cell and half a heading bin:"),
    step("\\sigma_{r,g}^2 = \\sigma_r^2 + \\big(\\tfrac{\\Delta}{2}\\big)^2,\\quad \\sigma_{\\varphi,g}^2 = \\sigma_\\varphi^2 + \\big(\\tfrac{\\Delta\\theta}{2}\\big)^2"),
    step(`bel(x) \\leftarrow (1 - \\epsilon)\\, bel(x) + \\frac{\\epsilon}{|X_\\text{free}|},\\quad \\epsilon = ${GRID.floor}`,
      "The floor that lets it find the robot again after a kidnap."),
    step("\\hat x = \\frac{\\sum_{x \\in B} bel(x)\\, x}{\\sum_{x \\in B} bel(x)}",
      `${inl("B")} is the 5 × 5 × 5 block of cells around the mode. The global mean is meaningless when the belief has two peaks.`),
  ].join("");

  $("#loc-math-predict").innerHTML = [
    step("\\bar\\mu = g(u, \\mu)"),
    step("\\bar\\Sigma = G\\,\\Sigma\\,G^\\top + V M V^\\top"),
    step(`G = \\frac{\\partial g}{\\partial x} = ${G_SYM}`),
    `<p class="eq-note">${inl("\\theta_m = \\theta + \\tfrac12\\omega\\,\\Delta t")}. ${inl("V = \\partial g / \\partial u")} maps the command noise ${inl("M")} into the state. ${inl("M")} adds a small floor so the covariance never shrinks to zero while the robot is stopped.</p>`,
  ].join("");

  $("#loc-math-update").innerHTML = [
    step("\\hat z = h(\\bar\\mu),\\quad \\nu = z - \\hat z", `The bearing part of ${inl("\\nu")} is wrapped to ${inl("\\pm\\pi")}.`),
    step("S = H\\bar\\Sigma H^\\top + R,\\quad K = \\bar\\Sigma H^\\top S^{-1}"),
    step("\\mu = \\bar\\mu + K\\nu,\\quad \\Sigma = (I - KH)\\,\\bar\\Sigma\\,(I - KH)^\\top + KRK^\\top",
      "The Joseph form: the same result in exact arithmetic, but it stays symmetric under rounding."),
    step(`d^2 = \\nu^\\top S^{-1}\\nu \\le ${GATE}`, "The gate: 99.9% for 2 degrees of freedom. Anything further out is rejected as an outlier."),
    step(`H = \\frac{\\partial h}{\\partial x} = ${H_SYM},\\quad q = \\Delta x^2 + \\Delta y^2`,
      "From the predicted pose to the landmark. Landmarks are folded in one at a time."),
  ].join("");

  $("#loc-math-slam").innerHTML = [
    step("\\mu = (x,\\ y,\\ \\theta,\\ m_{1,x},\\ m_{1,y},\\ \\dots,\\ m_{N,x},\\ m_{N,y})^\\top",
      `The robot and every landmark mapped so far, in one Gaussian. ${inl("\\Sigma")} is ${inl("(3 + 2N) \\times (3 + 2N)")}: it also holds how each landmark's error is tied to the robot's and to the other landmarks'.`),
    step("\\Sigma_{rr} \\leftarrow G \\Sigma_{rr} G^\\top + V M V^\\top,\\quad \\Sigma_{rm} \\leftarrow G\\, \\Sigma_{rm}",
      "Predict moves only the robot. The landmarks stay put, but their link to the robot turns with it."),
    step("m_j = \\begin{pmatrix} x + r\\cos(\\theta + b) \\\\ y + r\\sin(\\theta + b) \\end{pmatrix}",
      "The first time a landmark is seen it goes in where the measurement puts it."),
    step("\\Sigma_{jj} = G_r \\Sigma_{rr} G_r^\\top + G_z R\\, G_z^\\top,\\quad \\Sigma_{j\\cdot} = G_r \\Sigma_{r\\cdot}",
      `It inherits the robot's uncertainty (${inl("G_r")}) plus the sensor's (${inl("G_z")}), and starts out correlated with everything the robot is.`),
    step("H = \\begin{pmatrix} H_r & 0 \\cdots 0 & H_j & 0 \\cdots 0 \\end{pmatrix}",
      "Seen again, it updates like the EKF, but H has entries for the robot and that one landmark. The gain K is full, so one sighting moves the whole map."),
    step("\\mu \\leftarrow \\mu + K\\nu,\\quad \\Sigma \\leftarrow \\Sigma - K S K^\\top", `Gated at ${inl(`d^2 \\le ${GATE}`)}, like the EKF.`),
  ].join("");

  $("#loc-math-graph").className = "";
  $("#loc-math-graph").innerHTML = [
    step("\\mathbf{x} = (x_0,\\ x_1,\\ \\dots,\\ x_K,\\ m_1,\\ \\dots,\\ m_L)",
      `The nodes: every keyframe pose and every landmark. A new pose goes in once the robot has moved ${KEYFRAME.dist * 100} cm or turned ${Math.round(KEYFRAME.turn / DEG)}°. Sightings in between are skipped.`),
    step("e_k = (x_{k+1} \\ominus x_k) - u_k",
      `An odometry edge: where pose ${inl("k+1")} sits as seen from pose ${inl("k")}, against what the wheels said. Its weight ${inl("\\Omega_k")} is the inverse of the odometry's covariance over that stretch.`),
    step("e_{kj} = h(x_k,\\ m_j) - z_{kj}",
      `A sighting edge: the same range and bearing model as the EKF, from pose ${inl("k")} to landmark ${inl("j")}. Seeing a landmark again after more than ${CLOSURE_GAP} s out of sight is a loop closure.`),
    step("F(\\mathbf{x}) = e_0^\\top \\Omega_0\\, e_0 + \\sum_k e_k^\\top \\Omega_k\\, e_k"),
    step("\\qquad\\qquad + \\sum_{k,j} e_{kj}^\\top \\Omega_{kj}\\, e_{kj}",
      `Least squares over the whole path at once. ${inl("e_0")} pins the first pose to the start, which is what ties the map to the world.`),
    step("H = \\sum J^\\top \\Omega J,\\quad g = \\sum J^\\top \\Omega\\, e"),
    step("(H + \\lambda \\operatorname{diag} H)\\,\\delta = -g",
      `Gauss-Newton: linearize every edge at the current guess, solve for the step ${inl("\\delta")}, repeat. ${inl("\\lambda")} (Levenberg-Marquardt) shortens a step that would make ${inl("F")} worse.`),
    step("H = \\begin{pmatrix} B & C \\\\ C^\\top & D \\end{pmatrix},\\quad S = D - C^\\top B^{-1} C",
      `Odometry only links neighbors, so ${inl("B")}, the pose block, is a narrow band. Taking the poses out leaves ${inl("S")}, a small system over just the landmarks. That's why a few hundred poses solve in milliseconds. Past ${MAX_POSES} poses, every other one in the older half is merged into its neighbors.`),
    step("\\Sigma_m = S^{-1}", "The landmarks' covariance, for their ellipses on the map."),
  ].join("");

  $("#loc-math-assoc").className = "";
  $("#loc-math-assoc").innerHTML = [
    step("\\nu_j = z - h(\\mu,\\ m_j),\\quad d_j^2 = \\nu_j^\\top S_j^{-1} \\nu_j",
      `For each landmark it might be: how far the sighting is from what that landmark predicts, measured in units of the spread ${inl("S_j")} the filter expects.`),
    step("\\hat\\jmath = \\arg\\min_j d_j^2", "Pick the closest fit. This is nearest-neighbor matching."),
    step(`d_{\\hat\\jmath}^2 \\le ${GATE} \\ \\Rightarrow\\ \\text{use it}`, "The same 99.9% gate as before. Past it the sighting is dropped."),
    step(`\\min_j d_j^2 > ${NEW_LANDMARK} \\ \\Rightarrow\\ \\text{new landmark}`,
      "SLAM only: nothing on its map fits, so it's one it hasn't seen yet. In between, it's too unsure to use."),
    step("\\log w_i \\mathrel{+}= -\\tfrac12 \\min_j c_{ij}"),
    step("c_{ij} = \\frac{(r - \\hat r_{ij})^2}{\\sigma_r^2} + \\frac{(\\varphi - \\hat\\varphi_{ij})^2}{\\sigma_\\varphi^2}",
      `The particle and grid filters never commit to one match. Each guess ${inl("x_i")} at the pose is scored against whichever landmark fits it best, so a wrong guess can't drag the right ones along.`),
  ].join("");

  $("#loc-math-ukf").className = "";
  $("#loc-math-ukf").innerHTML = [
    step("\\chi_0 = \\mu,\\quad \\chi_i = \\mu + \\Big(\\sqrt{(n+\\lambda)\\Sigma}\\Big)_i,\\quad \\chi_{i+n} = \\mu - \\Big(\\sqrt{(n+\\lambda)\\Sigma}\\Big)_i",
      `Column ${inl("i")} of the Cholesky factor, so ${inl("2n + 1 = 7")} points.`),
    step("\\lambda = \\alpha^2 (n + \\kappa) - n"),
    step("W_0^{(m)} = \\frac{\\lambda}{n+\\lambda},\\quad W_0^{(c)} = W_0^{(m)} + 1 - \\alpha^2 + \\beta"),
    step("W_i^{(m)} = W_i^{(c)} = \\frac{1}{2(n+\\lambda)},\\quad i = 1, \\dots, 2n"),
    step("\\bar\\mu = \\sum_i W_i^{(m)}\\, g(u, \\chi_i)"),
    step("\\bar\\theta = \\operatorname{atan2}\\!\\Big(\\sum_i W_i^{(m)} \\sin\\theta_i,\\ \\sum_i W_i^{(m)} \\cos\\theta_i\\Big)",
      "Heading is averaged as a unit vector, so 179° and −179° average to 180°, not 0°."),
    step("\\bar\\Sigma = \\sum_i W_i^{(c)} (\\bar\\chi_i - \\bar\\mu)(\\bar\\chi_i - \\bar\\mu)^\\top + V M V^\\top", "Heading residuals wrapped."),
    step("\\hat z = \\sum_i W_i^{(m)} h(\\chi_i),\\quad S = \\sum_i W_i^{(c)} (Z_i - \\hat z)(Z_i - \\hat z)^\\top + R", "Bearing residuals wrapped."),
    step("P_{xz} = \\sum_i W_i^{(c)} (\\chi_i - \\mu)(Z_i - \\hat z)^\\top,\\quad K = P_{xz} S^{-1}"),
    step("\\mu \\leftarrow \\mu + K\\nu,\\quad \\Sigma \\leftarrow \\Sigma - K S K^\\top", `Gated at ${inl(`d^2 \\le ${GATE}`)}, like the EKF.`),
  ].join("");

  function gateVerdict(node, last, rej, name) {
    const n = sim.z.length;
    if (!last.accepted) {
      node.className = "small verdict is-bad";
      node.innerHTML = `Rejected: ${inl(`d^2 = ${num(last.d2, 1)}`)} is past the gate of ${GATE}. ${rej} of ${n} measurements were rejected this step, so the ${name} is coasting on odometry.`;
    } else {
      node.className = "small verdict is-ok";
      node.textContent = rej
        ? `Accepted. ${rej} of ${n} measurements were rejected this step.`
        : `Accepted. All ${n} measurements passed the gate this step.`;
    }
  }

  function update() {
    const u = sim.u;
    const k = sim.params.odomNoise;
    const [vv, vw] = noiseVar(u.v, u.w, k);
    live("#loc-math-motion-live", [
      `v = ${num(u.v)}${unit("m/s")},\\quad \\omega = ${num(u.w)}${unit("rad/s")},\\quad \\Delta t = ${fmt(DT * 1000, 0)}${unit("ms")}`,
      `\\alpha = (${ALPHA.join(",\\ ")}) \\times ${num(k, 1)}`,
      `\\sigma_v = ${num(Math.sqrt(vv), 3)}${unit("m/s")},\\quad \\sigma_\\omega = ${num(Math.sqrt(vw), 3)}${unit("rad/s")}`,
    ]);

    // The landmark the EKF used last, measured and as predicted from the true pose.
    const { r: sR, b: sB } = sim.sigma;
    const lastZ = sim.ekf.last;
    const zm = (lastZ && sim.z.find((m) => m.j === lastZ.j)) || sim.z[0];
    // Counts are a sentence, not math, so they read in the body font.
    let seenEl = $("#loc-math-seen");
    if (!seenEl) {
      seenEl = Object.assign(document.createElement("p"), { id: "loc-math-seen", className: "math-status subtle" });
      $("#loc-math-meas-live").after(seenEl);
    }
    const blockedN = sim.blocked.length;
    const seenText = `${sim.z.length} in view, ${blockedN} behind ${blockedN === 1 ? "a wall" : "walls"}.`;
    if (seenEl.textContent !== seenText) seenEl.textContent = seenText;
    if (zm && sim.landmarks[zm.j]) {
      const lm = sim.landmarks[zm.j];
      const h = expectedMeasurement(sim.truth, lm);
      live("#loc-math-meas-live", [
        `m_{${zm.j + 1}} = (${num(lm.x)},\\ ${num(lm.y)}),\\quad z = (${num(zm.r, 3)}${unit("m")},\\ ${num(zm.b, 3)}${unit("rad")})`,
        `h(x_\\text{true}) = (${num(h.r, 3)}${unit("m")},\\ ${num(h.b, 3)}${unit("rad")})`,
        `\\sigma_r = ${num(sR, 3)}${unit("m")},\\quad \\sigma_\\varphi = ${num(sB, 3)}${unit("rad")}`,
      ]);
    } else {
      live("#loc-math-meas-live", []);
    }

    const d = sim.dr.estimate();
    const dErr = sim.errors().dr;
    live("#loc-math-dr-live", [
      `\\hat x = (${num(d.x)},\\ ${num(d.y)},\\ ${num(d.th)}),\\quad \\lVert \\hat x - x \\rVert = ${num(dErr.pos)}${unit("m")},\\quad |\\hat\\theta - \\theta| = ${num(dErr.head / DEG, 1)}^\\circ`,
    ]);

    const pf = sim.pf;
    live("#loc-math-pf-live", [
      `N = ${pf.n},\\quad N_\\text{eff} = ${Math.round(pf.neff)}`,
      `w_\\text{avg} = ${num(pf.wAvg, 3)},\\quad w_\\text{fast} = ${num(pf.wFast, 3)},\\quad w_\\text{slow} = ${num(pf.wSlow, 3)}`,
      `p_\\text{inject} = ${num(pf.inject)}`,
    ]);
    const pv = $("#loc-math-pf-verdict");
    if (pf.inject > 0) {
      pv.className = "small verdict is-bad";
      pv.textContent = `Scans fit much worse than usual, so ${Math.round(pf.inject * 100)}% of each resample is placed at random to search for the robot.`;
    } else {
      pv.className = "small verdict is-ok";
      pv.textContent = "Scans fit about as well as usual. No random particles.";
    }

    const gr = sim.grid;
    const srg = Math.sqrt(sR * sR + (GRID.cell / 2) ** 2);
    const sbg = Math.sqrt(sB * sB + (gr.dth / 2) ** 2);
    live("#loc-math-grid-live", [
      `${gr.nx} \\times ${gr.ny} \\times ${gr.nt} = ${big(gr.N)}\\ \\text{cells},\\quad ${big(gr.free * gr.nt)}\\ \\text{free}`,
      `\\Delta = ${GRID.cell}${unit("m")},\\quad \\Delta\\theta = ${num(gr.dth / DEG, 1)}^\\circ,\\quad ${fmt(1 / (GRID.every * DT), 0)}${unit("Hz")}`,
      `\\text{last shift} = (${num(gr.lastMove.x, 3)}${unit("m")},\\ ${num(gr.lastMove.y, 3)}${unit("m")},\\ {${num(gr.lastMove.th / DEG, 1)}}^\\circ)`,
      `b_{xy} = ${num(gr.blur.xy, 3)},\\quad b_\\theta = ${num(gr.blur.th, 3)},\\quad \\sigma_{r,g} = ${num(srg, 3)}${unit("m")},\\quad \\sigma_{\\varphi,g} = ${num(sbg, 3)}${unit("rad")}`,
      `\\text{mode} = (${gr.mode.i},\\ ${gr.mode.j},\\ ${gr.mode.k}),\\quad \\textstyle\\sum_B bel = ${num(gr.modeMass, 3)}`,
    ]);

    const ekf = sim.ekf;
    live("#loc-math-predict-live", [`G = ${mat(ekf.G, 3)}`, `\\bar\\Sigma = ${scaled(ekf.Pbar)}`]);

    const uv = $("#loc-math-update-verdict");
    if (lastZ && sim.landmarks[lastZ.j]) {
      const lines = [
        `j = ${lastZ.j + 1}:\\quad \\nu = ${vec(lastZ.nu, 3)},\\quad d^2 = ${num(lastZ.d2)}`,
        `S = ${scaled(lastZ.S)}`,
      ];
      if (lastZ.K) lines.push(`K = ${mat(lastZ.K, 3)}`);
      live("#loc-math-update-live", lines);
      gateVerdict(uv, lastZ, ekf.rejected, "EKF");
    } else {
      live("#loc-math-update-live", []);
      uv.className = "small verdict";
      uv.textContent = "With no landmark in view the EKF only predicts, and its covariance grows.";
    }

    const P = ekf.P;
    const e = ekf.ellipse();
    live("#loc-math-cov-live", [
      `\\mu = ${vec(ekf.mu, 3)},\\quad \\Sigma = ${scaled(P)}`,
      `\\sigma_x = ${num(Math.sqrt(P[0][0]), 3)}${unit("m")},\\quad \\sigma_y = ${num(Math.sqrt(P[1][1]), 3)}${unit("m")},\\quad \\sigma_\\theta = ${num(Math.sqrt(P[2][2]) / DEG, 1)}^\\circ`,
      `\\sqrt{${ELLIPSE_K2}\\,\\lambda_{1,2}} = ${num(e.rx, 3)}${unit("m")},\\ ${num(e.ry, 3)}${unit("m")}`,
    ]);
    $("#loc-math-cov-note").innerHTML =
      `The diagonal holds the variances of ${inl("x")} and ${inl("y")} in m² and of ${inl("\\theta")} in rad². ${inl("\\lambda_{1,2}")} are the eigenvalues of the ${inl("x, y")} block; the last line gives the semi-axes of the 95% ellipse on the map. The wedge spans ${inl("\\pm 2\\sigma_\\theta")}.`;

    const ukf = sim.ukf;
    const { alpha, beta, kappa } = UKF_PARAMS;
    const ue = ukf.ellipse();
    const ul = ukf.last;
    const ukfLines = [
      `n = 3,\\quad \\alpha = ${alpha},\\quad \\beta = ${beta},\\quad \\kappa = ${kappa},\\quad \\lambda = ${num(ukf.lambda, 0)},\\quad \\sqrt{n + \\lambda} = ${num(ukf.gamma, 0)}`,
      `W_0^{(m)} = ${num(ukf.wm[0], 3)},\\quad W_0^{(c)} = ${num(ukf.wc[0], 3)},\\quad W_i = ${num(ukf.wm[1], 3)}`,
      // One sigma point per row: 7 × 3 fits a phone column, 3 × 7 does not.
      `\\chi^\\top = {\\small ${mat(ukf.chi, 3)}}`,
    ];
    if (ul && sim.landmarks[ul.j]) ukfLines.push(`\\nu = ${vec(ul.nu, 3)},\\quad d^2 = ${num(ul.d2)}`);
    ukfLines.push(`\\text{95\\% ellipse: } ${num(ue.rx, 3)}${unit("m")} \\times ${num(ue.ry, 3)}${unit("m")}`);
    live("#loc-math-ukf-live", ukfLines);

    const slam = sim.slam;
    const smap = slam.map();
    const sP = slam.P;
    const slamLines = [
      `N = ${smap.length},\\quad \\dim \\mu = ${slam.mu.length},\\quad \\Sigma: ${slam.mu.length} \\times ${slam.mu.length}`,
      `\\sigma_x = ${num(Math.sqrt(sP[0][0]), 3)}${unit("m")},\\quad \\sigma_y = ${num(Math.sqrt(sP[1][1]), 3)}${unit("m")},\\quad \\sigma_\\theta = ${num(Math.sqrt(sP[2][2]) / DEG, 1)}^\\circ`,
    ];
    if (smap.length) {
      const widest = smap.reduce((a, b) => (b.rx > a.rx ? b : a));
      slamLines.push(`\\text{least certain landmark: } ${num(widest.rx, 3)}${unit("m")} \\text{ (95\\%)}`);
    }
    live("#loc-math-slam-live", slamLines);
    const sv = $("#loc-math-slam-verdict");
    sv.className = "small verdict";
    sv.textContent = !smap.length
      ? "Nothing mapped yet: drive somewhere a landmark is in range."
      : slam.added
        ? `Just added a landmark to the map, which now has ${smap.length}.`
        : slam.last && slam.last.accepted
          ? "Updating the robot and the map from landmarks it has mapped before."
          : "Only predicting: no mapped landmark in view, so the robot's uncertainty grows.";
    const mn = measNote();
    if (noteOf("#loc-math-meas").textContent !== mn) noteOf("#loc-math-meas").textContent = mn;

    const pg = sim.graph;
    const nOdo = pg.odo.length, nSight = pg.sights.length;
    live("#loc-math-graph-live", [
      `K + 1 = ${pg.poses.length}\\ \\text{poses},\\quad L = ${pg.landmarks.length}\\ \\text{landmarks}`,
      `${nOdo}\\ \\text{odometry edges},\\quad ${nSight}\\ \\text{sightings}`,
      `\\dim \\mathbf{x} = ${3 * pg.poses.length + 2 * pg.landmarks.length},\\quad S: ${2 * pg.landmarks.length} \\times ${2 * pg.landmarks.length}`,
      `F = ${num(pg.cost, 1)},\\quad ${pg.iters}\\ \\text{iterations},\\quad ${num(pg.solveMs, 1)}${unit("ms")}`,
    ]);
    const gv = $("#loc-math-graph-verdict");
    const lastClose = pg.closures[pg.closures.length - 1];
    if (lastClose && pg.t - lastClose.t < 3 && pg.before && pg.before.poses.length === pg.poses.length) {
      const moved = Math.max(...pg.poses.map((p, k) => Math.hypot(p.x - pg.before.poses[k].x, p.y - pg.before.poses[k].y)));
      gv.className = "small verdict is-ok";
      gv.textContent = `Loop closure: it saw an old landmark again, and re-solving moved the path by up to ${fmt(moved * 100, 0)} cm.`;
    } else {
      gv.className = "small verdict";
      gv.textContent = pg.closures.length
        ? `${pg.closures.length} loop closure${pg.closures.length === 1 ? "" : "s"} so far. Between them the chain drifts with the odometry, like EKF-SLAM.`
        : "No loop closure yet. Each new pose is solved against the landmarks seen so far.";
    }

    const av = $("#loc-math-assoc-verdict");
    if (!sim.params.lookAlike) {
      live("#loc-math-assoc-live", []);
      av.className = "small verdict";
      av.textContent = "Off: every sighting says which landmark it is, so nothing here is used. Turn on Landmarks look alike.";
    } else {
      const el = sim.ekf.last;
      live("#loc-math-assoc-live", el && Number.isFinite(el.d2) ? [
        `\\text{EKF: } \\hat\\jmath = ${el.j + 1},\\quad d^2 = ${num(el.d2, 1)}`,
        `\\text{next best: } d^2 = ${Number.isFinite(el.next) ? num(el.next, 0) : "\\infty"}`,
      ] : []);
      const counts = MATCHERS.map((k) => sim.wrongCount(k));
      const total = counts.reduce((a, b) => a + b, 0);
      av.className = total ? "small verdict is-bad" : "small verdict is-ok";
      av.textContent = total
        ? `Wrong matches in the last 20 s: EKF ${counts[0]}, UKF ${counts[1]}, EKF-SLAM ${counts[2]}, graph SLAM ${counts[3]}. Each one pulls the estimate toward the wrong landmark, which makes the next match even more likely to be wrong.`
        : "No wrong matches in the last 20 s. While the estimate is close, the nearest landmark is the right one.";
    }

    const ukv = $("#loc-math-ukf-verdict");
    if (ul && sim.landmarks[ul.j]) gateVerdict(ukv, ul, ukf.rejected, "UKF");
    else {
      ukv.className = "small verdict";
      ukv.textContent = "With no landmark in view the UKF only predicts, and its covariance grows.";
    }
  }

  return { update };
}
