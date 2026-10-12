// "The math" panel: the control law with live numbers, the discrete update the
// code runs, the plant's equation of motion, and the linearized closed loop
// with its poles. Equations are LaTeX typeset by KaTeX (see tex.js); the
// sentences around them are plain text.

import { SYSTEMS, CONTROL_DT, analyzePoles, charPoly, polyRoots } from "./pid-sim.js";
import { inl, num, par, row, sum, tex, unit } from "./tex.js";
import { fmt } from "./util.js";

const OUTPUT = {
  drone: { sym: "T", name: "thrust", unit: "N" },
  arm: { sym: "\\tau", name: "motor torque", unit: "N·m" },
  cartpole: { sym: "F", name: "cart force", unit: "N" },
};

const PLANT = {
  drone: {
    eqs: ["m\\ddot y = T - mg - c\\,\\dot y"],
    params: "m = 1 kg, g = 9.81 m/s², c = 0.15 N·s/m, 0 ≤ T ≤ 25 N",
    note: "The motors cannot pull down, so T has a floor of 0. Hovering needs T = mg = 9.81 N, which only the integral term can supply with zero error.",
  },
  arm: {
    eqs: ["J\\ddot\\theta = \\tau - mgL\\sin\\theta - b\\,\\dot\\theta"],
    params: "J = 1.05 kg·m², m = 1 kg, L = 1 m, b = 0.25 N·m·s, |τ| ≤ 14 N·m",
    note: "θ is measured from hanging straight down. The gravity term is largest at ±90°, where holding the arm level takes 9.81 N·m.",
  },
  cartpole: {
    eqs: [
      "(M + m)\\,\\ddot x + ml\\,\\ddot\\theta\\cos\\theta - ml\\,\\dot\\theta^2\\sin\\theta = F",
      "l\\,\\ddot\\theta + \\ddot x\\cos\\theta - g\\sin\\theta = 0",
      "\\ddot\\theta \\approx \\frac{(M + m)\\,g\\,\\theta - F}{Ml} \\quad \\text{(linearized)}",
    ],
    params: "M = 1 kg (cart), m = 0.2 kg (bob), l = 1 m, |F| ≤ 40 N",
    note: "θ is measured from upright. Gravity pushes the pole further from upright in proportion to θ, so Kp must be larger than (M + m)g ≈ 11.8 N/rad just to stand still.",
  },
};

const POLY_FORM = {
  drone: "m s^3 + (c + K_d)\\,s^2 + K_p\\,s + K_i = 0",
  arm: "J s^3 + (b + K_d)\\,s^2 + (K_p + mgL\\cos\\theta_0)\\,s + K_i = 0",
  cartpole: "Ml\\,s^3 + K_d\\,s^2 + \\big(K_p - (M + m)\\,g\\big)\\,s + K_i = 0",
};

const eqRow = (cls = "eq") => {
  const d = document.createElement("div");
  d.className = cls;
  return d;
};

export function createMathPanel(root, sim) {
  const $ = (sel) => root.querySelector(sel);
  let key = null;

  function load(k) {
    key = k;
    const plant = PLANT[k];
    $("#math-plant").replaceChildren(...plant.eqs.map(() => eqRow()));
    plant.eqs.forEach((e, i) => tex($("#math-plant").children[i], e));
    $("#math-plant-params").textContent = plant.params;
    $("#math-plant-note").textContent = plant.note;
    tex($("#math-poly-form"), POLY_FORM[k]);
    $("#math-out-name").textContent = OUTPUT[k].name;
    $("#math-units").textContent = SYSTEMS[k].unit === "°"
      ? "Angles in radians, the way the controller sees them."
      : "Distances in meters.";
    $("#math-e").innerHTML = `${inl("e(t) = r(t) - y(t)")}, the setpoint minus the measurement.`;
    // Two rows for the live substitution, filled in by update().
    $("#math-live").replaceChildren(eqRow("eq live"), eqRow("eq live"));
    update();
  }

  function update() {
    if (!key) return;
    const sys = SYSTEMS[key];
    const out = OUTPUT[key];
    const pid = sim.pid;
    const last = sim.last;
    const dOnMeas = pid.derivOnMeasurement;
    const neg = (sys.sign ?? 1) < 0;
    const wrap = (s) => (neg ? `-\\left(${s}\\right)` : s);

    // The law, with the derivative term as the code actually computes it.
    const dTerm = dOnMeas ? "- K_d\\,\\dot y(t)" : "+ K_d\\,\\dot e(t)";
    tex($("#math-law"), `${out.sym}(t) = ${wrap(`K_p\\,e(t) + K_i \\int_0^t e(\\tau)\\,d\\tau ${dTerm}`)}`);

    // Live substitution: gains times the controller's own state.
    const [row1, row2] = $("#math-live").children;
    tex(row1, `= ${wrap(`${num(pid.kp)} \\cdot ${par(last.e ?? 0, 3)} + ${num(pid.ki)} \\cdot ${par(pid.integral, 3)} + ${num(pid.kd)} \\cdot ${par(pid.dState, 3)}`)}`);
    tex(row2, `= ${wrap(sum([last.p ?? 0, last.i ?? 0, last.d ?? 0]))} = ${num(last.raw ?? 0)}${unit(out.unit)}`);
    const status = $("#math-status");
    if (last.saturated) {
      status.className = "math-status bad";
      status.textContent = `Clamped to ${fmt(last.u, 1)} ${out.unit}: the actuator is at its limit.`;
    } else {
      status.className = "math-status subtle";
      status.textContent = `Within the limits, ${sys.uLimit[0]} to ${sys.uLimit[1]} ${out.unit}, so it is applied as is.`;
    }

    // The discrete update, as the code runs it.
    const dt = CONTROL_DT * 1000;
    const step = (latex, note) => `<div class="eq">${row(latex)}</div>${note ? `<p class="eq-note">${note}</p>` : ""}`;
    const discrete = [
      step("e_k = r_k - y_k"),
      step("I_k = I_{k-1} + e_k\\,\\Delta t",
        pid.antiWindup ? "Held while the actuator is pinned and the error pushes further in." : "No anti-windup: it keeps growing while saturated."),
      dOnMeas
        ? step("D_k = \\operatorname{lowpass}\\!\\left(-\\frac{y_k - y_{k-1}}{\\Delta t}\\right),\\quad \\tau_D = 20\\,\\text{ms}", "On the measurement, so a setpoint jump gives no kick.")
        : step("D_k = \\operatorname{lowpass}\\!\\left(\\frac{e_k - e_{k-1}}{\\Delta t}\\right),\\quad \\tau_D = 20\\,\\text{ms}", "On the error, so it spikes on every setpoint step."),
      step(`u_k = \\operatorname{clamp}\\!\\left(${neg ? "-(K_p e_k + K_i I_k + K_d D_k)" : "K_p e_k + K_i I_k + K_d D_k"},\\ u_{\\min},\\ u_{\\max}\\right),\\quad \\Delta t = ${dt}\\,\\text{ms}`),
    ].join("");
    const discreteEl = $("#math-discrete");
    if (discreteEl.dataset.src !== discrete) {
      discreteEl.dataset.src = discrete;
      discreteEl.innerHTML = discrete;
    }

    // Plant: the live gravity term or weight.
    const live = $("#math-plant-live");
    if (key === "arm") {
      live.hidden = false;
      live.innerHTML = `Gravity term right now: ${inl(`-mgL\\sin\\theta = ${num(sys.gravityTorque(sim.state))}${unit("N·m")}`)}`;
    } else if (key === "drone") {
      live.hidden = false;
      live.innerHTML = `Right now: ${inl(`mg = 9.81${unit("N")},\\quad T = ${num(sim.last.u)}${unit("N")}`)}`;
    } else {
      live.hidden = true;
    }

    // Closed loop
    const coeffs = charPoly(key, { kp: pid.kp, ki: pid.ki, kd: pid.kd }, sim.sp);
    const powers = coeffs.length === 4 ? ["s^3", "s^2", "s", ""] : ["s^2", "s", ""];
    tex($("#math-poly-live"), `${sum(coeffs.map((c, i) => [c, powers[i]]))} = 0${coeffs.length === 3 ? "\\qquad (K_i = 0 \\text{ drops a factor of } s)" : ""}`);

    const roots = polyRoots(coeffs);
    tex($("#math-poles"), `\\text{poles: } s = ${formatRoots(roots)}`);
    const a = analyzePoles(roots);
    const verdict = $("#math-verdict");
    if (!a.stable) {
      verdict.className = "small verdict is-bad";
      verdict.textContent = "Unstable: a pole has a positive real part, so small errors grow until the actuator saturates or the system falls over.";
    } else {
      verdict.className = "small verdict is-ok";
      const z = a.zeta;
      const ring = z < 0.2 ? "lightly damped, expect lots of ringing"
        : z < 0.5 ? "underdamped, expect clear overshoot"
        : z < 0.8 ? "a little overshoot"
        : "well damped, little or no overshoot";
      const settle = 4 / -a.slowest[0];
      verdict.innerHTML = `Stable. Least damped pole: ${inl(`\\zeta = ${num(z)},\\ \\omega_n = ${num(a.wn)}${unit("rad/s")}`)}, ${ring}. Slowest pole at ${inl(num(a.slowest[0]))}, so roughly ${settle > 60 ? "over a minute" : `${fmt(settle, 1)} s`} to settle.`;
    }

    // Routh-Hurwitz for the cubic, as a hand check on the poles.
    const routh = $("#math-routh");
    if (coeffs.length === 4) {
      const [a3, a2, a1, a0] = coeffs;
      const ok = coeffs.every((c) => c > 0) && a2 * a1 > a3 * a0;
      const rel = a2 * a1 > a3 * a0 ? ">" : "\\le";
      routh.innerHTML = `Routh–Hurwitz: every coefficient positive and ${inl("a_2 a_1 > a_3 a_0")}. Here ${inl(`${num(a2)} \\cdot ${num(a1)} = ${num(a2 * a1, 1)} ${rel} ${num(a3 * a0, 1)}`)}, so <span class="${ok ? "ok" : "bad"}">${ok ? "stable" : "unstable"}</span>.`;
    } else {
      const ok = coeffs.every((c) => c > 0);
      routh.innerHTML = `Second order: stable exactly when every coefficient is positive, <span class="${ok ? "ok" : "bad"}">${ok ? "and they are" : "and they are not"}</span>.`;
    }

    const extra = $("#math-extra");
    if (key === "cartpole") {
      const need = 1.2 * 9.81;
      const ok = pid.kp > need;
      extra.hidden = false;
      extra.innerHTML = `Kp must beat gravity: ${inl(`${num(pid.kp, 1)} ${ok ? ">" : "\\le"} (M + m)\\,g = ${num(need, 1)}${unit("N/rad")}`)}, <span class="${ok ? "ok" : "bad"}">${ok ? "and it does" : "and it does not"}</span>.`;
    } else if (key === "arm") {
      const c = Math.cos(sim.sp);
      const deg = fmt((sim.sp * 180) / Math.PI, 0);
      extra.hidden = false;
      extra.innerHTML = c >= 0
        ? `At ${inl(`\\theta_0 = ${deg}^\\circ`)}, gravity adds ${inl(`${num(9.81 * c)}${unit("N·m/rad")}`)} of stiffness, helping Kp.`
        : `At ${inl(`\\theta_0 = ${deg}^\\circ`)}, above horizontal, gravity subtracts ${inl(`${num(-9.81 * c)}${unit("N·m/rad")}`)} from Kp: the arm is balancing like an inverted pendulum.`;
    } else {
      extra.hidden = true;
    }
  }

  return { load, update };
}

function formatRoots(roots) {
  const out = [];
  const used = new Set();
  roots.forEach(([re, im], i) => {
    if (used.has(i)) return;
    if (im !== 0) {
      const j = roots.findIndex((r, k) => k !== i && !used.has(k) && Math.abs(r[0] - re) < 1e-6 && Math.abs(r[1] + im) < 1e-6);
      if (j >= 0) used.add(j);
      out.push(`${num(re)} \\pm ${num(Math.abs(im))}\\,j`);
    } else {
      out.push(num(re));
    }
    used.add(i);
  });
  return out.join(",\\quad ");
}
