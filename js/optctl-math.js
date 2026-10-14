// "The math" for MPC and iLQR: the cost both minimize, how each one uses the
// solver, and live numbers from the current plan. The plant block is shared
// with PID and filled in by pid-math.js.

import { SYSTEMS } from "./pid-sim.js";
import { MODELS } from "./optctl.js";
import { inl, num, row, unit } from "./tex.js";

const U_SYM = { drone: "T", arm: "\\tau", cartpole: "F" };
const STATE = {
  drone: "x = (y,\\ \\dot y)",
  arm: "x = (\\theta,\\ \\dot\\theta)",
  cartpole: "x = (x_c,\\ \\dot x_c,\\ \\theta,\\ \\dot\\theta)",
};

const diag = (v) => `\\operatorname{diag}(${v.map((x) => num(x, 2)).join(",\\ ")})`;

export function createOptMath(root, sim) {
  const $ = (sel) => root.querySelector(sel);

  function block(id, html) {
    const el = $(id);
    if (el.dataset.src !== html) {
      el.dataset.src = html;
      el.innerHTML = html;
    }
  }

  function update(key, mode) {
    if (mode === "pid" || !sim.opt) return;
    const m = MODELS[key];
    const sys = SYSTEMS[key];
    const u = U_SYM[key];
    const ctrl = sim.opt;
    const plan = ctrl.plan;
    const N = mode === "mpc" ? ctrl.N : plan ? plan.U.length : Math.round(ctrl.horizon / ctrl.dt);
    const dt = ctrl.dt;
    const goal = m.goal(sim.sp);

    block("#math-opt-cost", `
      <p class="eyebrow">What it minimizes</p>
      <div class="eq">${row(`J = \\sum_{k=0}^{N-1} \\Big( e_k^\\top Q\\, e_k + R\\,(${u}_k - ${u}^\\star)^2 \\Big) + e_N^\\top Q_f\\, e_N`)}</div>
      <p class="eq-note">${inl("e_k")} is how far the state ${inl(STATE[key])} is from the goal at step ${inl("k")}. ${inl(`${u}^\\star`)} is the input that holds the goal still, so the cost only charges for effort beyond that.</p>
      <div class="eq live">${row(`Q = ${diag(m.Q)},\\quad R = ${num(m.R, 2)},\\quad Q_f = ${diag(m.Qf)}`)}</div>
      <div class="eq live">${row(`${u}^\\star = ${num(goal.u)}${unit(sys.uUnit)},\\qquad ${sys.uLimit[0]} \\le ${u}_k \\le ${sys.uLimit[1]}${unit(sys.uUnit)}`)}</div>
      <p class="small subtle">Subject to the plant's own equations, ${inl("x_{k+1} = f(x_k, " + u + "_k)")}, and the motor limit. Big ${inl("Q")} means "get there fast", big ${inl("R")} means "go easy on the motor".</p>`);

    block("#math-opt-solve", mode === "mpc" ? `
      <p class="eyebrow">MPC: plan, take one step, plan again</p>
      <div class="eq">${row(`\\min_{${u}_0, \\dots, ${u}_{N-1}} J \\quad\\text{from the state right now}`)}</div>
      <p class="eq-note">Every ${num(dt * 1000, 0)} ms it solves for the next ${inl(`N = ${N}`)} inputs, ${num(N * dt, 1)} s ahead, then applies only the first one and throws the rest away. Re-planning from the measured state is what makes it feedback: a kick just shows up in the next plan.</p>
      <div class="eq">${row(`${u}(t) = \\bar ${u}_0 + K_0\\,\\big(x(t) - \\bar x(t)\\big)`)}</div>
      <p class="eq-note">Between re-plans it follows the first step of the plan, ${inl(`\\bar ${u}_0`)}, and corrects for drift with the gain ${inl("K_0")} the solver hands back. The solver is iLQR, warm-started from the previous plan shifted one step, so ${ctrl.iters} iterations per tick are enough.</p>` : `
      <p class="eyebrow">iLQR: one plan, refined until it stops improving</p>
      <div class="eq">${row(`\\text{backward: } k_t = -\\frac{Q_{u,t}}{Q_{uu,t}},\\quad K_t = -\\frac{Q_{ux,t}}{Q_{uu,t}}`)}</div>
      <p class="eq-note">Each iteration linearizes the plant around the current plan and solves that easier problem backward in time, from the end to now. With one input, ${inl("Q_{uu}")} is a single number, so each step is a division.</p>
      <div class="eq">${row(`\\text{forward: } ${u}_t = \\bar ${u}_t + \\alpha\\,k_t + K_t\\,(x_t - \\bar x_t)`)}</div>
      <p class="eq-note">Then it rolls the real plant forward with the corrected inputs, halving ${inl("\\alpha")} until the cost goes down. Where a correction would push past the motor limit, it is clamped there and the feedback on that step is turned off.</p>`);

    let live = `<p class="eyebrow">Right now</p>`;
    if (mode === "mpc") {
      live += `<div class="eq live">${row(`N = ${N},\\quad \\Delta t = ${num(dt * 1000, 0)}${unit("ms")},\\quad \\text{look-ahead} = ${num(N * dt, 1)}${unit("s")}`)}</div>`;
      if (plan) {
        live += `<div class="eq live">${row(`J = ${num(ctrl.lastCost, 1)},\\quad \\bar ${u}_0 = ${num(plan.U[0])}${unit(sys.uUnit)},\\quad ${u} = ${num(sim.last.u)}${unit(sys.uUnit)}`)}</div>`;
        live += `<p class="small subtle">${ctrl.solves} plans so far, the last one in ${ctrl.lastIters} iteration${ctrl.lastIters === 1 ? "" : "s"}.</p>`;
      }
    } else if (plan) {
      const c = plan.costs;
      const shown = c.length <= 6 ? c : [...c.slice(0, 4), null, c[c.length - 1]];
      live += `<div class="eq live">${row(`J:\\ ${shown.map((v) => (v === null ? "\\dots" : num(v, 0))).join(" \\to ")}`)}</div>`;
      live += `<p class="eq-note">The cost after each iteration of the last plan: ${plan.iters} iterations, from ${num(c[0], 0)} down to ${num(c[c.length - 1], 0)}.</p>`;
      const tPlan = ctrl.t;
      const following = tPlan < plan.U.length * dt;
      live += following
        ? `<p class="small">Following the plan: ${num(tPlan, 1)} s of ${num(plan.U.length * dt, 1)} s.</p>`
        : `<div class="eq live">${row(`${u} = ${u}^\\star + K_\\infty\\,e,\\quad K_\\infty = (${ctrl.hold.map((k) => num(k, 1)).join(",\\ ")})`)}</div>
           <p class="small">The plan is done, so a steady LQR holds the goal. A kick or a new target starts a fresh plan.</p>`;
    }
    block("#math-opt-live", live);
  }

  return { update };
}
