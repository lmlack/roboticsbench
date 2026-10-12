// Kalman filter demo UI: a cart on a track with the estimate drawn over it,
// live charts of position, error and gain, and the filter's matrices with
// the current numbers, typeset by KaTeX. Everything put into innerHTML is
// built from numbers.

import { PRESETS, PROFILES, Simulation } from "./kalman-sim.js";
import { inl, mat, num, par, tex, unit, vec } from "./tex.js";
import { cssVar, el, evalNumber, fmt, segmented } from "./util.js";

const WINDOW = 10; // seconds shown on the charts
const TRACK_METERS = 12; // visible width of the track
const BLIP_LIFE = 1.5; // seconds a measurement stays on the track

const newSeed = () => (Math.random() * 2 ** 32) >>> 0;

export function start(root) {
  const $ = (sel) => root.querySelector(sel);
  const sim = new Simulation({}, newSeed());
  // Opens at rest: the first frame is drawn, nothing moves until Play.
  let paused = true;
  let speed = 1;
  let active = false;
  let raf = 0;
  let colors = readColors();
  window.addEventListener("rb:theme", () => (colors = readColors()));

  function readColors() {
    const c = (n) => cssVar(n);
    return {
      bg: c("--raised"), panel: c("--panel"), line: c("--line"), lineStrong: c("--line-strong"),
      fg: c("--fg"), muted: c("--fg-muted"), subtle: c("--fg-subtle"),
      truth: c("--plot-setpoint"), est: c("--plot-measured"), meas: c("--plot-control"),
      error: c("--plot-error"), kp: c("--plot-p"), kv: c("--plot-i"),
      bad: c("--bad-fg"), solid: c("--solid"), highlight: c("--highlight"), accent: c("--accent"),
      warnBg: c("--warn-bg"), warnFg: c("--warn-fg"),
    };
  }

  // ---- Motion profile ----
  const profBox = $("#kf-profile");
  for (const [k, p] of Object.entries(PROFILES)) {
    profBox.append(el("button", { type: "button", "data-v": k, "aria-pressed": String(k === sim.params.profile), text: p.name }));
  }
  segmented(profBox, (k) => {
    sim.set({ profile: k });
    restart();
  });

  // ---- Sliders with number fields ----
  // Log sliders hold log10 of the value, so 0.001 and 100 are both reachable.
  const sig = (v) => String(Number(v.toPrecision(3)));
  const params = {
    rate: { id: "kf-rate", show: (v) => fmt(v, 0), ok: (v) => v >= 0.5 && v <= 200 },
    sigmaZ: { id: "kf-sz", show: (v) => fmt(v, 2), ok: (v) => v > 0 && v <= 20 },
    sigmaV: { id: "kf-sv", show: (v) => fmt(v, 2), ok: (v) => v > 0 && v <= 20 },
    filterSigmaZ: { id: "kf-fsz", log: true, show: sig, ok: (v) => v > 0 && v <= 1000 },
    filterQ: { id: "kf-fq", log: true, show: sig, ok: (v) => v > 0 && v <= 1e5 },
  };
  for (const [key, p] of Object.entries(params)) {
    p.slider = $(`#${p.id}`);
    p.num = $(`#${p.id}-num`);
    p.slider.addEventListener("input", () => {
      const s = Number(p.slider.value);
      setParam(key, p.log ? 10 ** s : s, "slider");
    });
    p.num.addEventListener("change", () => {
      const v = evalNumber(p.num.value);
      if (Number.isFinite(v) && p.ok(v)) setParam(key, key === "rate" ? Math.round(v * 2) / 2 : v, "num");
      else p.num.value = p.show(sim.params[key]);
    });
  }
  function setParam(key, v, from) {
    sim.set({ [key]: v });
    syncParam(key, from);
    markPreset();
  }
  function syncParam(key, from = null) {
    const p = params[key];
    const v = sim.params[key];
    if (from !== "slider") {
      // Typed values outside the slider range stretch the range.
      const s = p.log ? Math.log10(v) : v;
      if (s > Number(p.slider.max)) p.slider.max = s;
      if (s < Number(p.slider.min)) p.slider.min = s;
      p.slider.value = s;
    }
    if (from !== "num") p.num.value = p.show(v);
    if (key === "filterSigmaZ") $("#kf-r-val").textContent = sig(v * v);
  }
  const syncAll = () => Object.keys(params).forEach((k) => syncParam(k));

  // ---- Presets ----
  const presetBox = $("#kf-presets");
  for (const [k, p] of Object.entries(PRESETS)) {
    presetBox.append(el("button", {
      type: "button", class: "btn", "data-preset": k, text: p.name,
      onclick: () => {
        sim.set(p.apply(sim.params));
        syncParam("filterSigmaZ");
        syncParam("filterQ");
        markPreset();
      },
    }));
  }
  function markPreset() {
    for (const b of presetBox.querySelectorAll("[data-preset]")) {
      const want = PRESETS[b.dataset.preset].apply(sim.params);
      const on = Object.entries(want).every(([k, v]) => Math.abs(v - sim.params[k]) <= 1e-9 * Math.max(1, v));
      b.classList.toggle("solid", on);
      b.setAttribute("aria-pressed", String(on));
    }
  }

  // ---- Toggles and buttons ----
  $("#kf-fuse").addEventListener("change", (e) => {
    sim.set({ fuseVel: e.target.checked });
    $("#kf-sv-row").hidden = !e.target.checked;
  });
  $("#kf-drop").addEventListener("change", (e) => sim.set({ dropout: e.target.checked }));
  const pauseBtn = $("#kf-pause");
  function setPaused(p) {
    paused = p;
    pauseBtn.textContent = paused ? "Play" : "Pause";
    // While at rest, Play is the thing to press, so it gets the solid style.
    pauseBtn.classList.toggle("solid", paused);
  }
  pauseBtn.addEventListener("click", () => setPaused(!paused));
  setPaused(true);
  $("#kf-reset").addEventListener("click", () => {
    // A fresh seed, so Reset shows a new run rather than a replay.
    sim.seed = newSeed();
    sim.reset();
    restart();
  });
  segmented($("#kf-speed"), (v) => (speed = Number(v)));

  function restart() {
    sim.warmUp();
    $("#kf-blurb").textContent = PROFILES[sim.params.profile].blurb;
    camX = null;
    steerTouched = false;
    syncSteerMode();
    chartRange.pos = null;
    chartRange.err = null;
    chartRange.kv = null;
    updateReadouts();
    updateMath();
  }

  // ---- Canvas sizing ----
  const stage = $("#kf-canvas");
  const charts = $("#kf-charts");
  const sizes = new Map();
  for (const c of [stage, charts]) {
    new ResizeObserver(() => {
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      c.width = Math.round(c.clientWidth * dpr);
      c.height = Math.round(c.clientHeight * dpr);
      sizes.set(c, { w: c.clientWidth, h: c.clientHeight, dpr });
    }).observe(c);
  }
  const ctxOf = (c) => {
    const s = sizes.get(c);
    if (!s || !s.w) return null;
    const ctx = c.getContext("2d");
    ctx.setTransform(s.dpr, 0, 0, s.dpr, 0, 0);
    return { ctx, w: s.w, h: s.h };
  };
  const FONT = "15px Hyper, system-ui, sans-serif";
  const FONT_BOLD = "600 15px Hyper, system-ui, sans-serif";

  // ---- Steering: the "You steer" profile ----
  // The cart is dragged by pointer or arrow keys. The target is a plain
  // simulation parameter, so the physics stays in kalman-sim.js and the filter
  // learns about the maneuver the same way it learns about everything else:
  // from a noisy position reading, one sample late.
  const wrap = $(".kf-track-wrap");
  const steerMode = () => sim.params.profile === "drive";
  // Keep the cart on screen: the track shows 12 m and the view does not pan
  // while steering, so the far ends are half a cart in from the edge.
  const STEER_LIMIT = TRACK_METERS / 2 - 0.8;
  let steerTouched = false;
  let steering = false;

  const setSteer = (m) => sim.set({ driveTarget: Math.max(-STEER_LIMIT, Math.min(STEER_LIMIT, m)) });

  /** Screen x to meters. The view is fixed while steering, so this is stable. */
  function toMeters(clientX) {
    const r = stage.getBoundingClientRect();
    if (!r.width) return 0;
    return ((clientX - r.left) / r.width - 0.5) * TRACK_METERS;
  }

  // Steering a frozen cart does nothing, so an explicit grab starts the run.
  function beginSteer() {
    steerTouched = true;
    if (paused) setPaused(false);
  }

  stage.addEventListener("pointerdown", (e) => {
    if (!steerMode()) return;
    steering = true;
    stage.setPointerCapture(e.pointerId);
    beginSteer();
    setSteer(toMeters(e.clientX));
    e.preventDefault();
  });
  stage.addEventListener("pointermove", (e) => {
    if (steering) setSteer(toMeters(e.clientX));
  });
  for (const ev of ["pointerup", "pointercancel"]) {
    stage.addEventListener(ev, () => (steering = false));
  }
  stage.addEventListener("keydown", (e) => {
    if (!steerMode()) return;
    const step = e.shiftKey ? 1.5 : 0.5;
    const d = e.key === "ArrowLeft" ? -step : e.key === "ArrowRight" ? step : 0;
    if (!d && e.key !== "Home") return;
    beginSteer();
    setSteer(e.key === "Home" ? 0 : sim.params.driveTarget + d);
    e.preventDefault();
  });

  function syncSteerMode() {
    const on = steerMode();
    wrap.classList.toggle("steerable", on);
    $(".kf-steer-key").hidden = !on;
    // Only a tab stop when there is something to do with it.
    if (on) {
      stage.tabIndex = 0;
      stage.setAttribute("role", "application");
      stage.setAttribute("aria-label", "Cart on a track. Drag or use the left and right arrow keys to steer it; Home centers it.");
    } else {
      stage.removeAttribute("tabindex");
      stage.removeAttribute("role");
      stage.setAttribute("aria-label", "Cart on a track with the filter's estimate");
      steering = false;
    }
  }

  // ---- Drawing: the track ----
  let camX = null;

  function drawTrack() {
    const c = ctxOf(stage);
    if (!c) return;
    const { ctx, w, h } = c;
    ctx.clearRect(0, 0, w, h);
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.font = FONT;
    const now = sim.history[sim.history.length - 1];
    const scale = w / TRACK_METERS;
    // The camera follows the truth loosely, so motion still reads as motion.
    // While steering it stays put instead: a panning view would slide the
    // meter under your finger out from under it.
    if (steerMode()) {
      camX = 0;
    } else {
      if (camX === null) camX = now.p;
      camX += (now.p - camX) * 0.05;
      if (Math.abs(now.p - camX) > TRACK_METERS * 0.4) camX = now.p - Math.sign(now.p - camX) * TRACK_METERS * 0.4;
    }
    const toX = (m) => w / 2 + (m - camX) * scale;

    const groundY = Math.round(h * 0.45);
    const bandY = groundY + Math.max(46, h * 0.16);
    const blipY = bandY + Math.max(28, h * 0.12);

    // Track with meter marks
    ctx.strokeStyle = colors.lineStrong;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(0, groundY);
    ctx.lineTo(w, groundY);
    ctx.stroke();
    ctx.fillStyle = colors.subtle;
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    ctx.lineWidth = 1;
    ctx.strokeStyle = colors.lineStrong;
    const first = Math.floor(camX - TRACK_METERS / 2) - 1;
    for (let m = first; m <= camX + TRACK_METERS / 2 + 1; m++) {
      const x = toX(m);
      ctx.beginPath();
      ctx.moveTo(x, groundY);
      ctx.lineTo(x, groundY + (m % 2 === 0 ? 10 : 5));
      ctx.stroke();
      // A label centered on the edge is cut in half, which is obvious once the
      // view stops panning. The tick alone carries it there.
      if (m % 2 === 0 && x > 26 && x < w - 26) ctx.fillText(`${m} m`, x, groundY + 13);
    }

    // Measurement blips, fading with age
    for (const m of sim.meas) {
      const age = sim.t - m.t;
      if (age > BLIP_LIFE || age < 0) continue;
      ctx.globalAlpha = 1 - age / BLIP_LIFE;
      ctx.fillStyle = colors.meas;
      ctx.beginPath();
      ctx.arc(toX(m.z), blipY, age < 0.05 ? 6 : 4, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.globalAlpha = 1;

    // ±2σ bar under the track, centered on the estimate
    const ex = toX(now.pe);
    const x0 = toX(now.pe - 2 * now.sp), x1 = toX(now.pe + 2 * now.sp);
    ctx.fillStyle = colors.est;
    ctx.globalAlpha = 0.2;
    ctx.fillRect(x0, bandY - 6, x1 - x0, 12);
    ctx.globalAlpha = 1;
    ctx.strokeStyle = colors.est;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x0, bandY);
    ctx.lineTo(x1, bandY);
    for (const x of [x0, x1]) {
      ctx.moveTo(x, bandY - 9);
      ctx.lineTo(x, bandY + 9);
    }
    ctx.stroke();

    // Truth marker: a line from the cart through the bar. It turns red when
    // the truth sits outside the band the filter claims.
    const tx = toX(now.p);
    const outside = Math.abs(now.pe - now.p) > 2 * now.sp;
    ctx.strokeStyle = outside ? colors.bad : colors.truth;
    ctx.lineWidth = outside ? 2.5 : 1.5;
    ctx.beginPath();
    ctx.moveTo(tx, groundY + 2);
    ctx.lineTo(tx, bandY + 12);
    ctx.stroke();

    // Estimate marker
    ctx.strokeStyle = colors.est;
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(ex, groundY - 10);
    ctx.lineTo(ex, bandY);
    ctx.stroke();
    ctx.setLineDash([]);

    const label = `±2σ = ±${fmt(2 * now.sp, 2)} m${outside ? ", truth outside" : ""}`;
    ctx.font = FONT;
    const lw = ctx.measureText(label).width;
    const lx = x1 + 12 + lw + 12 < w ? x1 + 12 + lw / 2 + 7 : x0 - 12 - lw / 2 - 7 > 0 ? x0 - 12 - lw / 2 - 7 : w / 2;
    pill(ctx, label, lx, lx === w / 2 ? bandY + 22 : bandY, outside ? colors.bad : colors.est);

    // Carts: truth solid, estimate as a dashed ghost on top
    const cw = Math.max(44, Math.min(80, 0.8 * scale)), ch = 26;
    const cy = groundY - 10 - ch;

    // Steer handle: where your hand is pulling. The gap between the handle and
    // the cart is the maneuver the filter has not been told about yet.
    if (steerMode()) {
      const hx = toX(sim.params.driveTarget);
      const hy = cy - 18;
      ctx.strokeStyle = colors.accent;
      ctx.lineWidth = 2;
      if (Math.abs(hx - tx) > cw / 2) {
        ctx.setLineDash([3, 4]);
        ctx.beginPath();
        ctx.moveTo(hx, hy);
        ctx.lineTo(tx, hy);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      ctx.fillStyle = colors.accent;
      ctx.beginPath();
      ctx.moveTo(hx, hy + 11);
      ctx.lineTo(hx - 7, hy - 1);
      ctx.lineTo(hx + 7, hy - 1);
      ctx.closePath();
      ctx.fill();
      ctx.beginPath();
      ctx.arc(hx, hy - 7, 5, 0, Math.PI * 2);
      ctx.fill();
      if (!steerTouched) pill(ctx, "Drag the track to steer", w / 2, Math.max(24, hy - 30), colors.accent);
    }
    ctx.fillStyle = colors.highlight;
    roundRect(ctx, tx - cw / 2, cy, cw, ch, 6);
    ctx.fill();
    ctx.strokeStyle = colors.fg;
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.fillStyle = colors.fg;
    for (const k of [-1, 1]) {
      ctx.beginPath();
      ctx.arc(tx + k * cw * 0.3, groundY - 6, 6, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.fillStyle = colors.est;
    ctx.globalAlpha = 0.14;
    roundRect(ctx, ex - cw / 2, cy - 4, cw, ch + 8, 8);
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = colors.est;
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 4]);
    ctx.stroke();
    ctx.setLineDash([]);

    const badge = $("#kf-badge");
    if (now.drop) {
      badge.textContent = "Sensor dropout: no measurements";
      badge.className = "sim-badge bad";
    } else {
      badge.textContent = `${fmt(sim.params.rate, sim.params.rate % 1 ? 1 : 0)} Hz${sim.params.fuseVel ? ", position and velocity" : ""}`;
      badge.className = "sim-badge";
    }
  }

  function pill(ctx, text, x, y, color) {
    ctx.save();
    ctx.font = FONT;
    const tw = ctx.measureText(text).width;
    ctx.fillStyle = colors.panel;
    ctx.globalAlpha = 0.9;
    roundRect(ctx, x - tw / 2 - 7, y - 11, tw + 14, 22, 11);
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.fillStyle = color;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, x, y);
    ctx.restore();
  }

  function roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  // ---- Drawing: charts ----
  const chartRange = { pos: null, err: null, kv: null };
  const ease = (prev, target) => (prev ? { lo: prev.lo + (target.lo - prev.lo) * 0.12, hi: prev.hi + (target.hi - prev.hi) * 0.12 } : target);
  const mapY = (v, p, r) => p.y1 - ((v - r.lo) / (r.hi - r.lo)) * (p.y1 - p.y0);

  function drawCharts() {
    const c = ctxOf(charts);
    if (!c) return;
    const { ctx, w, h } = c;
    ctx.clearRect(0, 0, w, h);
    ctx.font = FONT;
    const now = sim.t;
    const t0 = now - WINDOW;
    const data = sim.history.filter((p) => p.t >= t0);
    const meas = sim.meas.filter((m) => m.t >= t0);
    const gains = sim.gains.filter((g) => g.t >= t0);
    // Room on the right for the velocity gain axis.
    const left = 58, right = w - 46;
    const gap = 30, top = 22, axisH = 22;
    const avail = h - top - axisH - gap * 2;
    const panels = [];
    let y = top;
    for (const f of [0.4, 0.34, 0.26]) {
      panels.push({ y0: y, y1: y + f * avail });
      y += f * avail + gap;
    }
    const tx = (t) => right - ((now - t) / WINDOW) * (right - left);

    // Dropout stretches, shaded in every panel
    const gaps = [];
    for (const p of data) {
      if (!p.drop) continue;
      const g = gaps[gaps.length - 1];
      if (g && p.t - g.t1 < 0.05) g.t1 = p.t;
      else gaps.push({ t0: p.t, t1: p.t });
    }

    // Position: truth, measurements, estimate with its band
    {
      let lo = Infinity, hi = -Infinity;
      for (const p of data) {
        lo = Math.min(lo, p.p, p.pe - 2 * p.sp);
        hi = Math.max(hi, p.p, p.pe + 2 * p.sp);
      }
      for (const m of meas) {
        lo = Math.min(lo, m.z);
        hi = Math.max(hi, m.z);
      }
      if (!data.length) [lo, hi] = [-1, 1];
      if (hi - lo < 2) {
        const mid = (hi + lo) / 2;
        [lo, hi] = [mid - 1, mid + 1];
      }
      const pad = (hi - lo) * 0.08;
      chartRange.pos = ease(chartRange.pos, { lo: lo - pad, hi: hi + pad });
      const r = chartRange.pos;
      const P = panels[0];
      panel(ctx, P, left, right, r, "Position", [["m", colors.subtle]]);
      shadeGaps(ctx, gaps, P, tx, left, right, true);
      band(ctx, data, (p) => p.pe - 2 * p.sp, (p) => p.pe + 2 * p.sp, P, r, tx, left, right);
      series(ctx, data, (p) => p.p, P, r, tx, colors.truth, 2);
      clip(ctx, P, left, right, () => {
        ctx.fillStyle = colors.meas;
        for (const m of meas) {
          ctx.beginPath();
          ctx.arc(tx(m.t), mapY(m.z, P, r), 2.5, 0, Math.PI * 2);
          ctx.fill();
        }
      });
      series(ctx, data, (p) => p.pe, P, r, tx, colors.est, 2.5);
    }

    // Error with the band around zero: the consistency check
    let inside = 0;
    {
      let m = 0;
      for (const p of data) {
        const e = Math.abs(p.pe - p.p);
        m = Math.max(m, e, 2 * p.sp);
        if (e <= 2 * p.sp) inside++;
      }
      m = Math.max(m * 1.12, 0.05);
      chartRange.err = ease(chartRange.err, { lo: -m, hi: m });
      const r = chartRange.err;
      const P = panels[1];
      panel(ctx, P, left, right, r, "Error", [["estimate − truth, m", colors.subtle]], true);
      shadeGaps(ctx, gaps, P, tx, left, right, false);
      band(ctx, data, (p) => -2 * p.sp, (p) => 2 * p.sp, P, r, tx, left, right);
      series(ctx, data, (p) => p.pe - p.p, P, r, tx, colors.error, 2);
      if (data.length) {
        const text = `${fmt((100 * inside) / data.length, 0)}% inside ±2σ`;
        ctx.font = FONT_BOLD;
        const tw = ctx.measureText(text).width;
        ctx.font = FONT;
        const titleEnd = left + ctx.measureText("Error  estimate − truth, m").width + 24;
        if (right - tw > titleEnd) {
          ctx.font = FONT_BOLD;
          ctx.fillStyle = colors.fg;
          ctx.textAlign = "right";
          ctx.textBaseline = "bottom";
          ctx.fillText(text, right, P.y0 - 4);
          ctx.font = FONT;
        }
      }
    }

    // Gain: K for position on the left axis (unitless), K for velocity on
    // the right axis in 1/s, since the two have different units.
    {
      const P = panels[2];
      const r = { lo: -0.08, hi: 1.08 };
      let kvMax = 0;
      for (const g of gains) if (g.kv !== null) kvMax = Math.max(kvMax, Math.abs(g.kv));
      chartRange.kv = ease(chartRange.kv, { lo: 0, hi: Math.max(0.5, kvMax * 1.15) });
      const rv = { lo: -0.08 * chartRange.kv.hi, hi: 1.08 * chartRange.kv.hi };
      panel(ctx, P, left, right, r, "Gain", [["K for p", colors.kp], ["K for v, 1/s", colors.kv]], false, [0, 0.5, 1]);
      shadeGaps(ctx, gaps, P, tx, left, right, false);
      // Right axis labels
      ctx.fillStyle = colors.kv;
      ctx.textAlign = "left";
      ctx.textBaseline = "middle";
      for (const v of niceTicks(0, chartRange.kv.hi, 2)) {
        const yy = mapY(v, P, rv);
        if (yy < P.y0 + 2 || yy > P.y1 - 2) continue;
        ctx.fillText(trimNum(v), right + 6, yy);
      }
      stepSeries(ctx, gains, (g) => g.kv, P, rv, tx, colors.kv, 2, left, right, now);
      stepSeries(ctx, gains, (g) => g.kp, P, r, tx, colors.kp, 2.5, left, right, now);
    }

    // Time axis
    ctx.fillStyle = colors.subtle;
    ctx.textBaseline = "top";
    for (let s = 0; s <= WINDOW; s += 2) {
      const x = right - (s / WINDOW) * (right - left);
      ctx.textAlign = s === 0 ? "right" : "center";
      ctx.fillText(s === 0 ? "now" : `−${s} s`, x, h - axisH + 4);
    }
  }

  function clip(ctx, p, left, right, draw) {
    ctx.save();
    ctx.beginPath();
    ctx.rect(left, p.y0, right - left, p.y1 - p.y0);
    ctx.clip();
    draw();
    ctx.restore();
  }

  function shadeGaps(ctx, gaps, p, tx, left, right, label) {
    if (!gaps.length) return;
    clip(ctx, p, left, right, () => {
      for (const g of gaps) {
        const x0 = tx(g.t0), x1 = tx(g.t1);
        ctx.fillStyle = colors.warnBg;
        ctx.fillRect(x0, p.y0, x1 - x0, p.y1 - p.y0);
        if (label && x1 - x0 > 70) {
          ctx.fillStyle = colors.warnFg;
          ctx.textAlign = "left";
          ctx.textBaseline = "top";
          ctx.fillText("dropout", x0 + 6, p.y0 + 4);
        }
      }
    });
  }

  function band(ctx, data, lo, hi, p, r, tx, left, right) {
    if (data.length < 2) return;
    const step = Math.max(1, Math.floor(data.length / 600));
    clip(ctx, p, left, right, () => {
      ctx.fillStyle = colors.est;
      ctx.globalAlpha = 0.18;
      ctx.beginPath();
      for (let i = 0; i < data.length; i += step) {
        const x = tx(data[i].t), yy = mapY(hi(data[i]), p, r);
        if (i === 0) ctx.moveTo(x, yy);
        else ctx.lineTo(x, yy);
      }
      for (let i = data.length - 1; i >= 0; i -= step) ctx.lineTo(tx(data[i].t), mapY(lo(data[i]), p, r));
      ctx.closePath();
      ctx.fill();
      ctx.globalAlpha = 1;
    });
    // Thin edges so the band reads in both themes
    for (const get of [lo, hi]) series(ctx, data, get, p, r, tx, colors.est, 1, true);
  }

  function series(ctx, data, get, p, r, tx, color, width, dashed = false) {
    if (data.length < 2) return;
    ctx.save();
    ctx.beginPath();
    ctx.rect(tx(data[0].t) - 2, p.y0, 1e5, p.y1 - p.y0);
    ctx.clip();
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dashed ? [4, 4] : []);
    ctx.beginPath();
    // Decimate to roughly one point per pixel.
    const step = Math.max(1, Math.floor(data.length / 900));
    for (let i = 0; i < data.length; i += step) {
      const x = tx(data[i].t), yy = mapY(get(data[i]), p, r);
      if (i === 0) ctx.moveTo(x, yy);
      else ctx.lineTo(x, yy);
    }
    ctx.stroke();
    ctx.restore();
  }

  /** Gains hold their value until the next update; no line where there was none. */
  function stepSeries(ctx, gains, get, p, r, tx, color, width, left, right, now) {
    clip(ctx, p, left, right, () => {
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.beginPath();
      gains.forEach((g, i) => {
        const v = get(g);
        if (v === null) return;
        const next = gains[i + 1];
        if (next && get(next) === null) return;
        const yy = mapY(v, p, r);
        ctx.moveTo(tx(g.t), yy);
        ctx.lineTo(tx(next ? next.t : now), yy);
        if (next) ctx.lineTo(tx(next.t), mapY(get(next), p, r));
      });
      ctx.stroke();
    });
  }

  function panel(ctx, p, left, right, r, title, labels, zeroLine = false, ticks = null) {
    ctx.fillStyle = colors.panel;
    ctx.fillRect(left, p.y0, right - left, p.y1 - p.y0);
    ctx.strokeStyle = colors.line;
    ctx.lineWidth = 1;
    ctx.strokeRect(left + 0.5, p.y0 + 0.5, right - left - 1, p.y1 - p.y0 - 1);
    ctx.fillStyle = colors.subtle;
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (const v of ticks ?? niceTicks(r.lo, r.hi, 3)) {
      const yy = mapY(v, p, r);
      if (yy < p.y0 + 2 || yy > p.y1 - 2) continue;
      ctx.strokeStyle = colors.line;
      ctx.beginPath();
      ctx.moveTo(left, yy);
      ctx.lineTo(right, yy);
      ctx.stroke();
      ctx.fillStyle = ticks ? colors.kp : colors.subtle;
      ctx.fillText(trimNum(v), left - 6, yy);
    }
    if (zeroLine && r.lo < 0 && r.hi > 0) {
      ctx.strokeStyle = colors.lineStrong;
      ctx.lineWidth = 1.5;
      const yy = mapY(0, p, r);
      ctx.beginPath();
      ctx.moveTo(left, yy);
      ctx.lineTo(right, yy);
      ctx.stroke();
    }
    ctx.fillStyle = colors.fg;
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    ctx.font = FONT_BOLD;
    ctx.fillText(title, left, p.y0 - 4);
    let x = left + ctx.measureText(title).width + 8;
    ctx.font = FONT;
    for (const [text, color] of labels) {
      ctx.fillStyle = color;
      ctx.fillText(text, x, p.y0 - 4);
      x += ctx.measureText(text).width + 12;
    }
  }

  function niceTicks(lo, hi, n) {
    const raw = (hi - lo) / n;
    const mag = 10 ** Math.floor(Math.log10(raw));
    const stepSize = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= raw) ?? 10 * mag;
    const out = [];
    for (let v = Math.ceil(lo / stepSize) * stepSize; v <= hi + 1e-9; v += stepSize) out.push(Math.abs(v) < stepSize * 1e-6 ? 0 : v);
    return out;
  }
  const trimNum = (v) => String(Number(v.toPrecision(4)));

  // ---- Metrics ----
  function updateReadouts() {
    const m = sim.metrics(WINDOW);
    const set = (id, text, pending = false) => {
      const node = $(id);
      node.textContent = text;
      node.classList.toggle("pending", pending);
    };
    const enough = m.span >= 2;
    set("#kf-m-meas", m.measRmse !== null ? `${fmt(m.measRmse, 3)} m` : "no data", m.measRmse === null);
    set("#kf-m-est", enough ? `${fmt(m.estRmse, 3)} m` : "measuring", !enough);
    set("#kf-m-cov", enough ? `${fmt(m.coverage * 100, 0)} %` : "measuring", !enough);
    set("#kf-m-sig", `${fmt(m.sigma, 3)} m`);
    $("#kf-m-note").textContent = `Over the last ${fmt(Math.min(WINDOW, m.span), 0)} s. RMSE is the root mean square error against the truth. Inside ±2σ should be near 95% when the filter's assumptions match reality.`;
  }

  // ---- The math ----
  // The panel is built here rather than in the page markup, so equations and
  // live rows share one source. Only LaTeX built from numbers reaches tex().
  const eq = (id, cls = "eq") => `<div class="${cls}" id="kf-math-${id}"></div>`;
  $("#kf-math .math-grid").innerHTML = `
    <div class="math-block">
      <p class="eyebrow">Model</p>
      ${eq("x")}
      <p class="eq-note">Position and velocity.</p>
      ${eq("fh")}${eq("f-live", "eq live")}${eq("h-live", "eq live")}
      ${eq("q")}${eq("q-live", "eq live")}${eq("r-live", "eq live")}
      <p class="eq-note" id="kf-math-model-note"></p>
    </div>
    <div class="math-block">
      <p class="eyebrow">Predict</p>
      ${eq("xp")}${eq("xp-live", "eq live")}
      ${eq("pp")}${eq("pp-live", "eq live")}
      <p class="eq-note">Prediction always adds Q, so uncertainty grows until a measurement arrives.</p>
    </div>
    <div class="math-block">
      <p class="eyebrow">Update</p>
      ${eq("y")}${eq("y-live", "eq live")}
      ${eq("s")}${eq("s-live", "eq live")}
      ${eq("k")}${eq("k-live", "eq live")}
      ${eq("xu")}${eq("xu-live", "eq live")}
      ${eq("pu")}${eq("pu-live", "eq live")}
      <p class="math-status" id="kf-math-status"></p>
    </div>
    <div class="math-block">
      <p class="eyebrow">Reading the result</p>
      <p class="small verdict" id="kf-math-gain"></p>
      <p class="small verdict" id="kf-math-check"></p>
      <p class="small subtle" id="kf-math-foot"></p>
    </div>`;
  const T = (id, src) => tex($(`#kf-math-${id}`), src);
  T("x", "x = \\begin{bmatrix} p \\\\ v \\end{bmatrix}");
  T("q", "Q = q \\begin{bmatrix} \\Delta t^3/3 & \\Delta t^2/2 \\\\ \\Delta t^2/2 & \\Delta t \\end{bmatrix}");
  T("xp", "\\hat x^- = F\\,\\hat x");
  T("pp", "P^- = F P F^\\top + Q");
  T("y", "y = z - H\\,\\hat x^-");
  T("s", "S = H P^- H^\\top + R");
  T("k", "K = P^- H^\\top S^{-1}");
  T("xu", "\\hat x = \\hat x^- + K y");
  T("pu", "P = (I - K H)\\,P^-");
  $("#kf-math-foot").innerHTML = `Numbers are from the most recent filter step, in m, m/s and s. The ${inl("\\pm 2\\sigma")} band should hold the true error about 95% of the time when the assumed noise matches the real noise.`;

  // 1×1 matrices print as a plain number, larger ones in brackets.
  const M = (A, d = 3) => (A.length === 1 && A[0].length === 1 ? num(A[0][0], d) : mat(A, d));
  // Q's entries span several powers of ten (dt³/3 next to dt), so neither
  // fixed decimals nor scaledMat keep them all readable. Pull out the power
  // of ten of the smallest entry and show each cell to three significant digits.
  function smallMat(A) {
    const nz = A.flat().map(Math.abs).filter((v) => v > 0);
    const e = nz.length ? Math.floor(Math.log10(Math.min(...nz))) : 0;
    const cell = (v) => String(Number((v / 10 ** e).toPrecision(3)));
    const body = mat(A.map((r) => r.map(cell)));
    return e < 0 ? `10^{${e}} ${body}` : body;
  }

  function updateMath() {
    const L = sim.last;
    const p = sim.params;
    const H = L?.H ?? (p.fuseVel ? [[1, 0], [0, 1]] : [[1, 0]]);
    const rz = p.filterSigmaZ ** 2;
    const R = L?.R ?? (p.fuseVel ? [[rz, 0], [0, p.sigmaV ** 2]] : [[rz]]);
    const Hsym = p.fuseVel ? "\\begin{bmatrix} 1 & 0 \\\\ 0 & 1 \\end{bmatrix}" : "\\begin{bmatrix} 1 & 0 \\end{bmatrix}";
    T("fh", `F = \\begin{bmatrix} 1 & \\Delta t \\\\ 0 & 1 \\end{bmatrix},\\quad H = ${Hsym}`);
    const live = ["f-live", "h-live", "q-live", "r-live", "xp-live", "pp-live", "y-live", "s-live", "k-live", "xu-live", "pu-live"];
    if (!L) {
      for (const id of live) T(id, "");
      $("#kf-math-status").textContent = "Waiting for the first filter step.";
      $("#kf-math-status").className = "math-status subtle";
      $("#kf-math-gain").textContent = "";
      $("#kf-math-check").textContent = "";
      $("#kf-math-check").className = "small verdict";
      $("#kf-math-model-note").textContent = "";
      return;
    }
    T("f-live", `F = ${mat([["1", num(L.dt, 3)], ["0", "1"]])}`);
    T("h-live", `H = ${mat(H, 0)}`);
    T("q-live", `Q = ${smallMat(L.Q)}`);
    T("r-live", `R = ${R.length === 1 ? num(R[0][0], 4) : mat([[num(R[0][0], 4), "0"], ["0", num(R[1][1], 4)]])}`);
    $("#kf-math-model-note").innerHTML =
      `${inl(`\\Delta t = ${num(L.dt, 3)}${unit("s")}`)} (${fmt(1 / L.dt, 1)} Hz), ${inl(`q = ${num(L.q, 3)}\\,\\text{m}^2/\\text{s}^3`)}, ` +
      `${inl(`R = \\sigma_z^2 = ${num(rz, 4)}\\,\\text{m}^2`)}` +
      (p.fuseVel ? `, velocity ${inl(`\\sigma_v^2 = ${num(p.sigmaV ** 2, 4)}\\,\\text{m}^2/\\text{s}^2`)}.` : ".") +
      " Q is the exact discretization of white-noise acceleration over one step." +
      (p.profile === "drive"
        ? ` Nothing in F or Q knows about your hand. A filter on a real robot adds its own command to the prediction as ${inl("Bu")}; that term is missing here, so every maneuver you make has to be absorbed by Q and discovered through the measurements.`
        : "");

    T("xp-live", `\\hat x^- = ${mat([["1", num(L.dt, 3)], ["0", "1"]])} ${vec(L.xPrev.map((r) => r[0]))} = ${vec(L.xPrior.map((r) => r[0]))}`);
    T("pp-live", `P^- = ${mat(L.PPrior, 4)}`);

    const status = $("#kf-math-status");
    const gainNode = $("#kf-math-gain");
    if (L.dropped) {
      for (const [id, v] of [["y-live", "y"], ["s-live", "S"], ["k-live", "K"]]) T(id, `${v}\\ \\text{not computed: no measurement}`);
      T("xu-live", `\\hat x = \\hat x^- = ${vec(L.x.map((r) => r[0]))}`);
      T("pu-live", `P = P^- = ${mat(L.P, 4)}`);
      status.className = "math-status bad";
      status.textContent = "Dropout: no measurement this step, so there is no update.";
      gainNode.textContent = "Dropout: the filter only predicts. Each step adds Q to P, so the band widens until measurements return. The first update after the gap has a large gain because P is large.";
    } else {
      const Hx = L.xPrior.slice(0, L.H.length);
      // A scalar being subtracted goes in parentheses when negative, so it
      // reads "z - (-7.07)" rather than "z - -7.07".
      const sub = Hx.length === 1 && Hx[0].length === 1 ? par(Hx[0][0], 3) : M(Hx);
      T("y-live", `y = ${M(L.z)} - ${sub} = ${M(L.y)}`);
      T("s-live", `S = ${M(L.S, 4)}`);
      T("k-live", `K = ${M(L.K)}`);
      T("xu-live", `\\hat x = ${vec(L.x.map((r) => r[0]))}`);
      T("pu-live", `P = ${mat(L.P, 4)}`);
      status.className = "math-status subtle";
      status.textContent = p.fuseVel
        ? "Both readings arrived, so y has two entries and S and K are 2×2."
        : "One position reading arrived, so y and S are single numbers.";
      const k = L.K[0][0];
      const lean = k > 0.7 ? "The filter mostly follows the sensor."
        : k < 0.15 ? "The filter mostly follows its model."
        : "The filter weighs the sensor and the model.";
      gainNode.innerHTML =
        `Position gain ${inl(`K_{11} = ${num(k, 3)}`)}: each reading moves the position estimate ${fmt(k * 100, 0)}% of the way to the measurement. Near 1 means trusting the sensor, near 0 means trusting the model. ${lean}` +
        (p.fuseVel ? " With the velocity sensor fused, column 1 of K acts on the position reading and column 2 on the velocity reading." : "");
    }

    const m = sim.metrics(WINDOW);
    const check = $("#kf-math-check");
    const band = inl("\\pm 2\\sigma");
    if (m.span < 3 || m.coverage === null) {
      check.className = "small verdict";
      check.textContent = "Collecting a few seconds of data for the consistency check.";
    } else {
      const pct = `${fmt(m.coverage * 100, 0)}%`;
      if (m.coverage < 0.85) {
        check.className = "small verdict is-bad";
        check.innerHTML = `Overconfident: the error is inside ${band} only ${pct} of the time. P is smaller than the real error, so the filter trusts its estimate more than it should.`;
      } else if (m.coverage > 0.99 && 2 * m.sigma > 2.5 * m.estRmse) {
        check.className = "small verdict";
        check.innerHTML = `Too cautious: the error is inside ${band} ${pct} of the time and the band is much wider than the real error. The estimate may be fine; the stated uncertainty is not.`;
      } else {
        check.className = "small verdict is-ok";
        check.innerHTML = `Consistent: the error is inside ${band} ${pct} of the time, close to the expected 95%.`;
      }
    }
    $("#kf-math-when").textContent = paused ? "paused" : "updates live";
  }

  // ---- Loop ----
  let last = performance.now();
  let readoutClock = 0;
  let mathClock = 0;
  function frame(now) {
    raf = 0;
    if (!active) return;
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    if (!paused) sim.advance(dt * speed);
    drawTrack();
    drawCharts();
    readoutClock += dt;
    mathClock += dt;
    if (readoutClock > 0.1) {
      readoutClock = 0;
      updateReadouts();
    }
    // Slower than the readouts so the matrices can be read while running.
    if (mathClock > 0.4) {
      mathClock = 0;
      updateMath();
    }
    raf = requestAnimationFrame(frame);
  }

  syncAll();
  markPreset();
  restart();

  const tool = {
    setActive(on) {
      active = on;
      if (on && !raf) {
        colors = readColors();
        last = performance.now();
        raf = requestAnimationFrame(frame);
      } else if (!on && raf) {
        cancelAnimationFrame(raf);
        raf = 0;
      }
    },
  };
  tool.setActive(true);
  return tool;
}
