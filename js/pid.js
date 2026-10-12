// PID tuning simulator UI: animated plant on a canvas, gain sliders, and a
// live chart of response, error and controller output.

import { SYSTEMS, Simulation } from "./pid-sim.js";
import { cssVar, el, evalNumber, fmt, segmented } from "./util.js";
import { createMathPanel } from "./pid-math.js";

const DEG = Math.PI / 180;
const WINDOW = 10; // seconds shown on the charts
const AUTO_PERIOD = 6;

export function start(root) {
  const $ = (sel) => root.querySelector(sel);
  let key = "drone";
  const sim = new Simulation(key, SYSTEMS[key].presets.tuned);
  let paused = false;
  let speed = 1;
  let active = true;
  let autoOn = false;
  let autoClock = 0;
  let autoFlip = false;
  let colors = readColors();
  const math = createMathPanel(root, sim);
  window.addEventListener("rb:theme", () => (colors = readColors()));

  function readColors() {
    const c = (n) => cssVar(n);
    return {
      bg: c("--raised"), panel: c("--panel"), line: c("--line"), lineStrong: c("--line-strong"),
      fg: c("--fg"), muted: c("--fg-muted"), subtle: c("--fg-subtle"), accent: c("--accent"),
      measured: c("--plot-measured"), setpoint: c("--plot-setpoint"), error: c("--plot-error"),
      control: c("--plot-control"), p: c("--plot-p"), i: c("--plot-i"), d: c("--plot-d"),
      bad: c("--bad-fg"), solid: c("--solid"), highlight: c("--highlight"),
    };
  }

  // ---- System picker ----
  const sysBox = $("#pid-system");
  for (const [k, s] of Object.entries(SYSTEMS)) {
    sysBox.append(el("button", { type: "button", "data-v": k, "aria-pressed": String(k === key), text: s.short, title: s.name }));
  }
  segmented(sysBox, (k) => loadSystem(k));

  // ---- Gains ----
  const gainIds = ["kp", "ki", "kd"];
  const gainUI = Object.fromEntries(gainIds.map((g) => [g, { slider: $(`#g-${g}`), num: $(`#g-${g}-num`) }]));
  for (const g of gainIds) {
    const { slider, num } = gainUI[g];
    slider.addEventListener("input", () => setGain(g, Number(slider.value), "slider"));
    num.addEventListener("change", () => {
      const v = evalNumber(num.value);
      if (Number.isFinite(v) && v >= 0) setGain(g, v, "num");
      else num.value = fmt(sim.pid[g], 2);
    });
  }
  function setGain(g, v, from) {
    sim.pid[g] = v;
    const { slider, num } = gainUI[g];
    if (from !== "slider") {
      // Typed values above the slider range stretch the range.
      if (v > Number(slider.max)) slider.max = v;
      slider.value = v;
    }
    if (from !== "num") num.value = fmt(v, 2);
    markPreset();
  }
  function applyGains(gains) {
    for (const g of gainIds) setGain(g, gains[g], null);
  }

  const presetBox = $("#pid-presets");
  function buildPresets() {
    presetBox.replaceChildren();
    for (const [name, gains] of Object.entries(SYSTEMS[key].presets)) {
      presetBox.append(el("button", {
        type: "button", class: "btn", "data-preset": name, text: name,
        onclick: () => {
          applyGains(gains);
          sim.reset();
          autoClock = 0;
        },
      }));
    }
  }
  function markPreset() {
    for (const b of presetBox.querySelectorAll("[data-preset]")) {
      const p = SYSTEMS[key].presets[b.dataset.preset];
      const on = gainIds.every((g) => Math.abs(p[g] - sim.pid[g]) < 1e-9);
      b.classList.toggle("solid", on);
    }
  }

  // ---- Setpoint ----
  const spSlider = $("#pid-sp");
  const spVal = $("#pid-sp-val");
  spSlider.addEventListener("input", () => setSetpointDisplay(Number(spSlider.value)));
  function setSetpointDisplay(v) {
    const sys = SYSTEMS[key];
    const clamped = Math.min(sys.setpoint.max, Math.max(sys.setpoint.min, v));
    sim.setSetpoint(sys.fromDisplay(clamped));
    syncSetpoint();
  }
  function syncSetpoint() {
    const sys = SYSTEMS[key];
    const v = sys.display(sim.sp);
    spSlider.value = v;
    spVal.textContent = `${fmt(v, sys.unit === "m" ? 1 : 0)} ${sys.unit}`;
  }

  $("#pid-auto").addEventListener("change", (e) => {
    autoOn = e.target.checked;
    autoClock = AUTO_PERIOD; // act right away
  });

  // ---- Buttons and toggles ----
  $("#pid-kick").addEventListener("click", () => sim.kick());
  const pauseBtn = $("#pid-pause");
  pauseBtn.addEventListener("click", () => {
    paused = !paused;
    pauseBtn.textContent = paused ? "Play" : "Pause";
  });
  $("#pid-reset").addEventListener("click", () => {
    sim.reset();
    syncSetpoint();
    autoClock = 0;
  });
  segmented($("#pid-speed"), (v) => (speed = Number(v)));
  $("#pid-aw").addEventListener("change", (e) => (sim.pid.antiWindup = e.target.checked));
  $("#pid-dmeas").addEventListener("change", (e) => (sim.pid.derivOnMeasurement = e.target.checked));
  $("#pid-noise").addEventListener("change", (e) => (sim.noise = e.target.checked));

  // ---- Term bars ----
  const termBox = $("#pid-terms");
  const termRows = {};
  for (const [k, label, color] of [["p", "P", "--plot-p"], ["i", "I", "--plot-i"], ["d", "D", "--plot-d"], ["u", "u", "--plot-control"]]) {
    const fill = el("div", { class: "fill", style: `background:var(${color})` });
    const num = el("span", { class: "num" });
    termBox.append(el("div", { class: "term" }, el("span", { text: label, style: k === "u" ? "font-weight:600" : "" }), el("div", { class: "bar" }, fill), num));
    termRows[k] = { fill, num };
  }
  const satNote = el("div", { class: "small subtle", style: "margin-top:4px" });
  termBox.after(satNote);

  // ---- Load a system ----
  function loadSystem(k) {
    key = k;
    const sys = SYSTEMS[k];
    sim.load(k);
    $("#pid-blurb").textContent = sys.blurb;
    for (const g of gainIds) {
      gainUI[g].slider.min = sys.gains[g][0];
      gainUI[g].slider.max = sys.gains[g][1];
    }
    buildPresets();
    applyGains(sys.presets.tuned);
    sim.reset();
    const fixed = sys.setpoint.min === sys.setpoint.max;
    $("#pid-sp").closest(".gain").hidden = fixed;
    $("#pid-auto-label").textContent = fixed ? "Kick the pole every 6 s" : "Step the setpoint every 6 s";
    spSlider.min = sys.setpoint.min;
    spSlider.max = sys.setpoint.max;
    spSlider.step = sys.setpoint.step;
    syncSetpoint();
    $("#pid-hint").textContent = {
      drone: "Drag up or down to move the target",
      arm: "Click or drag to set the target",
      cartpole: "Click either side to nudge the pole",
    }[k];
    autoClock = 0;
    math.load(k);
    chartRange.response = null;
    chartRange.error = null;
  }

  // ---- Canvas sizing ----
  const stage = $("#pid-canvas");
  const charts = $("#pid-charts");
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

  // ---- Stage interaction ----
  let dragging = false;
  stage.addEventListener("pointerdown", (e) => {
    dragging = true;
    stage.setPointerCapture(e.pointerId);
    pointer(e, true);
  });
  stage.addEventListener("pointermove", (e) => dragging && pointer(e, false));
  stage.addEventListener("pointerup", () => (dragging = false));
  stage.addEventListener("pointercancel", () => (dragging = false));

  function pointer(e, first) {
    const r = stage.getBoundingClientRect();
    const x = e.clientX - r.left;
    const y = e.clientY - r.top;
    const { w, h } = sizes.get(stage) ?? { w: r.width, h: r.height };
    if (key === "drone") {
      const g = droneGeom(w, h);
      setSetpointDisplay(g.toAlt(y));
    } else if (key === "arm") {
      const g = armGeom(w, h);
      const ang = Math.atan2(x - g.cx, y - g.cy) / DEG;
      setSetpointDisplay(Math.round(ang));
    } else if (key === "cartpole" && first) {
      sim.state.w += x < w / 2 ? -0.6 : 0.6;
      sim.markEvent();
    }
  }

  // ---- Drawing: plants ----
  const droneGeom = (w, h) => {
    const top = 28, bottom = h - 36;
    const maxAlt = 10;
    return {
      ground: bottom,
      toY: (alt) => bottom - (alt / maxAlt) * (bottom - top),
      toAlt: (y) => ((bottom - y) / (bottom - top)) * maxAlt,
    };
  };
  const armGeom = (w, h) => ({ cx: w / 2, cy: h * 0.48, L: Math.min(w, h) * 0.34 });

  let camX = 0;

  function drawStage(t) {
    const c = ctxOf(stage);
    if (!c) return;
    const { ctx, w, h } = c;
    ctx.clearRect(0, 0, w, h);
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.font = "15px Hyper, system-ui, sans-serif";
    const sys = SYSTEMS[key];
    const uFrac = sim.last.u / Math.max(Math.abs(sys.uLimit[0]), Math.abs(sys.uLimit[1]));
    if (key === "drone") drawDrone(ctx, w, h, t, uFrac);
    else if (key === "arm") drawArm(ctx, w, h);
    else drawCartpole(ctx, w, h, uFrac);

    const badge = $("#pid-badge");
    const y = sys.display(sys.measure(sim.state));
    if (key === "cartpole" && Math.abs(sim.state.th) > 90 * DEG) {
      badge.textContent = "Fallen. Press Reset.";
      badge.className = "sim-badge bad";
    } else {
      badge.textContent = `${fmt(y, sys.unit === "m" ? 2 : 1)} ${sys.unit}${sim.last.saturated ? "  ·  actuator maxed" : ""}`;
      badge.className = "sim-badge";
    }
  }

  function dashedLine(ctx, x1, y1, x2, y2, color) {
    ctx.save();
    ctx.setLineDash([6, 6]);
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
    ctx.restore();
  }

  function drawDrone(ctx, w, h, t, uFrac) {
    const g = droneGeom(w, h);
    // Altitude scale
    ctx.fillStyle = colors.subtle;
    ctx.strokeStyle = colors.line;
    ctx.lineWidth = 1;
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let a = 0; a <= 10; a += 1) {
      const y = g.toY(a);
      ctx.beginPath();
      ctx.moveTo(48, y);
      ctx.lineTo(w - 16, y);
      ctx.stroke();
      if (a % 2 === 0 && a > 0) ctx.fillText(`${a} m`, 40, y);
    }
    // Ground
    ctx.fillStyle = colors.lineStrong;
    ctx.fillRect(0, g.ground, w, h - g.ground);

    // Target
    const ty = g.toY(sim.sp);
    dashedLine(ctx, 48, ty, w - 16, ty, colors.setpoint);
    ctx.fillStyle = colors.setpoint;
    ctx.textAlign = "right";
    ctx.textBaseline = "bottom";
    ctx.fillText(`target ${fmt(sim.sp, 1)} m`, w - 20, ty - 4);

    // Drone
    const cx = w / 2;
    const cy = g.toY(sim.state.y) - 10;
    const thrust = Math.max(0, uFrac);
    // Downwash, longer with more thrust
    ctx.strokeStyle = colors.control;
    ctx.globalAlpha = 0.55;
    ctx.lineWidth = 2;
    for (const side of [-1, 1]) {
      for (let k = -1; k <= 1; k++) {
        const x = cx + side * 52 + k * 8;
        const len = 8 + thrust * 70;
        const phase = (t * 6 + k * 0.3) % 1;
        ctx.beginPath();
        ctx.moveTo(x, cy + 10 + phase * 6);
        ctx.lineTo(x, cy + 10 + phase * 6 + len * (0.6 + 0.4 * phase));
        ctx.stroke();
      }
    }
    ctx.globalAlpha = 1;
    // Arms and body
    ctx.strokeStyle = colors.fg;
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(cx - 52, cy);
    ctx.lineTo(cx + 52, cy);
    ctx.stroke();
    ctx.fillStyle = colors.solid;
    roundRect(ctx, cx - 24, cy - 9, 48, 18, 6);
    ctx.fill();
    ctx.fillStyle = colors.highlight;
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();
    // Rotors spin faster with thrust
    for (const side of [-1, 1]) {
      const rx = cx + side * 52;
      const spin = Math.abs(Math.cos(t * (8 + thrust * 50) + side));
      ctx.strokeStyle = colors.fg;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(rx, cy);
      ctx.lineTo(rx, cy - 8);
      ctx.stroke();
      ctx.fillStyle = colors.muted;
      ctx.globalAlpha = 0.8;
      ctx.beginPath();
      ctx.ellipse(rx, cy - 9, 6 + spin * 18, 3, 0, 0, Math.PI * 2);
      ctx.fill();
      ctx.globalAlpha = 1;
    }
  }

  function drawArm(ctx, w, h) {
    const sys = SYSTEMS.arm;
    const { cx, cy, L } = armGeom(w, h);
    const th = sim.state.th;
    const lim = sys.uLimit[1];
    const motor = sim.last.u;
    const gravity = sys.gravityTorque(sim.state);

    // Reach circle and angle ticks every 30 degrees
    ctx.strokeStyle = colors.line;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(cx, cy, L, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = colors.subtle;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    for (let a = -150; a <= 180; a += 30) {
      const r = a * DEG;
      ctx.beginPath();
      ctx.moveTo(cx + Math.sin(r) * (L - 6), cy + Math.cos(r) * (L - 6));
      ctx.lineTo(cx + Math.sin(r) * (L + 6), cy + Math.cos(r) * (L + 6));
      ctx.stroke();
      if (a % 90 === 0 || Math.abs(a) === 150) {
        const label = a === 0 ? "0° hangs down" : a === 180 ? "±180°" : `${a}°`;
        ctx.fillText(label, cx + Math.sin(r) * (L + 30), cy + Math.cos(r) * (L + 20));
      }
    }

    // Target ghost, labeled outside the circle
    const sp = sim.sp;
    const gx = cx + Math.sin(sp) * L, gy = cy + Math.cos(sp) * L;
    dashedLine(ctx, cx, cy, gx, gy, colors.setpoint);
    ctx.strokeStyle = colors.setpoint;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(gx, gy, 15, 0, Math.PI * 2);
    ctx.stroke();
    pill(ctx, `target ${Math.round(sp / DEG)}°`, cx + Math.sin(sp) * (L + 52), cy + Math.cos(sp) * (L + 40), colors.setpoint);

    // Torque arcs at the same scale: motor (solid) and gravity (dashed).
    // When the arm holds still, the two are equal and opposite.
    torqueArc(ctx, cx, cy, 36, th, motor / lim, colors.control, false);
    torqueArc(ctx, cx, cy, 50, th, gravity / lim, colors.muted, true);

    // Link and load
    const tx = cx + Math.sin(th) * L, ty = cy + Math.cos(th) * L;
    ctx.strokeStyle = colors.fg;
    ctx.lineWidth = 12;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(tx, ty);
    ctx.stroke();

    // Gravity on the load: always straight down
    ctx.strokeStyle = colors.muted;
    ctx.fillStyle = colors.muted;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(tx, ty + 18);
    ctx.lineTo(tx, ty + 44);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(tx, ty + 52);
    ctx.lineTo(tx - 6, ty + 42);
    ctx.lineTo(tx + 6, ty + 42);
    ctx.fill();
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    ctx.font = "italic 15px Hyper, system-ui, sans-serif";
    ctx.fillText("g", tx + 9, ty + 38);
    ctx.font = "15px Hyper, system-ui, sans-serif";

    ctx.fillStyle = colors.highlight;
    ctx.beginPath();
    ctx.arc(tx, ty, 15, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = colors.solid;
    ctx.beginPath();
    ctx.arc(cx, cy, 20, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = colors.panel;
    ctx.beginPath();
    ctx.arc(cx, cy, 6, 0, Math.PI * 2);
    ctx.fill();

    // Current angle, beside the link so it never sits on the target label
    const side = Math.sin(th) >= 0 ? 1 : -1;
    const lx = cx + Math.sin(th) * L * 0.62 + Math.cos(th) * 26 * side;
    const ly = cy + Math.cos(th) * L * 0.62 - Math.sin(th) * 26 * side;
    pill(ctx, `${fmt(th / DEG, 1)}°`, lx, ly, colors.fg, true);

    drawTorqueLegend(ctx, w, h, motor, gravity, lim);
  }

  /** Arc around the pivot starting at the link, sweep proportional to torque. */
  function torqueArc(ctx, cx, cy, r, th, frac, color, dashed) {
    if (Math.abs(frac) < 0.01) return;
    const base = Math.PI / 2 - th; // canvas angle of the link
    const sweep = -Math.max(-1, Math.min(1, frac)) * Math.PI * 0.9;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = dashed ? 2.5 : 4;
    ctx.setLineDash(dashed ? [5, 4] : []);
    ctx.beginPath();
    ctx.arc(cx, cy, r, base, base + sweep, sweep < 0);
    ctx.stroke();
    ctx.setLineDash([]);
    const end = base + sweep;
    const ex = cx + Math.cos(end) * r, ey = cy + Math.sin(end) * r;
    const dir = end + (sweep < 0 ? -Math.PI / 2 : Math.PI / 2);
    ctx.beginPath();
    ctx.moveTo(ex + Math.cos(dir) * 9, ey + Math.sin(dir) * 9);
    ctx.lineTo(ex + Math.cos(dir + 2.4) * 8, ey + Math.sin(dir + 2.4) * 8);
    ctx.lineTo(ex + Math.cos(dir - 2.4) * 8, ey + Math.sin(dir - 2.4) * 8);
    ctx.fill();
    ctx.restore();
  }

  /**
   * Bottom-left key: what each arc means, live values, and a gauge of motor
   * torque against its limit with a mark for the torque needed to hold here.
   */
  function drawTorqueLegend(ctx, w, h, motor, gravity, lim) {
    const pad = 12, rowH = 22;
    const bw = 236, bh = rowH * 2 + 50;
    const x0 = 10, y0 = h - bh - 10;
    ctx.save();
    ctx.fillStyle = colors.panel;
    ctx.strokeStyle = colors.line;
    ctx.lineWidth = 1;
    roundRect(ctx, x0, y0, bw, bh, 10);
    ctx.globalAlpha = 0.92;
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.stroke();

    const row = (i, label, value, color, dashed) => {
      const y = y0 + pad + rowH * i + rowH / 2 - 2;
      ctx.strokeStyle = color;
      ctx.lineWidth = dashed ? 2.5 : 4;
      ctx.setLineDash(dashed ? [5, 4] : []);
      ctx.beginPath();
      ctx.moveTo(x0 + pad, y);
      ctx.lineTo(x0 + pad + 18, y);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = colors.fg;
      ctx.textAlign = "left";
      ctx.textBaseline = "middle";
      ctx.fillText(label, x0 + pad + 26, y);
      ctx.textAlign = "right";
      ctx.fillText(`${value >= 0 ? "+" : "−"}${fmt(Math.abs(value), 1)} N·m`, x0 + bw - pad, y);
    };
    row(0, "motor", motor, colors.control, false);
    row(1, "gravity", gravity, colors.muted, true);

    // Gauge: motor torque within ±limit; the tick is what holding still needs.
    const gy = y0 + pad + rowH * 2 + 10;
    // Limit numbers sit beside the bar so the "hold" label below never hits them.
    const gx0 = x0 + pad + 30, gx1 = x0 + bw - pad - 30, gmid = (gx0 + gx1) / 2;
    const toX = (v) => gmid + (Math.max(-lim, Math.min(lim, v)) / lim) * (gx1 - gmid);
    ctx.fillStyle = colors.bg;
    roundRect(ctx, gx0, gy, gx1 - gx0, 8, 4);
    ctx.fill();
    ctx.fillStyle = sim.last.saturated ? colors.bad : colors.control;
    const mx = toX(motor);
    ctx.fillRect(Math.min(gmid, mx), gy, Math.abs(mx - gmid), 8);
    ctx.strokeStyle = colors.lineStrong;
    ctx.beginPath();
    ctx.moveTo(gmid, gy - 2);
    ctx.lineTo(gmid, gy + 10);
    ctx.stroke();
    const hx = toX(-gravity);
    ctx.fillStyle = colors.fg;
    ctx.beginPath();
    ctx.moveTo(hx, gy + 9);
    ctx.lineTo(hx - 5, gy + 16);
    ctx.lineTo(hx + 5, gy + 16);
    ctx.fill();
    ctx.fillStyle = colors.subtle;
    ctx.textBaseline = "middle";
    ctx.textAlign = "right";
    ctx.fillText(`−${lim}`, gx0 - 6, gy + 4);
    ctx.textAlign = "left";
    ctx.fillText(`+${lim}`, gx1 + 6, gy + 4);
    ctx.textBaseline = "top";
    ctx.textAlign = "center";
    ctx.fillStyle = sim.last.saturated ? colors.bad : colors.fg;
    ctx.fillText(sim.last.saturated ? "motor maxed out" : "hold", Math.max(gx0 + 16, Math.min(gx1 - 16, hx)), gy + 17);
    ctx.restore();
  }

  /** Text on a small panel-colored pill so it reads over ticks and lines. */
  function pill(ctx, text, x, y, color, bold = false) {
    ctx.save();
    ctx.font = `${bold ? "600 " : ""}15px Hyper, system-ui, sans-serif`;
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

  function drawCartpole(ctx, w, h, uFrac) {
    const s = sim.state;
    const scale = Math.min(w, h * 1.6) / 6; // px per meter
    const groundY = h * 0.78;
    camX += (s.x - camX) * 0.08;
    const toX = (x) => w / 2 + (x - camX) * scale;

    // Track with distance marks, so cart motion is visible while followed
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
    const first = Math.floor((camX - w / 2 / scale) * 2) / 2;
    for (let m = first; m < camX + w / 2 / scale + 0.5; m += 0.5) {
      const x = toX(m);
      ctx.beginPath();
      ctx.moveTo(x, groundY);
      ctx.lineTo(x, groundY + (Number.isInteger(m) ? 12 : 6));
      ctx.stroke();
      if (Number.isInteger(m)) ctx.fillText(`${m} m`, x, groundY + 16);
    }

    // Cart
    const cx = toX(s.x);
    const cw = 0.5 * scale, ch = 0.18 * scale;
    const cy = groundY - ch - 10;
    ctx.fillStyle = colors.solid;
    roundRect(ctx, cx - cw / 2, cy, cw, ch, 6);
    ctx.fill();
    ctx.fillStyle = colors.fg;
    for (const k of [-1, 1]) {
      ctx.beginPath();
      ctx.arc(cx + k * cw * 0.3, groundY - 6, 7, 0, Math.PI * 2);
      ctx.fill();
    }

    // Force arrow
    if (Math.abs(uFrac) > 0.01) {
      const len = uFrac * 110;
      const ay = cy + ch / 2;
      const ax = cx + Math.sign(len) * (cw / 2 + 6);
      ctx.strokeStyle = colors.control;
      ctx.fillStyle = colors.control;
      ctx.lineWidth = 4;
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(ax + len, ay);
      ctx.stroke();
      const sgn = Math.sign(len);
      ctx.beginPath();
      ctx.moveTo(ax + len + sgn * 10, ay);
      ctx.lineTo(ax + len - sgn * 2, ay - 7);
      ctx.lineTo(ax + len - sgn * 2, ay + 7);
      ctx.fill();
    }

    // Upright reference
    const px = cx, py = cy;
    const Lpx = 1 * scale * 0.95;
    dashedLine(ctx, px, py, px, py - Lpx, colors.setpoint);

    // Pole
    const bx = px + Math.sin(s.th) * Lpx, by = py - Math.cos(s.th) * Lpx;
    ctx.strokeStyle = colors.fg;
    ctx.lineWidth = 7;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(bx, by);
    ctx.stroke();
    ctx.fillStyle = colors.highlight;
    ctx.beginPath();
    ctx.arc(bx, by, 13, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = colors.panel;
    ctx.beginPath();
    ctx.arc(px, py, 5, 0, Math.PI * 2);
    ctx.fill();
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
  const chartRange = { response: null, error: null };
  const ease = (prev, target) => (prev ? { lo: prev.lo + (target.lo - prev.lo) * 0.12, hi: prev.hi + (target.hi - prev.hi) * 0.12 } : target);

  function drawCharts() {
    const c = ctxOf(charts);
    if (!c) return;
    const { ctx, w, h } = c;
    ctx.clearRect(0, 0, w, h);
    ctx.font = "15px Hyper, system-ui, sans-serif";
    const sys = SYSTEMS[key];
    const now = sim.t;
    const data = sim.history.filter((p) => p.t >= now - WINDOW);
    const left = 58, right = w - 10;
    const gap = 30, top = 22, axisH = 22;
    const avail = h - top - axisH - gap * 2;
    const heights = [0.36, 0.36, 0.28].map((f) => f * avail);
    const panels = [];
    let y = top;
    for (const ph of heights) {
      panels.push({ y0: y, y1: y + ph });
      y += ph + gap;
    }
    const tx = (t) => right - ((now - t) / WINDOW) * (right - left);
    const disp = sys.display;
    const minSpan = { m: 1, "°": 8 }[sys.unit] ?? 1;

    // Response
    {
      let lo = Infinity, hi = -Infinity;
      for (const p of data) {
        lo = Math.min(lo, disp(p.y), disp(p.sp));
        hi = Math.max(hi, disp(p.y), disp(p.sp));
      }
      if (!data.length) [lo, hi] = [0, 1];
      if (hi - lo < minSpan) {
        const mid = (hi + lo) / 2;
        lo = mid - minSpan / 2;
        hi = mid + minSpan / 2;
      }
      const pad = (hi - lo) * 0.12;
      chartRange.response = ease(chartRange.response, { lo: lo - pad, hi: hi + pad });
      const r = chartRange.response;
      panel(ctx, panels[0], left, right, r, sys.unit, "Response");
      legend(ctx, panels[0], left, [["measured", colors.measured, false], ["setpoint", colors.setpoint, true]]);
      series(ctx, data, (p) => disp(p.sp), panels[0], r, tx, colors.setpoint, 1.5, true);
      series(ctx, data, (p) => disp(p.y), panels[0], r, tx, colors.measured, 2.5);
    }

    // Error, symmetric about zero and shaded so its size reads at a glance
    {
      let m = 0;
      for (const p of data) m = Math.max(m, Math.abs(disp(p.e)));
      m = Math.max(m * 1.15, minSpan / 4);
      chartRange.error = ease(chartRange.error, { lo: -m, hi: m });
      const r = chartRange.error;
      panel(ctx, panels[1], left, right, r, sys.unit, "Error (setpoint − measured)", true);
      const yz = mapY(0, panels[1], r);
      ctx.save();
      ctx.beginPath();
      ctx.rect(left, panels[1].y0, right - left, panels[1].y1 - panels[1].y0);
      ctx.clip();
      ctx.fillStyle = colors.error;
      ctx.globalAlpha = 0.16;
      ctx.beginPath();
      data.forEach((p, i) => {
        const x = tx(p.t), yy = mapY(disp(p.e), panels[1], r);
        if (i === 0) ctx.moveTo(x, yz);
        ctx.lineTo(x, yy);
      });
      if (data.length) ctx.lineTo(tx(data[data.length - 1].t), yz);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
      series(ctx, data, (p) => disp(p.e), panels[1], r, tx, colors.error, 2.5);
      if (data.length) {
        const e = disp(data[data.length - 1].e);
        ctx.fillStyle = colors.error;
        ctx.textAlign = "right";
        ctx.textBaseline = "bottom";
        ctx.font = "600 15px Hyper, system-ui, sans-serif";
        ctx.fillText(`${e >= 0 ? "+" : ""}${fmt(e, sys.unit === "m" ? 3 : 2)} ${sys.unit}`, right, panels[1].y0 - 4);
        ctx.font = "15px Hyper, system-ui, sans-serif";
      }
    }

    // Controller output against its limits
    {
      const [ulo, uhi] = sys.uLimit;
      const span = uhi - ulo;
      const r = { lo: ulo - span * 0.08, hi: uhi + span * 0.08 };
      panel(ctx, panels[2], left, right, r, sys.uUnit, "Controller output", ulo < 0);
      for (const lim of [ulo, uhi]) {
        const yy = mapY(lim, panels[2], r);
        dashedLine(ctx, left, yy, right, yy, colors.lineStrong);
      }
      ctx.fillStyle = colors.subtle;
      ctx.textAlign = "right";
      ctx.textBaseline = "bottom";
      ctx.fillText("limit", right, mapY(uhi, panels[2], r) - 2);
      series(ctx, data, (p) => p.u, panels[2], r, tx, colors.control, 2);
      // Mark saturated stretches along the top edge
      ctx.fillStyle = colors.bad;
      for (const p of data) if (p.sat) ctx.fillRect(tx(p.t) - 1, panels[2].y0, 2, 4);
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

  const mapY = (v, p, r) => p.y1 - ((v - r.lo) / (r.hi - r.lo)) * (p.y1 - p.y0);

  function panel(ctx, p, left, right, r, unit, title, zeroLine = false) {
    ctx.fillStyle = colors.panel;
    ctx.fillRect(left, p.y0, right - left, p.y1 - p.y0);
    ctx.strokeStyle = colors.line;
    ctx.lineWidth = 1;
    ctx.strokeRect(left + 0.5, p.y0 + 0.5, right - left - 1, p.y1 - p.y0 - 1);
    // Three gridlines with labels
    ctx.fillStyle = colors.subtle;
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    const ticks = niceTicks(r.lo, r.hi, 3);
    for (const v of ticks) {
      const y = mapY(v, p, r);
      if (y < p.y0 + 2 || y > p.y1 - 2) continue;
      ctx.strokeStyle = colors.line;
      ctx.beginPath();
      ctx.moveTo(left, y);
      ctx.lineTo(right, y);
      ctx.stroke();
      ctx.fillText(trimNum(v), left - 6, y);
    }
    if (zeroLine && r.lo < 0 && r.hi > 0) {
      ctx.strokeStyle = colors.lineStrong;
      ctx.lineWidth = 1.5;
      const y = mapY(0, p, r);
      ctx.beginPath();
      ctx.moveTo(left, y);
      ctx.lineTo(right, y);
      ctx.stroke();
    }
    ctx.fillStyle = colors.fg;
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    ctx.font = "600 15px Hyper, system-ui, sans-serif";
    ctx.fillText(title, left, p.y0 - 4);
    const tw = ctx.measureText(title).width;
    ctx.font = "15px Hyper, system-ui, sans-serif";
    ctx.fillStyle = colors.subtle;
    ctx.fillText(unit, left + tw + 8, p.y0 - 4);
  }

  function legend(ctx, p, left, items) {
    ctx.save();
    ctx.font = "15px Hyper, system-ui, sans-serif";
    let x = left + 260;
    ctx.textBaseline = "bottom";
    ctx.textAlign = "left";
    for (const [label, color, dashed] of items) {
      ctx.strokeStyle = color;
      ctx.lineWidth = 2.5;
      ctx.setLineDash(dashed ? [5, 4] : []);
      ctx.beginPath();
      ctx.moveTo(x, p.y0 - 10);
      ctx.lineTo(x + 18, p.y0 - 10);
      ctx.stroke();
      ctx.fillStyle = colors.muted;
      ctx.fillText(label, x + 24, p.y0 - 3);
      x += 24 + ctx.measureText(label).width + 18;
    }
    ctx.restore();
  }

  function series(ctx, data, get, p, r, tx, color, width, dashed = false) {
    if (data.length < 2) return;
    ctx.save();
    ctx.beginPath();
    ctx.rect(tx(data[0].t) - 2, p.y0, 1e5, p.y1 - p.y0);
    ctx.clip();
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dashed ? [6, 5] : []);
    ctx.beginPath();
    // Decimate to roughly one point per pixel.
    const step = Math.max(1, Math.floor(data.length / 900));
    for (let i = 0; i < data.length; i += step) {
      const x = tx(data[i].t), y = mapY(get(data[i]), p, r);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.restore();
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

  // ---- Metrics and term bars ----
  function updateReadouts() {
    const sys = SYSTEMS[key];
    const m = sim.metrics();
    const set = (id, text, pending = false) => {
      const node = $(id);
      node.textContent = text;
      node.classList.toggle("pending", pending);
    };
    const noStep = m.riseTime === null && m.overshoot === null;
    set("#m-rise", m.riseTime !== null ? `${fmt(m.riseTime, 2)} s` : noStep ? "n/a" : "rising…", m.riseTime === null);
    set("#m-over", m.overshoot !== null ? `${fmt(m.overshoot, 1)} %` : "n/a", m.overshoot === null);
    set("#m-settle", m.settlingTime !== null ? `${fmt(m.settlingTime, 2)} s` : "settling…", m.settlingTime === null);
    set("#m-sse", m.steadyError !== null ? `${fmt(sys.display(m.steadyError), sys.unit === "m" ? 3 : 2)} ${sys.unit}` : "settling…", m.steadyError === null);

    const lim = Math.max(Math.abs(sys.uLimit[0]), Math.abs(sys.uLimit[1]));
    const sign = sys.sign ?? 1;
    const vals = { p: sign * sim.last.p, i: sign * sim.last.i, d: sign * sim.last.d, u: sim.last.u };
    for (const [k, v] of Object.entries(vals)) {
      const f = Math.max(-1, Math.min(1, v / lim));
      const { fill, num } = termRows[k];
      fill.style.left = `${50 + Math.min(0, f) * 50}%`;
      fill.style.width = `${Math.abs(f) * 50}%`;
      num.textContent = `${fmt(v, 1)}`;
    }
    satNote.textContent = sim.last.saturated
      ? `Actuator at its limit (${sys.uLimit[0]} to ${sys.uLimit[1]} ${sys.uUnit}). Asking for more does nothing.`
      : `Bars are in ${sys.uUnit}, scaled to the actuator limit.`;
  }

  // ---- Loop ----
  let last = performance.now();
  let readoutClock = 0;
  function frame(now) {
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    if (active) {
      if (!paused) {
        const simDt = dt * speed;
        sim.advance(simDt);
        if (autoOn) {
          autoClock += simDt;
          if (autoClock >= AUTO_PERIOD) {
            autoClock = 0;
            autoFlip = !autoFlip;
            const targets = { drone: [3, 7], arm: [30, 100] };
            if (key === "cartpole") {
              sim.state.w += autoFlip ? 0.9 : -0.9;
              sim.markEvent();
            } else {
              setSetpointDisplay(targets[key][autoFlip ? 1 : 0]);
            }
          }
        }
      }
      drawStage(now / 1000);
      drawCharts();
      readoutClock += dt;
      if (readoutClock > 0.1) {
        readoutClock = 0;
        updateReadouts();
        math.update();
      }
    }
    requestAnimationFrame(frame);
  }

  loadSystem(key);
  requestAnimationFrame(frame);

  return {
    setActive(on) {
      active = on;
      last = performance.now();
    },
  };
}
