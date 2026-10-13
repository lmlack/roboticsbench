// Small shared helpers.

/**
 * Evaluates a number typed into a field: plain numbers, + - * / ^, parens,
 * pi, e, deg, and sqrt/sin/cos/tan/abs. A hand-written parser, never eval.
 * Returns NaN for anything it does not understand.
 */
export function evalNumber(src) {
  const s = String(src).trim().toLowerCase().replace(/π/g, "pi").replace(/−/g, "-").replace(/°/g, "*deg");
  if (s === "") return NaN;
  let i = 0;
  const peek = () => s[i];
  const skip = () => { while (s[i] === " ") i++; };
  const FUNCS = { sqrt: Math.sqrt, sin: Math.sin, cos: Math.cos, tan: Math.tan, abs: Math.abs };
  const CONSTS = { pi: Math.PI, e: Math.E, deg: Math.PI / 180 };

  function expr() {
    let v = term();
    for (skip(); peek() === "+" || peek() === "-"; skip()) {
      const op = s[i++];
      const r = term();
      v = op === "+" ? v + r : v - r;
    }
    return v;
  }
  function term() {
    let v = unary();
    for (skip(); peek() === "*" || peek() === "/"; skip()) {
      const op = s[i++];
      const r = unary();
      v = op === "*" ? v * r : v / r;
    }
    return v;
  }
  // Unary minus binds looser than ^, so -2^2 is -4 as on paper.
  function unary() {
    skip();
    if (peek() === "-") { i++; return -unary(); }
    if (peek() === "+") { i++; return unary(); }
    return power();
  }
  function power() {
    const base = atom();
    skip();
    if (peek() === "^") {
      i++;
      return base ** unary();
    }
    return base;
  }
  function atom() {
    skip();
    if (peek() === "(") {
      i++;
      const v = expr();
      skip();
      if (s[i++] !== ")") throw 0;
      return v;
    }
    const num = s.slice(i).match(/^(\d+\.?\d*|\.\d+)(e[-+]?\d+)?/);
    if (num) {
      i += num[0].length;
      return Number(num[0]);
    }
    const word = s.slice(i).match(/^[a-z]+/);
    if (word) {
      i += word[0].length;
      const w = word[0];
      if (w in CONSTS) return CONSTS[w];
      if (w in FUNCS) {
        skip();
        if (s[i++] !== "(") throw 0;
        const v = expr();
        skip();
        if (s[i++] !== ")") throw 0;
        return FUNCS[w](v);
      }
    }
    throw 0;
  }

  try {
    const v = expr();
    skip();
    return i === s.length && Number.isFinite(v) ? v : NaN;
  } catch {
    return NaN;
  }
}

/** Fixed decimals without "-0.000". */
export function fmt(v, digits = 4) {
  const s = v.toFixed(digits);
  return /^-0\.?0*$/.test(s) ? s.slice(1) : s;
}

export function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

export function el(tag, attrs = {}, ...children) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") node.className = v;
    else if (k === "text") node.textContent = v;
    else if (k.startsWith("on")) node.addEventListener(k.slice(2), v);
    else if (v !== false && v !== null && v !== undefined) node.setAttribute(k, v === true ? "" : v);
  }
  for (const c of children) if (c != null) node.append(c);
  return node;
}

/**
 * On a single-column layout, moves `node` to the end of `target`, and puts it
 * back where it came from when the layout widens again. Used to bring a
 * simulation's Play/Run controls up beside the canvas on a phone, where the
 * settings panel otherwise stacks far below it.
 */
export function dockWhenNarrow(node, target, query = "(max-width: 960px)") {
  const home = node.parentNode;
  const next = node.nextSibling;
  const mq = window.matchMedia(query);
  const place = () => {
    node.classList.toggle("docked", mq.matches);
    if (mq.matches) target.append(node);
    else home.insertBefore(node, next);
  };
  mq.addEventListener("change", place);
  place();
}

/** Wires a .segmented group: returns get/set, calls onChange with the value. */
export function segmented(root, onChange) {
  const buttons = [...root.querySelectorAll("button[data-v]")];
  const set = (v) => {
    for (const b of buttons) b.setAttribute("aria-pressed", String(b.dataset.v === v));
  };
  for (const b of buttons) {
    b.addEventListener("click", () => {
      set(b.dataset.v);
      onChange?.(b.dataset.v);
    });
  }
  return {
    get: () => buttons.find((b) => b.getAttribute("aria-pressed") === "true")?.dataset.v,
    set,
  };
}

export async function copyText(text, button) {
  try {
    await navigator.clipboard.writeText(text);
    if (button) {
      const old = button.textContent;
      button.textContent = "Copied";
      setTimeout(() => (button.textContent = old), 1200);
    }
  } catch {
    // Clipboard blocked (insecure context or permissions): nothing to do.
  }
}
