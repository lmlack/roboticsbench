// Math typesetting for the "The math" panels, via a self-hosted KaTeX.
// Every equation and every live-number line goes through here, so they all
// share one face, real fractions, accents and matrices, and proper minus
// signs. Plain-language explanations stay as ordinary HTML text.
//
// Only strings built in this codebase from numbers reach KaTeX, never user
// text, and trust stays off, so the output is safe for innerHTML.

import katex from "../vendor/katex/katex.mjs";

const OPTS = { throwOnError: false, output: "html", strict: false, trust: false };
const lastSrc = new WeakMap();

/**
 * Render an equation row, skipping the work if nothing changed. Display
 * style gives full-size fractions and integral limits, while the row stays
 * left-aligned like the text around it.
 */
export function tex(el, src) {
  if (!el || lastSrc.get(el) === src) return;
  lastSrc.set(el, src);
  katex.render(`\\displaystyle ${src}`, el, OPTS);
}

/** An equation row as an HTML string, for rows built in bulk. */
export function row(src) {
  return katex.renderToString(`\\displaystyle ${src}`, OPTS);
}

/** LaTeX to an HTML string, for math inside a sentence. */
export function inl(src) {
  return katex.renderToString(src, OPTS);
}

/**
 * A number for LaTeX. Fixed decimals, no "-0.00", and scientific notation
 * when fixed decimals would show only zeros or run long.
 */
export function num(v, d = 2) {
  if (!Number.isFinite(v)) return "\\text{--}";
  const a = Math.abs(v);
  if (a !== 0 && (a < 10 ** -d || a >= 1e5)) {
    const e = Math.floor(Math.log10(a));
    const m = v / 10 ** e;
    // Braced, so a unit exponent after it ({…}^\\circ) cannot become a
    // second superscript on the 10.
    return `{${m.toFixed(Math.max(1, d - 1))} \\times 10^{${e}}}`;
  }
  const s = v.toFixed(d);
  return /^-0\.?0*$/.test(s) ? s.slice(1) : s;
}

/** A number in parentheses when negative, for products like 5 × (−2.9). */
export const par = (v, d = 2) => (v < 0 && num(v, d) !== num(0, d) ? `(${num(v, d)})` : num(v, d));

/**
 * Join terms into a sum without "+ -": [3.1, -14.5] becomes "3.10 - 14.50".
 * Terms are [value, suffix] pairs or plain numbers; suffix is LaTeX such as "s^2".
 */
export function sum(terms, d = 2) {
  return terms
    .map((t, i) => {
      const [v, suffix = ""] = Array.isArray(t) ? t : [t];
      const body = `${num(Math.abs(v), d)}${suffix ? `\\,${suffix}` : ""}`;
      if (i === 0) return v < 0 ? `-${body}` : body;
      return `${v < 0 ? "-" : "+"} ${body}`;
    })
    .join(" ");
}

/** Fixed decimals with no "-0.000": for matrix cells, so columns line up. */
export function fixed(v, d = 3) {
  if (!Number.isFinite(v)) return "\\text{--}";
  const s = v.toFixed(d);
  return /^-0\.?0*$/.test(s) ? s.slice(1) : s;
}

/**
 * A bracketed matrix. rows is an array of arrays of numbers (or LaTeX strings).
 * Cells are fixed-point so the columns line up; scale a small-valued matrix
 * first (see scaledMat) rather than letting cells go scientific one by one.
 */
export function mat(rows, d = 3) {
  const cell = (c) => (typeof c === "string" ? c : fixed(c, d));
  return `\\begin{bmatrix} ${rows.map((r) => r.map(cell).join(" & ")).join(" \\\\ ")} \\end{bmatrix}`;
}

/**
 * A matrix with its power of ten pulled out front when every entry is small,
 * so 0.00021 and 0.00005 read as 10^{-4} [2.10 0.50] rather than a grid of
 * exponents.
 */
export function scaledMat(rows, d = 2) {
  const max = Math.max(...rows.flat().map(Math.abs));
  const e = max > 0 ? Math.floor(Math.log10(max)) : 0;
  if (e >= -1) return mat(rows, 3);
  return `10^{${e}} ${mat(rows.map((r) => r.map((v) => v / 10 ** e)), d)}`;
}

/** A column vector. */
export const vec = (xs, d = 3) => mat(xs.map((x) => [x]), d);

/** A unit after a number, upright and thin-spaced. */
export const unit = (u) => `\\,\\text{${u}}`;
