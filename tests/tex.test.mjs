import { test } from "node:test";
import assert from "node:assert/strict";

// tex.js imports KaTeX's browser module, which also runs under node.
const { num, par, sum, mat } = await import("../js/tex.js");

test("number formatting for LaTeX", () => {
  assert.equal(num(-0.0001, 2), "{-1.0 \\times 10^{-4}}");
  assert.equal(num(-0.004, 2), "{-4.0 \\times 10^{-3}}");
  assert.equal(num(-0.001, 3), "-0.001");
  assert.equal(num(0, 2), "0.00");
  assert.equal(num(-0.00001, 0), "{-1.0 \\times 10^{-5}}");
  assert.equal(num(3.14159, 2), "3.14");
  assert.equal(par(-2.5), "(-2.50)");
  assert.equal(par(2.5), "2.50");
});

test("a unit exponent after a tiny number is not a double superscript", async () => {
  const katex = (await import("../vendor/katex/katex.mjs")).default;
  // num() braces scientific notation, so "{…}^\\circ" always parses.
  assert.doesNotThrow(() => katex.renderToString(`${num(0.074, 1)}^\\circ`, { throwOnError: true }));
});

test("sums never show '+ -'", () => {
  assert.equal(sum([43.81, 0, -14.46]), "43.81 + 0.00 - 14.46");
  assert.equal(sum([[1, "s^3"], [-5.15, "s^2"], [12, "s"], [6]]), "1.00\\,s^3 - 5.15\\,s^2 + 12.00\\,s + 6.00");
  assert.equal(sum([-2, 3]), "-2.00 + 3.00");
});

test("matrices", () => {
  assert.equal(mat([[1, 0], [0, 1]], 1), "\\begin{bmatrix} 1.0 & 0.0 \\\\ 0.0 & 1.0 \\end{bmatrix}");
});
