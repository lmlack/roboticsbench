import { test } from "node:test";
import assert from "node:assert/strict";
import { evalNumber, fmt } from "../js/util.js";

test("evalNumber", () => {
  const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-12, `${a} vs ${b}`);
  near(evalNumber("0.5"), 0.5);
  near(evalNumber("-.25"), -0.25);
  near(evalNumber("1e-3"), 0.001);
  near(evalNumber("pi/2"), Math.PI / 2);
  near(evalNumber("π"), Math.PI);
  near(evalNumber("sqrt(2)/2"), Math.SQRT1_2);
  near(evalNumber("-sqrt(2) / 2"), -Math.SQRT1_2);
  near(evalNumber("2*(3+4)"), 14);
  near(evalNumber("2^3^2"), 512);
  near(evalNumber("-2^2"), -4);
  near(evalNumber("2^-1"), 0.5);
  near(evalNumber("90*deg"), Math.PI / 2);
  near(evalNumber("45°"), Math.PI / 4);
  for (const bad of ["", "abc", "1+", "alert(1)", "2**", "(1", "1/0"]) {
    assert.ok(Number.isNaN(evalNumber(bad)), bad);
  }
});

test("fmt", () => {
  assert.equal(fmt(-0.00001, 3), "0.000");
  assert.equal(fmt(-1.5, 2), "-1.50");
});
