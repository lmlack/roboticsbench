// Copies the few runtime files the page needs out of node_modules, so the
// site is plain static files with no CDN and no build step.
import { cpSync, mkdirSync, readdirSync, rmSync } from "node:fs";

const copies = [
  ["node_modules/three/build/three.module.js", "vendor/three/build/three.module.js"],
  ["node_modules/three/build/three.core.js", "vendor/three/build/three.core.js"],
  ["node_modules/three/examples/jsm/controls/OrbitControls.js", "vendor/three/examples/jsm/controls/OrbitControls.js"],
  ["node_modules/three/examples/jsm/loaders/STLLoader.js", "vendor/three/examples/jsm/loaders/STLLoader.js"],
  ["node_modules/three/examples/jsm/loaders/ColladaLoader.js", "vendor/three/examples/jsm/loaders/ColladaLoader.js"],
  ["node_modules/three/examples/jsm/loaders/TGALoader.js", "vendor/three/examples/jsm/loaders/TGALoader.js"],
  ["node_modules/three/examples/jsm/loaders/OBJLoader.js", "vendor/three/examples/jsm/loaders/OBJLoader.js"],
  ["node_modules/urdf-loader/src/URDFLoader.js", "vendor/urdf-loader/URDFLoader.js"],
  ["node_modules/urdf-loader/src/URDFClasses.js", "vendor/urdf-loader/URDFClasses.js"],
];
for (const w of [400, 600, 700]) {
  copies.push([
    `node_modules/@fontsource/atkinson-hyperlegible-next/files/atkinson-hyperlegible-next-latin-${w}-normal.woff2`,
    `fonts/hyper-${w}.woff2`,
  ]);
}
copies.push(["node_modules/@fontsource/righteous/files/righteous-latin-400-normal.woff2", "fonts/righteous-400.woff2"]);
for (const w of [400, 600]) {
  copies.push([
    `node_modules/@fontsource/atkinson-hyperlegible-mono/files/atkinson-hyperlegible-mono-latin-${w}-normal.woff2`,
    `fonts/hyper-mono-${w}.woff2`,
  ]);
}

copies.push(["node_modules/katex/dist/katex.mjs", "vendor/katex/katex.mjs"]);
copies.push(["node_modules/katex/dist/katex.min.css", "vendor/katex/katex.min.css"]);

rmSync("vendor", { recursive: true, force: true });
for (const [from, to] of copies) {
  try {
    mkdirSync(to.slice(0, to.lastIndexOf("/")), { recursive: true });
    cpSync(from, to);
  } catch (e) {
    if (!from.endsWith("three.core.js")) throw e; // only split builds have it
  }
}
// KaTeX's stylesheet loads its fonts from ./fonts next to it. Only woff2 is
// needed: every browser this site supports reads it.
mkdirSync("vendor/katex/fonts", { recursive: true });
let fonts = 0;
for (const f of readdirSync("node_modules/katex/dist/fonts")) {
  if (f.endsWith(".woff2")) {
    cpSync(`node_modules/katex/dist/fonts/${f}`, `vendor/katex/fonts/${f}`);
    fonts++;
  }
}
console.log(`vendored ${copies.length} files and ${fonts} KaTeX fonts`);
