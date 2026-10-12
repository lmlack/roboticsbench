# Third-party components

The MIT grant in `LICENSE` and the CC BY grant in `LICENSE-CC-BY-4.0` cover this
project's own code and writing. The components below are other people's work,
are copied into this repository so the site makes no third-party requests, and
keep their own terms.

| Component | License | Text |
| --- | --- | --- |
| three.js r170 | MIT | `vendor/three/LICENSE` |
| urdf-loader (gkjohnson/urdf-loaders) | Apache-2.0 | `vendor/urdf-loader/LICENSE` |
| KaTeX | MIT | `vendor/katex/LICENSE` |
| Atkinson Hyperlegible Next | SIL Open Font License 1.1 | `fonts/atkinson.LICENSE` |
| Righteous | SIL Open Font License 1.1 | `fonts/righteous.LICENSE` |

KaTeX also ships its own fonts under `vendor/katex/fonts/`, covered by the KaTeX
MIT license above.

Both display faces have reserved font names under the OFL: a modified copy has
to be renamed.

## Not third party

The sample robots in `js/urdf-samples.js` are built from URDF primitives for
this project. They are loosely proportioned after common arms and are not any
manufacturer's model, so they fall under the licenses in the repository root.
