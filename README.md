# roboticsbench

Robotics tools that run in the browser. No build step, no accounts, nothing uploaded.

- **Learn**: an eight-step class, each step with something to move and its math updating live. Frames and transforms, forward and inverse kinematics on a 2-link arm (with both elbow solutions), rotations and SO(3), forward kinematics of a 6-axis arm, numerical IK with the Jacobian and singularities, then iLQR planning a swing-up for an arm too weak to lift itself (and MPC re-planning after a kick), and SLAM mapping a loop until it closes.
- **Rotation converter**: quaternion (either order), Euler angles (all 12 orders, intrinsic or extrinsic, deg or rad), axis-angle, rotation vector and matrix, kept in sync, with a 3D preview, paste detection (tf2_echo, ROS messages, NumPy, Eigen) and copyable code for SciPy, Eigen and ROS.
- **URDF viewer**: samples built in, or open a URDF with its meshes (STL, DAE, OBJ) by file, folder or drag and drop. Joint sliders, forward kinematics between any two links, and the geometric Jacobian with manipulability.
- **Kalman filter**: a noisy 1D tracking problem where the filter's assumed noise can be mis-tuned against the truth, the cart can be steered by hand to see a maneuver the filter was never told about, and outliers can be thrown in and gated out. A panel shows where the gain comes from: the prediction's bell curve times the reading's.
- **Localization**: dead reckoning, a particle filter, an EKF, a UKF and a grid filter localizing against landmarks, side by side, plus EKF-SLAM, which is never given the map and builds it as it drives, including a kidnapped robot and a wind that pushes the robot in a way none of them is told about.
- **Motion planning**: BFS (breadth-first search), Dijkstra, greedy, A*, weighted A*, RRT (rapidly-exploring random tree), RRT* and PRM (probabilistic roadmap) on a grid you draw, upload as a text file or download. The search is a timeline you can scrub forward and back one expansion at a time. Below it, a 2-link arm plans in configuration space: drag obstacles in the room and watch the blocked joint angles change, with A* on the wrapping θ1 × θ2 grid and the arm playing the path.
- **Control**: PID, LQR (linear-quadratic regulator), MPC and iLQR on a drone holding altitude, a robot arm against gravity, and an inverted pendulum, simulated live with charts of response, error and output. PID shows its control law and closed-loop poles; LQR shows its one fixed gain and the Riccati equation it comes from; MPC re-plans two seconds ahead twenty times a second and draws its forecast; iLQR optimizes one whole plan and follows it. On MPC and iLQR the pendulum starts hanging down and has to be swung up.

The look is Googie (atomic-age signage): a turquoise sky by day, a neon night, Righteous for display type and Atkinson Hyperlegible for everything else.

## Run

```sh
npm run serve        # serves on http://localhost:5180 with caching off
```

Any static file server works. It has to be served over http, not opened as a file, because the page uses ES modules.

## Test

```sh
npm test             # math and simulation for every tool
```

## Dependencies

three.js, urdf-loader, KaTeX (for the math panels), the Atkinson Hyperlegible fonts and Righteous are copied into `vendor/` and `fonts/` so the site makes no third-party requests. To update them, change the versions in `package.json`, then:

```sh
npm install && npm run vendor
```

## License

Two licenses, because the code and the explanations are different kinds of work.

- **Code** (`js/`, `css/`, `scripts/`, `tests/`, and the page itself) is MIT.
  See `LICENSE`.
- **Writing** (the explanatory text and math panels in `index.html`, the
  algorithm blurbs in `js/`, and this README) is Creative Commons Attribution
  4.0. See `LICENSE-CC-BY-4.0`.

Lift the simulations into your own page with nothing but the notice to carry.
If you reuse the explanations, in a course page or a set of slides, credit them:

```
roboticsbench by lmlack, CC BY 4.0, https://roboticsbench.gwark.dev/
```

The gwark mark (`gwark-mark.svg`) is brand identity, not content, and is
reserved rather than licensed.

Vendored libraries and fonts keep their own licenses. See `THIRD-PARTY.md`.
