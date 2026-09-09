# Convex-MPC

Quadruped locomotion with the convex model-predictive controller from

> J. Di Carlo, P. M. Wensing, B. Katz, G. Bledt, and S. Kim,
> **"Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control,"**
> 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS),
> pp. 7440-7447. doi: [10.1109/IROS.2018.8594448](https://doi.org/10.1109/IROS.2018.8594448)

The controller is implemented from scratch in C++ (Eigen + qpOASES) and runs an
ANYmal C robot in the [MuJoCo](https://mujoco.org) simulator. The same C++
simulation core builds natively and as a WebAssembly module, rendered in the
browser with Three.js. Everything is open source and dependencies are fetched
automatically at build time.

## Quick start

Requirements: Linux, a C++17 compiler, CMake ≥ 3.16, Eigen ≥ 3.4
(`sudo apt install libeigen3-dev`). qpOASES and the MuJoCo SDK are fetched
automatically by CMake. The supplied Dev Container includes Emscripten and
Node.js 20 for the browser viewer.

```bash
git clone https://github.com/CMaybe/Convex-MPC.git
cd Convex-MPC
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Run:

```bash
# walk forward at 0.4 m/s while turning at 0.3 rad/s
./build/simulation/mpc_locomotion --vx 0.4 --wz 0.3

# headless 10 s stability test (exits non-zero if the robot falls)
./build/simulation/mpc_locomotion --headless --steps 10000

# four-legged standing only (handy for debugging force control)
./build/simulation/mpc_locomotion --stand
```

Flags: `--vx/--vy` body-frame velocity command [m/s], `--wz` yaw rate [rad/s],
`--steps N` simulation steps (1 ms each), `--headless`, `--stand`, `--xml PATH`.
The controller uses a fixed-timing diagonal Trot gait. Commands are limited to
$|v_x| \leq 1.2\,\mathrm{m/s}$, $|v_y| \leq 0.6\,\mathrm{m/s}$, and
$|\omega_z| \leq 1.0\,\mathrm{rad/s}$.

## Web viewer

Build the native project once, then build and run the browser viewer inside the
Dev Container:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
npm --prefix web run build:full-wasm
npm --prefix web run dev
```

Open the forwarded port reported by VS Code, typically `http://localhost:3001`.
The viewer renders the original ANYmal C visual meshes using live MuJoCo poses.
It provides velocity and body-height controls, MPC Q/R scale controls, a reset
command, ground-reaction-force arrows, camera follow, and mouse orbit, zoom,
and pan controls.

## Project layout

```
ConvexMPC/    controller library (simulator-independent, Eigen + qpOASES only)
  robot_model      single-rigid-body dynamics, eq. (16)-(17), ZOH discretization eq. (25)-(26)
  convex_mpc       condensed QP formulation, eq. (27)-(32), solved with qpOASES
  robot_controller reference generation (Sec. IV-B), gait schedule + per-step
                   force constraints eq. (20)-(24), swing-leg control eq. (1) & (33)
  robot_state      robot state container
simulation/   shared SimulationCore (MuJoCo integration) + native headless CLI
wasm/         source-linked MuJoCo + ConvexMPC Emscripten target and bindings
web/          React + Three.js viewer, WebAssembly loader, and build config
```

Implementation notes:

- The MPC state is `[Θ, p, ω, ṗ, g] ∈ R^13`; ground-reaction forces for all
  four feet are the decision variables, re-planned at 40 Hz over a horizon of
  one full gait cycle (16 × 25 ms) with a zero-order-hold discretization.
- Force constraints over the horizon follow the *predicted* contact schedule of
  the fixed-timing trot, so the QP plans through upcoming stance/swing
  transitions rather than assuming the current contacts persist.
- Yaw and xy-position references are obtained by integrating the operator
  velocity commands (with a saturation band around the measured state), as in
  Sec. IV-B of the paper.
- Swing feet follow a time-parameterized world-frame trajectory (smoothstep in
  the ground plane, Bezier height profile with velocity feedforward) from the
  captured lift-off point to the foothold heuristic `p_des = p_ref + v_CoM Δt/2`
  (eq. 33), tracked by Cartesian PD (eq. 1) with gravity/Coriolis compensation.
- Leg gravity/Coriolis compensation is applied to stance legs as well: ANYmal
  C's legs are a much larger mass fraction (~57%) than Cheetah 3's (~10%), and
  without it the realized contact forces do not match what the single-rigid-body
  model assumes.
- The single-rigid-body mass and composite inertia are computed from the
  simulation model at startup, not hardcoded.

The simulation adds a short joint-PD settling phase before the gait starts and
slew-rate-limits the velocity commands, so runs begin from a clean stand.

Tracking performance (60 s headless runs): linear velocity and yaw rate within
a few percent of the command, lateral drift of millimeters over 20+ m of
walking. Known limitations (deliberate, in the spirit of the paper's
simplifications): the gait is fixed-timing with no contact detection, so
landings show brief force transients; and the purely proportional MPC cost
leaves small steady-state offsets — the trotting body height sits a few
centimeters above the reference (touchdown impulses) and the body pitches
nose-down by ~3 degrees at walking speed (unmodeled swing-leg reactions).
`SimulationCore` owns model loading, state extraction, MPC/swing control,
Jacobian torque mapping, and MuJoCo stepping. The native CLI and browser use
the same implementation. Runtime body-height and Q/R scaling controls are
applied through this core.

## Docker / devcontainer

A `Dockerfile` and a VS Code devcontainer are provided. The image contains only
the native and browser toolchains, Eigen, GLFW, Emscripten, and Node.js 20 —
everything else (qpOASES, MuJoCo) is fetched by CMake inside the container. The
repository is mounted at `~/convex-mpc` in the container.

The web workspace follows the React + webpack development shape used by the
companion WASM projects:

```bash
# inside the dev container
em++ --version
npm --prefix web run build:full-wasm
npm --prefix web run dev
```

`build:full-wasm` compiles MuJoCo 3.11.0, qpOASES, and the shared C++
`SimulationCore` into one module and packages the ANYmal resources. The browser
calls `SimulationCore::step()` directly, so its control timing matches the
native headless simulator. Verify the MPC-only C++ to JavaScript boundary
without a browser:

```bash
npm --prefix web run test:wasm
```

If `em++ --version` is not available, run **Dev Containers: Rebuild Container**
in VS Code. Emscripten is installed in the image, while the current workspace
mount preserves the source tree and `web/node_modules`.

### VS Code devcontainer

1. Install the **Dev Containers** extension and open the repository folder —
   VS Code offers *"Reopen in Container"* (or run it from the command palette).
2. Inside the container, build and test exactly as on the host:

   ```bash
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j
   ./build/simulation/mpc_locomotion --headless --steps 10000   # prints STABLE on success
   ```

3. Run `npm --prefix web run build:full-wasm`, then `npm --prefix web run dev`
  for the interactive browser viewer.

> [!NOTE]
> The executable resolves `libmujoco` and the robot model relative to its own
> location, so a build tree still runs when the repository is mounted at a
> different path. If you switch between host and container builds, prefer
> separate build directories (e.g. `build/` and `build-container/`) — a CMake
> cache created at one path cannot be *re-configured* from another.

### Plain Docker (no VS Code)

```bash
docker build -t convex-mpc --build-arg USER_NAME=$USER .
docker run -it --rm --net=host -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev/dri:/dev/dri \
    -v $(pwd):/home/$USER/convex-mpc convex-mpc
```

## Licenses

- Code: MIT (see `LICENSE`).
- ANYmal C model (`simulation/rsc/anymal_c`): BSD-3-Clause, © ANYbotics AG,
  taken from [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)
  with small modifications (torque actuators, foot sites, standing keyframe).
- MuJoCo (downloaded at build time): Apache-2.0. qpOASES: LGPL-2.1.
