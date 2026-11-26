# Working rules for agents

`handeye_autopilot` is an open-source ROS 2 package for automated, quality-gated
hand-eye calibration (eye-to-hand and eye-in-hand). Python only (rclpy),
released through the ROS build farm for Jazzy, Lyrical and Rolling.

The plan is in [docs/design/](docs/design/README.md). Read it before changing
anything; it is the specification.

## Rules

- **Clean-room.** Work only from `docs/design/` and this repository. Never copy
  code, constants or wording from the author's earlier projects (tcp_manager,
  sesto_prime), even if they are available. Tuning values come from simulation
  sweeps and real-image fixtures.
- **Reuse before writing (ponytail).** Use what ROS 2, MoveIt, OpenCV, SciPy
  and NumPy already ship. Write only what the reuse map in
  `docs/design/01-architecture.md` §2 says we write. Every new module must point
  to a row there; if it can't, update the plan first. PR descriptions list
  what was reused and what was written.
- **Plan first, then code.** A change that alters an interface, a milestone's
  scope or an algorithm in `docs/design/` updates the design doc in the same
  change.
- **Codex review per milestone or plan checkpoint.** Before merging a
  milestone PR or finishing a plan revision, run a read-only Codex review, e.g.
  `codex exec -s read-only -o review.md "<what to review and focus areas>"`.
  Fix blockers; fix or ticket should-fixes.
- **Explain ROS 2 concepts.** The maintainer is new to ROS 2 and C++. When a
  change introduces a ROS 2 concept (executors, QoS, TF, actions, launch,
  rosdep, bloom…), explain it briefly and link the official docs.

## Conventions

- SI units and REP 103/105 frames on the ROS side; ROS quaternion order
  (x, y, z, w). Humans get mm and degrees in reports.
- `handeye_autopilot/core/` must not import any ROS module (a test enforces
  it).
- Every non-trivial logic change ships with a test. Shortcuts with a known
  limit get a comment: `shortcut: <the limit>, <when to upgrade>`.

## Environment

- Develop in WSL2 Ubuntu 24.04 (ROS 2 Jazzy), in the clone under the Linux
  home (`~/workspaces/handeye_autopilot`), not in a Windows or OneDrive
  checkout. Lyrical and Rolling are covered by CI and Docker.
- Build from the workspace root (`~/workspaces`): `colcon build`.
- Test: `colcon test --return-code-on-test-failure && colcon test-result --verbose`.
