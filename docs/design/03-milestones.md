# Checkpoint 3 — Milestones and release (rev 2, after Codex review)

**Assumption for estimates:** ~10 hours/week with an AI coding assistant.
Effort is listed separately from **waits** (hardware access, reviewers, build
farm sync), which don't shrink with more effort. Re-estimate after M1a.

## Process rules

- **Clean-room:** implementation sessions get `docs/design/` and this repo
  only, never the author's earlier projects.
- **Ponytail** in every implementation session (`full`; plan reviews `ultra`).
  PR descriptions list what was reused and what was written.
- **Review gate:** Codex review per milestone PR; blockers fixed, should-fixes
  fixed or ticketed.
- **Test command** everywhere (CI and local):
  `colcon test --return-code-on-test-failure && colcon test-result --verbose`.
- **Early integration is allowed:** milestones are ordered, but small probes
  against real middleware or hardware may run ahead (listed per milestone).
- **Learning notes:** each milestone names the ROS 2 topics to read first.

## M0 — Identity, skeleton, release paperwork (effort 2 wk; waits: rosdistro review)

- Final repo/package names chosen (collision-checked), lowercase. Packages:
  `handeye_autopilot`, `handeye_autopilot_interfaces`, `handeye_autopilot_demos`.
- Metadata: real maintainer name + email, Apache-2.0 in every `package.xml`,
  matching versions in `package.xml` and `setup.py` (`0.1.0`), one
  `CHANGELOG.rst` per package, `docs/design/` with the three checkpoints,
  `AGENTS.md`/`CLAUDE.md` (clean-room + ponytail + reuse-map rule).
- Interface package with generator/runtime deps and `member_of_group`; the
  action and message defined.
- CI: `ros-tooling/action-ros-ci` on Jazzy (24.04), Lyrical (26.04), Rolling;
  `ruff`, `mypy` (core), the import-purity test.
- **Dependency gate:** in a clean container per OS, `rosdep install
  --from-paths src -iry` resolves every runtime, test, launch and demo
  dependency through **apt keys** (no pip). `pymoveit2` binary installs and
  its used API (`MoveIt2`, cancel) is smoke-tested on each distro.
- **Release paperwork starts now:** PR adding the repo to rosdistro as a
  *source* entry (gets name review early); request a release repository
  (Rolling requires it under `ros2-gbp`); set up bloom config. These are
  external approvals and must not sit on the M6 critical path.
- Reserve access to a real arm + camera for M5 (date booked).
- *Learn:* workspaces, colcon, `package.xml`/rosdep, ament_python vs
  ament_cmake, interface packages, REP 149.
- **Done when:** the test command passes on all three distros in CI; the
  dependency gate passes in clean containers; a downstream test package
  imports and constructs `Calibrate` goal/result/feedback and
  `QualityReport`; rosdistro source PR is open.

## M1a — Detection and synthetic data (effort 2 wk)

- Board definitions (ChArUco, parity-asymmetric checkerboard; symmetric
  rejected), board printout script, OpenCV 4.6/4.10 shim, renderer with
  distortion and true-vs-reported poses, per-capture gates, consensus,
  stray-marker masking.
- **Probe ahead:** record ~20 real board images with any camera (no robot
  needed) into `test/fixtures/` for detection tests.
- **Done when (committed in a test spec file):** fixed seeds and pose ranges;
  detection fixture tests give identical IDs/coordinates on 4.6 and 4.10; each
  gate has a test that triggers its specific rejection code (including a
  synthetic stray marker and an ambiguous near-face-on view); real-image
  fixtures detect.

## M1b — Solver and verification (effort 2–3 wk)

- Mode adapter, initial guess, refinement, outlier rejection, observability,
  k-fold verification and precision measure, gates, observations YAML,
  offline re-solve CLI.
- **Done when:** for the committed sweep, both modes recover ground truth
  within committed translation/rotation tolerances; expected-failure cases
  (parallel axes, ambiguity, wrong square size, systematic robot error) fail
  with their committed codes; observations save → reload → re-solve gives an
  identical result; transform-direction tests pass.
- First public post ("offline solver works") is possible here.

## M2 — Planning core (effort 2 wk)

- Bootstrap (incremental excitation, halving), candidate sampling, visibility
  filter, box/envelope/orientation filters, information-based selection,
  joint-space ordering, state machine, with injectable IK and a perfect-motion
  simulated robot.
- **Probe ahead:** a throwaway script drives the Panda mock-hardware demo via
  pymoveit2: plan, execute, cancel mid-motion, and calls `compute_ik` with
  `is_diff=true` on all three distros. Findings feed M4.
- **Done when:** full pure-Python sessions run against the renderer + fake IK
  in both modes within a test deadline and pass gates; probe halving, spares
  and "plan too weak" paths each have a test asserting their outcome.

## M3 — ROS node, manual mode (effort 3 wk)

- Node: parameters/profiles, camera input, capture timing, TF, action server
  with cancel, `~/capture`, markers, annotated image, result YAML, result
  launch (subtree rule + ownership checks), `ros2 bag record` launch option,
  synthetic camera node.
- **Interfaces frozen at the end of M3**, after a node has exercised them
  (version bump if changed).
- *Learn:* nodes, parameters, executors/callback groups, actions, TF2, QoS,
  launch, `launch_testing`.
- **Done when:** `launch_testing` completes a manual-mode calibration with the
  synthetic camera and scripted captures; tests for delayed TF, frozen joint
  states, clock jump, TF-ownership conflict and cancel during capture each
  assert their outcome.

## M4 — MoveIt mode + Panda and UR demos (effort 3–4 wk)

- `compute_ik` client (IK contract), pymoveit2 motion with result checks,
  box as path constraint, cancel during motion.
- `handeye_autopilot_demos`: Panda and UR (mock hardware), both modes, synthetic camera, a
  table collision object and a point-cloud obstacle.
- **Failure tests required before hardware:** planning-only `move_group`
  reporting success without motion (caught by the TF check); collision
  rejection by `compute_ik`; cancel where the MoveGroup goal never reaches a
  terminal state (→ `ABORTED`, `MOTION_FAILED`); attached collision object
  still present after IK calls; real `compute_ik` contract test (timeout
  behaviour) on all three distros.
- *Learn:* MoveIt 2 concepts (groups, SRDF, planning scene, move_group),
  pymoveit2.
- **Done when:** both demos run unattended in both modes and pass; the failure
  tests pass; one demo runs in CI on every distro where MoveIt + the robot
  config install.

## M5 — Real hardware, fixtures, exposure (effort 2 wk; waits: hardware slot)

- Real-image fixtures from the arm runs added to tests (including a stray
  marker in view); σ and detection thresholds re-tuned.
- Exposure: driver ROI auto-exposure (checkpoint 2 §L), tested on a fake
  parameter server and the real camera.
- **Evidence required for release** (committed to `docs/results/`): one
  arm + camera, one mode at least, **5 repeated runs** with the same config:
  per-run gate report, held-out reprojection, spread of the camera transform
  across runs (mm, deg), plus a physical check (e.g. touching a known point
  with the TCP, or a measured distance) as accuracy evidence. Numbers are
  published whatever they are; release proceeds only if gates pass in ≥4 of 5
  runs.
- **Done when:** the evidence file exists and meets that rule.

## M6 — Docs and 1.0 release (effort 2 wk; waits: farm builds + sync, 2–4 wk)

- Docs: 5-minute demo quick start, concepts (modes, frames, IK contract),
  operator runbook, config reference, troubleshooting, "plug in a non-MoveIt
  robot", landscape comparison.
- `bloom-release` per distro track (Jazzy, Lyrical, Rolling); release PRs.
- **Done when:** farm builds succeed for all three packages on all three
  distros; after the public sync, on each distro in a clean container
  `sudo apt install ros-<distro>-handeye-autopilot ros-<distro>-handeye-autopilot-interfaces
  ros-<distro>-handeye-autopilot-demos` works, and a smoke test imports the interfaces and
  launches a demo. (Availability in `ros-testing` first, public after sync.)
- Announcement on ROS Discourse with the demo video and the M5 evidence.

## After 1.0 (ordered)

1. rqt plugin UI.
2. RViz-editable safety box (interactive marker).
3. Live verify mode and CAD reference check.
4. Export to `industrial_calibration` observation format.
5. ArUco/AprilTag grid boards.
6. Gazebo realistic camera demo.
7. TCP and fixture calibration.
8. PyPI release of the core.

## Totals

Effort ≈ 20–22 weeks at 10 h/week, plus waits (rosdistro review overlaps
M0–M5; build-farm sync 2–4 weeks after M6). Treat this as optimistic until
recalibrated after M1a. If work stops after M1b, the offline solver is still a
coherent, demonstrable result.
