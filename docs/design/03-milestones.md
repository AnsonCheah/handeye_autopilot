# Checkpoint 3 — Milestones and release (rev 3.1, after pre-implementation Codex review)

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
- Metadata: maintainer `Anson Cheah <ansoncta.sg@gmail.com>` (decided
  2026-11-26), Apache-2.0 in every `package.xml`,
  matching versions in `package.xml` and `setup.py` (`0.1.0`), one
  `CHANGELOG.rst` per package, `docs/design/` with the three checkpoints,
  `AGENTS.md`/`CLAUDE.md` (clean-room + ponytail + reuse-map rule),
  `.gitattributes` (`* text=auto eol=lf`). Release tags are numeric
  `X.Y.Z`, created by bloom/catkin_prepare_release, and never moved or
  deleted once pushed.
- Interface package with generator/runtime deps and `member_of_group`; the
  `Calibrate` action and the `QualityReport` and `GateResult` messages defined.
- CI: `ros-tooling/action-ros-ci` on Jazzy (24.04), Lyrical (26.04), Rolling;
  `ruff`, `mypy` (core), the import-purity test.
- **Dependency gate** (checkpoint 1 §4): in a clean container per OS, every
  key declared so far (the M0 table: skeleton + demo robot configs) resolves
  to an **apt** package with an install candidate, checked per key before
  installing (no pip). The same gate reruns on every PR as later milestones
  add dependencies. `pymoveit2` binary installs and its used API (`MoveIt2`,
  cancel) is smoke-tested (not skipped) on each distro.
- **Release paperwork starts now:** PR adding the repo to rosdistro as a
  *source* entry (gets name review early); request a release repository
  (Rolling requires it under `ros2-gbp`) **and write access to it for the
  maintainer**; set up bloom config. These are external approvals and must not
  sit on the M6 critical path: M5 cannot close until write access is
  confirmed.
- Real arm + camera for M5: **deferred by the maintainer (2026-11-26)** until
  simulation results (M1b–M4) are in; booked then. Moves the hardware wait
  onto M5's critical path, accepted.
- *Learn:* workspaces, colcon, `package.xml`/rosdep, ament_python vs
  ament_cmake, interface packages, REP 149.
- **Done when:** the test command passes on all three distros in CI; the
  dependency gate passes in clean containers; the CI-only downstream package
  (`ci/interface_consumer/`, checkpoint 1 §12) imports, constructs and
  serialization-round-trips (asserting the fields of) `Calibrate` goal/result/feedback,
  `QualityReport` and `GateResult`; rosdistro source PR is open.

## M1a — Detection and synthetic data (effort 2 wk)

- Board definitions (ChArUco, parity-asymmetric checkerboard; symmetric
  rejected), board printout script, OpenCV 4.6/4.10 shim, renderer with
  distortion and true-vs-reported poses, per-capture gates, consensus,
  stray-marker masking.
- **Probe ahead:** record ~20 real board images with any camera (no robot
  needed) into `test/fixtures/` for detection tests. Needs the maintainer
  (a printed board and a calibrated camera); the synthetic fixtures don't
  wait for it.
- Test spec: `handeye_autopilot/test/spec/m1a.yaml`; synthetic fixture images
  are generated from it once and committed, so both OpenCV versions read the
  same pixels.
- **Output:** the capture record (checkpoint 2 §D5), which M1b consumes.
- **Done when (committed in a test spec file):** fixed seeds and pose ranges;
  detection fixture tests on 4.6 and 4.10 give **identical** corner IDs and
  board-frame coordinates and image corners within a committed pixel
  tolerance; each gate has a test that triggers its specific rejection code;
  the three stray-marker fixtures (masking succeeds → accepted; rejected after
  masking; two boards) and the ambiguity pair (meets both conditions →
  rejected; near face-on but unambiguous → accepted) have the outcomes in
  checkpoint 2 §D; real-image fixtures detect.

## M1b — Solver and verification (effort 2–3 wk)

- Mode adapter (with the OpenCV mapping tests), initial guess, refinement,
  outlier rejection, observability, k-fold verification and fold sensitivity,
  gates, observations YAML, offline re-solve CLI.
- Input: capture records (from the renderer + M1a detection, or loaded
  from an observations YAML). No bootstrap, TF or clocks here.
- **Done when:** for the committed sweep, both modes recover ground truth
  within committed translation/rotation tolerances; the M1b-owned
  expected-failure case (parallel rotation axes) fails with its committed
  code; known-blind-spot cases pass the gates and their measured bias is
  recorded in the test output; observations save → reload → re-solve gives
  the same result within a committed numerical tolerance (`schema_version`
  checked, unknown versions refused); the observability definitions
  (checkpoint 2 §E4) are committed with the sweep; transform-direction tests
  pass.
- First public post ("offline solver works") is possible here.

## M2 — Planning core (effort 2 wk)

- Bootstrap (incremental excitation, halving), candidate sampling, visibility
  filter, box/envelope/orientation filters, information-based selection,
  joint-space ordering, state machine, with injectable IK and a perfect-motion
  simulated robot.
- **Probe ahead:** a throwaway script drives the Panda mock-hardware demo via
  pymoveit2: plan, execute, cancel mid-motion, and calls `compute_ik` with
  `is_diff=true` on all three distros. It records the MoveGroup terminal
  status and error code for success, planning failure and cancel, so the
  M3 interface freeze is based on observed outcomes. Findings feed M4.
- **Done when:** full pure-Python sessions run against the renderer + fake IK
  in both modes within a test deadline and pass gates; probe halving, spares
  and "plan too weak" paths each have a test asserting their outcome; the
  too-small-`L_max` case fails with its committed code.

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
  synthetic camera and scripted captures; tests for delayed TF, bad/mismatched
  timestamps (clock preflight aborts with `TF_UNAVAILABLE`), frozen joint
  states, clock jump, TF-ownership conflict and cancel during capture each
  assert their outcome; admission rejections (busy, invalid mode, unknown
  profile, latched fault) are tested as `accepted = false`.

## M4 — MoveIt mode + Panda and UR demos (effort 3–4 wk)

- `compute_ik` client (IK contract), pymoveit2 motion with result checks,
  box as path constraint, cancel during motion.
- `handeye_autopilot_demos`: Panda and UR (mock hardware), both modes, synthetic camera, a
  table collision object and a point-cloud obstacle. Both demos are in 1.0 by
  the maintainer's decision (review suggested deferring UR; declined).
- **Failure tests required before hardware:** planning-only `move_group`
  reporting success without motion (caught by the TF check); collision
  rejection by `compute_ik`; a pose where **only the attached board/camera**
  collides must be rejected (proves attached bodies take part in the IK
  collision check); cancel where the MoveGroup goal never reaches a terminal
  state (→ `ABORTED`, `MOTION_FAULT`), then a new goal is rejected until
  `~/clear_fault` succeeds; real `compute_ik` contract test (timeout
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
- **Evidence protocol, committed to `docs/results/` before the first run**
  (so limits can't be fitted to the outcome): one arm + camera, one mode at
  least, **5 repeated runs** with the same config, each reporting the gates,
  held-out reprojection and the camera transform; the spread across runs (mm,
  deg).
- **Independent accuracy check sensitive to the camera transform:** a second,
  separate board fixed in the workspace is located by the camera, and its
  corner positions are mapped into the robot base **through the calibrated
  transform**. The robot then touches the same corners with a pointed tool of
  known TCP. The error (mm) between camera-predicted and touched positions is
  the accuracy number; a camera-transform error shows up in it directly. Its
  limit is declared in the protocol before the runs.
- **Release rule:** gates pass in ≥4 of 5 runs **and** the touch-check error
  is within the predeclared limit. All numbers are published whatever they
  are.
- **Done when:** the protocol and the evidence file exist and meet the rule;
  release-repository write access (M0) is confirmed.

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

Milestone effort sums to 18–20 weeks at 10 h/week; plan on ~22 with a
2-week buffer for rework after reviews, plus waits (rosdistro review overlaps
M0–M5; build-farm sync 2–4 weeks after M6). Treat this as optimistic until
recalibrated after M1a. If work stops after M1b, the offline solver is still a
coherent, demonstrable result.
