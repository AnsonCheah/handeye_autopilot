# Checkpoint 1 — Architecture and interfaces (rev 3.1, after pre-implementation Codex review)

Package: `handeye_autopilot`.

## 0. Ground rules

- **Clean-room rewrite.** No code, constants, layout or wording copied from the
  author's earlier projects. This plan describes behaviour only; implementation
  runs in a fresh session given this plan and nothing else. Tuning constants
  are re-derived from simulation sweeps.
- **Ponytail rule:** use what ROS 2 / OpenCV / SciPy already ship; write only
  what does not exist. Every new module points to a row in the reuse map (§2).
- **License:** Apache-2.0. **Distros:** Jazzy (24.04), Lyrical (26.04),
  Rolling. Kilted skipped (EOL around first release). Verified in rosdistro
  (2026-11-25): `moveit`, `moveit_resources`, `ur_robot_driver`, `pymoveit2`
  are released on all three.
- **Python only** (rclpy). The only CMake package is the interface package.
- **Conventions:** SI units, REP 103/105, ROS quaternion order (x,y,z,w).

## 1. Landscape and where this package fits

| Project | What it does well | Why it doesn't cover this need |
|---|---|---|
| `easy_handeye2` | Simple GUI, OpenCV solvers, publishes result | Samples collected by hand; no pose planning, motion or quality gates; not on the build farm |
| `industrial_calibration` + `industrial_calibration_ros2` (ROS-I) | Ceres solver, covariance, noise and homography analysis, several target types, RViz panel | Data collection is manual (save on trigger); no pose generation or motion; not on the build farm; C++ with no Python API, so we can't depend on it from a farm-released Python package |
| `moveit_calibration` (ros2 branch) | RViz plugin, automatic sampling around a pose | Unreleased for ROS 2, 59 open issues |
| `robot_calibration` | Released; full kinematic calibration | Different scope (robot kinematics), heavier setup |
| `pymoveit2` | Released rclpy wrapper for MoveGroup, IK, planning scene | A building block, not a calibration tool. **We reuse it.** |

**This package's contribution:** automated, safe, quality-gated *data
collection and validation* for hand-eye calibration: pose planning from the
camera's point of view, reachability/collision through MoveIt, robust
per-capture rejection, sanity and fold-sensitivity gates, held-out
verification, and simulation tests.
Solving uses OpenCV + SciPy. Interop (v1.1): export observations in
`industrial_calibration`'s format so users can cross-check with its solver.

## 2. Reuse map

| Need | Use (exists) | We write |
|---|---|---|
| Image conversion | `cv_bridge` | Nothing |
| Camera model, raw vs rectified | `image_geometry` `PinholeCameraModel` (ROS layer); in `core`, K/D as NumPy with OpenCV `projectPoints`/`undistortPoints` (no ROS) | Validation of `CameraInfo` (§7); a small `CameraModel` holder in `core` |
| Board detection | OpenCV `findChessboardCornersSB`, ArUco/ChArUco detectors, `cornerSubPix` | Compatibility shim for OpenCV 4.6 (24.04) and 4.10 (26.04); canonical checkerboard corner order; a final `cornerSubPix` on ChArUco corners (OpenCV's interpolation returns them ~0.5 px off on both versions, measured in M1a) |
| Board pose per frame, both planar solutions | OpenCV `solvePnPGeneric` with `SOLVEPNP_IPPE` | Nothing |
| Board printouts | OpenCV `CharucoBoard.generateImage` (ChArUco) | A short NumPy function for checkerboards; `cli/board_image.py` (exact scale, PNG `pHYs`) |
| Intrinsics | `camera_calibration` (image_pipeline) | Nothing |
| Initial hand-eye solve | OpenCV `calibrateHandEye`, `calibrateRobotWorldHandEye` | Nothing |
| Refinement and uncertainty | SciPy `least_squares` (robust loss, Jacobian) | Residual function |
| Rotations | SciPy `Rotation`, OpenCV `Rodrigues` | `core/transforms.py`: 4×4 helpers (make, inverse, angle) over them |
| Quasi-random sampling | SciPy `qmc.Halton` | Nothing |
| Image + camera info | Image subscription; latest `CameraInfo` cached (§7) | Cache + consistency check |
| Robot pose at a stamp | `tf2_ros` Buffer, async wait with deadline | Nothing |
| Motion (plan + execute, cancel) | `pymoveit2` `MoveIt2` | Goal parameters, result checks |
| Reachability + collision | `moveit_msgs/srv/GetPositionIK` on `compute_ik` | A direct client (~20 lines; pymoveit2's IK call sets `is_diff=false`, see §3.1) |
| Collision scene (meshes, point clouds) | MoveIt planning scene, octomap | Docs and demo config only |
| Exposure | Driver's own ROI auto-exposure, set via rclpy `AsyncParameterClient` | ROI update + driver parameter-name map (checkpoint 2 §L) |
| Stray-marker rejection | OpenCV `solvePnPRansac` + ChArUco interpolation | Glue (checkpoint 2 §D4) |
| Publish result | `tf2_ros static_transform_publisher` from a launch file | Launch file |
| Visualisation | RViz2, `MarkerArray`, OpenCV drawing | Marker builders |
| Run recording | `ros2 bag record` (launch option) | Observations YAML for offline re-solve (`core/observations.py`); result YAML (`core/results.py`); `resolve` CLI (`cli/resolve.py`) |
| Start / cancel / progress | ROS 2 action | One action definition |
| Manual capture trigger | `std_srvs/srv/Trigger` | Nothing |
| Robots for demos and CI | UR (`ur_robot_driver` mock hardware), Panda (`moveit_resources_panda`) | Demo launch files |
| Camera simulation | Gazebo + `ros_gz` (optional, realistic) | Lightweight homography renderer for deterministic CI (§10) |

Modules we write that have no ecosystem equivalent (the actual contribution);
each still builds on the reused pieces named:

| Module | Builds on | Spec |
|---|---|---|
| Mode adapter (both modes, OpenCV argument mapping) | NumPy, SciPy `Rotation`, OpenCV hand-eye solvers | checkpoint 2 §A |
| Session state machine + bootstrap | the modules below | checkpoint 2 §B |
| Visibility filter + information-based pose selection + ordering | OpenCV `projectPoints`, SciPy `qmc`, `compute_ik` joint solutions | checkpoint 2 §C |
| Capture gates + consensus + stray-marker masking | OpenCV detection, PnP, RANSAC | checkpoint 2 §D |
| Refinement residual, outlier rejection, observability | SciPy `least_squares`, `median_abs_deviation` | checkpoint 2 §E |
| Fold refits (held-out reprojection, fold sensitivity) + quality gates | the solver above | checkpoint 2 §F–G |

## 3. Robot side: MoveIt's interfaces are the hook

The node never talks to a robot driver. It uses:

| Need | Interface | Default name (relative, remappable) |
|---|---|---|
| Pose reachable and collision-free | `moveit_msgs/srv/GetPositionIK` | `compute_ik` |
| Plan + execute to a pose | `moveit_msgs/action/MoveGroup` via `pymoveit2` | `move_action` |
| Robot pose for the solve | TF `base_frame → tool_frame` | — |

A robot without MoveIt plugs in by serving `compute_ik` with the
`GetPositionIK` type from its own IK, **honouring the contract in §3.1**, and
uses manual mode for motion. The repo supports no brand explicitly and every
MoveIt brand implicitly.

### 3.1 IK contract (documented, enforced where checkable)

- `group_name` must be an SRDF planning group; `ik_link_name` must be a link in
  the robot model, rigidly attached to the group's tip.
- If the user's `tool_frame` exists only in TF (a TCP not in the URDF), the
  node looks up the fixed `ik_link → tool_frame` transform once at goal start
  (must be static; checked twice, 1 s apart) and converts every candidate pose
  to an `ik_link` pose. Refuse to start if the transform is missing or moving.
- Requests use the **monitored planning-scene state** (`robot_state` empty,
  `is_diff = true`), so attached collision objects are kept. The seed is the
  scene's current state. (A joint-only state with `is_diff = false` can drop
  attached bodies; this is why we don't use pymoveit2's IK call.)
- `avoid_collisions = true`. Third-party servers must honour it; the docs say
  so and the launch test includes a stub server that does.
- **One candidate per request.** `pose_stamped_vector` means several end
  effectors at once, not a batch. Requests are bounded: at most K outstanding
  *(tune)*, per-request timeout, no retries beyond one.
- **Timeout gotcha:** the Jazzy `compute_ik` capability passes only
  `timeout.sec` and drops nanoseconds. Use whole seconds, or leave it zero and
  set the server-side default; verify on each distro in CI. A client deadline
  does not cancel the server's computation; the bound on outstanding calls is
  what protects us.
- `NO_IK_SOLUTION` does not distinguish "unreachable" from "in collision".
  Rejection reason is recorded as `IK_FAILED` with the MoveIt error code; no
  finer claim is made.

### 3.2 Motion (pymoveit2)

- Goal per planned pose: a **joint goal** equal to the IK solution kept for
  that candidate (checkpoint 2 §C5), so execution reaches the same arm
  configuration the ordering was computed for (a pose goal could pick another
  IK branch and a long wrist flip). Finite planning time, and **both** velocity
  and acceleration scaling set in (0, 1] from parameters (a zero scaling means
  full speed in MoveIt; parameter validation rejects 0).
- If planning to the joint goal fails from the actual current state, the
  candidate is skipped and a spare is used.
- Success requires: goal accepted, terminal status `SUCCEEDED`, MoveIt error
  code `SUCCESS`, **and** the measured TF pose within tolerance of the target
  after settling. A planning-only `move_group` can report success without
  moving; the TF check catches that.
- The safety box, if enabled, is also sent as a `path_constraints` position
  constraint so intermediate motion stays inside it (option; can make planning
  slower, default on when the box is on).

### 3.3 Validation chain per candidate

1. Our robot-agnostic filters (numpy): safety box (toggleable), seed-relative
   envelope, orientation limit from the seed, camera visibility. Visibility
   needs a current estimate of both unknowns; before the preliminary solve only
   the probe moves (§checkpoint 2 B) run, so no visibility prediction is needed
   there.
2. `compute_ik` (§3.1). If the service is absent, only manual mode is allowed.
3. Optional plan-only check (pymoveit2 `plan`), off by default.

Every rejection is recorded with its reason.

### 3.4 Safety box

`bounds.enabled: true|false`. Pose + size in a named frame. Constrains the
`tool_frame` origin, optionally all board corners. Defined in parameters,
shown in RViz as a marker (`shortcut: not editable in RViz, add an interactive
marker if users ask`). Disabled = limits come from `compute_ik` and visibility
only.

The collision warning checks both representations of the carried board or
camera: URDF collision geometry on links fixed to the tool, and attached
collision objects in the planning scene. Warn only if neither exists.

## 4. Packages and installation

| Package | Build | Contents |
|---|---|---|
| `handeye_autopilot` | `ament_python` | `handeye_autopilot/core/` (no ROS imports anywhere under it, and `handeye_autopilot/__init__.py` imports nothing ROS; a test enforces both), `handeye_autopilot/ros/` (node, synthetic camera node), launch, config, RViz files installed via `data_files`; packages found with `find_packages`. |
| `handeye_autopilot_interfaces` | `ament_cmake` | One action, two messages (`QualityReport`, `GateResult`). Declares `rosidl_default_generators` (build), `rosidl_default_runtime` (exec), `member_of_group rosidl_interface_packages`. |
| `handeye_autopilot_demos` | `ament_python` | UR and Panda demos. Only this package depends on robot configs. |

Every dependency is a rosdep key that must resolve to an **apt** package on
all three distros. The `package.xml` files are the source of truth; this table
is the M0 baseline and grows as milestones add dependencies (each PR that adds
one updates it):

| Package | build / buildtool | exec | test |
|---|---|---|---|
| `handeye_autopilot_interfaces` | `ament_cmake`, `rosidl_default_generators`, `geometry_msgs`, `action_msgs`, `service_msgs` | `rosidl_default_runtime`, `geometry_msgs`, `action_msgs`, `service_msgs` | — |
| `handeye_autopilot` | — | `python3-numpy`, `python3-scipy`, `python3-opencv`, `python3-yaml`, `rclpy`, `cv_bridge`, `image_geometry`, `tf2_ros`, `geometry_msgs`, `sensor_msgs`, `std_srvs`, `moveit_msgs`, `pymoveit2`, `handeye_autopilot_interfaces` | `python3-pytest` |
| `handeye_autopilot_demos` | — | `handeye_autopilot`, `handeye_autopilot_interfaces`, `launch`, `launch_ros`, `moveit_resources_panda_moveit_config`, `ur_moveit_config`, `ur_robot_driver` | `python3-pytest` |

**Dependency gate (CI, every PR, every distro;
`ci/scripts/check_deps.sh`):** keys are listed with `rosdep keys --from-paths
.` **without `-i`** (with `-i` rosdep also drops every key already installed in
the sourced ROS underlay), minus the names of this repo's own packages; for
each key, `rosdep resolve` must name the `apt` installer
and `apt-cache policy` must show an install candidate, checked **before**
anything is installed (so already-installed keys can't hide a pip-only or
missing key); then `rosdep install` runs. Known gaps go in
`ci/known_missing.txt` (distro, key, date, reason): the gate warns instead of
failing, and fails once the package appears so the entry gets removed. First
entry: `ur_moveit_config` is not built for Rolling (needed from M4). CI builds and tests a normal
**installed** (not symlink) build on each distro's own OS image with distro
OpenCV.

## 5. The custom interfaces

### `Calibrate.action`

The `.msg`/`.action` files in `handeye_autopilot_interfaces` are the canonical
schema (names, types, numeric constants); this section is the summary and the
semantics. Shape:
```
# Goal
string mode                # "eye_to_hand" | "eye_in_hand"
string execution           # "moveit" | "manual"
string profile             # parameter profile name; empty = node parameters
---
# Result
uint8  error_code          # NONE, INVALID_GOAL, NO_CAMERA_INFO, TF_UNAVAILABLE,
                           # IK_SERVICE_MISSING, MOTION_FAILED, MOTION_FAULT,
                           # NOT_ENOUGH_SAMPLES, SOLVE_FAILED, CANCELED, INTERNAL
bool   transform_valid     # an estimate exists (true even when quality failed)
bool   passed              # all quality gates passed
geometry_msgs/TransformStamped camera_transform   # parent -> camera optical frame
geometry_msgs/TransformStamped target_transform   # board in tool (e2h) / base (eih)
handeye_autopilot_interfaces/QualityReport quality
string result_file
string message
---
# Feedback
string phase               # one of the PHASE_* constants
uint32 accepted
uint32 rejected
string last_rejection      # one of the REJECT_* constants, or empty
string last_rejection_detail   # human-readable, free text
```

**Semantics:**
- **Admission.** A ROS 2 action server can only accept or reject a goal; a
  rejected goal gets **no result** (the client sees `accepted = false`, see
  [ROS 2 actions design](https://design.ros2.org/articles/actions.html)).
  Rejected at admission, with the reason logged: a run is already active;
  `mode` or `execution` not one of the constants; `profile` names no known
  profile; the **motion fault latch** is set (§6). There is therefore no
  `BUSY` result code.
- Everything else is checked **after** acceptance and ends as `ABORTED` with a
  code: profile parameters failing validation (`INVALID_GOAL`), camera-info and
  clock preflight (`NO_CAMERA_INFO`, `TF_UNAVAILABLE`, §7–8), IK service
  missing in MoveIt mode (`IK_SERVICE_MISSING`).
- Configuration is snapshotted at goal acceptance; parameter changes mid-run
  don't apply.
- `SUCCEEDED`: the run completed. `passed` may be false (quality gates failed)
  with `transform_valid` true.
- **Transforms in the result.** `camera_transform`: `header.frame_id` = the
  parent (`base_frame` for eye-to-hand, `tool_frame` for eye-in-hand),
  `child_frame_id` = the camera optical frame from `CameraInfo`, i.e. the pose
  of the optical frame in the parent (`X`, checkpoint 2 §A).
  `target_transform`: `header.frame_id` = `tool_frame` (eye-to-hand) or
  `base_frame` (eye-in-hand), `child_frame_id` = the configured board frame
  name (`Z`). `header.stamp` of both = node time when the final solve finished.
  When `transform_valid` is false both are identity with empty frame ids.
- `ABORTED`: a runtime failure; `error_code` says which.
- `CANCELED`: cancel requested and completed (see §6).
- `~/capture` (`std_srvs/Trigger`, manual mode): success means "capture
  started"; the accept/reject outcome arrives in feedback. Returns
  `success=false` outside the manual capture phase or while a capture runs.

**Unset vs zero.** Every float metric that was not computed (run aborted
before that stage, or no data) is **NaN**; a measured zero is 0.0. Counts are
always set. Gates that were not evaluated are absent from `gates`.
**Failed folds.** Held-out aggregates use only folds whose training set solved
and passed observability; `folds_failed` counts the others (any failed fold
fails the run, checkpoint 2 §F).

`QualityReport.msg`: a `GateResult[]` list plus summary fields: capture
counts (accepted, rejected, inliers, outliers); reprojection RMS/max (px);
observability (largest relative rotation, rotation-axis spread, scaled smallest
singular value); folds and failed folds with worst-fold translation (m) and
rotation (rad); held-out reprojection RMS/max (px). SI on the ROS side; the
result YAML and docs show mm and degrees.

`GateResult.msg` (added in M0: a list of gates needs an element type; one
message with parallel arrays was rejected as error-prone): `name`, `group`
(`sanity` / `fold_sensitivity` / `accuracy_evidence`), `unit`, `comparison`
(`<=` or `>=`; empty for a measurement-only row, whose `limit` is NaN and
whose `passed` is false and unused), `value`, `limit`, `gated` (false for
accuracy evidence), `passed`. See checkpoint 2 §F for what each group does and does not prove.

Error codes and the `mode`/`execution` strings are constants in the action
definition. Fields may still change until the interface freeze at the end of
M3.

## 6. Concurrency and cancellation

- `MultiThreadedExecutor`. Callback groups: a reentrant group for TF, image
  and camera-info subscriptions; a separate group for the action server and
  capture service; the run itself executes as the action's async execute
  callback and only awaits futures, never blocks on spin.
- The action server installs a cancel callback that **accepts** cancellation
  (rclpy's default rejects it).
- Cancel during motion: call pymoveit2's cancel on the active MoveGroup goal,
  then wait (with deadline) for its terminal status; only then report
  `CANCELED`. A cancel acknowledgement is not proof the robot stopped; if the
  terminal status doesn't arrive by the deadline, report `ABORTED` with
  `MOTION_FAULT` and a message telling the operator to check the robot.
- **Motion fault latch.** After `MOTION_FAULT` the node refuses every new goal
  until the operator calls `~/clear_fault` (`std_srvs/Trigger`). Clearing
  succeeds only when no MoveGroup goal from this node is still active **and**
  fresh `joint_states` (newer than the call) show all joint velocities below a
  threshold over a window *(tune)*; otherwise it returns `success=false` with
  the reason. The latch survives goal completion and is logged loudly.
- Cancel during capture or IK: stop issuing new requests, drop outstanding
  results, report `CANCELED`.
- Launch tests cover cancel during motion, during capture, and during IK, and
  a new goal sent after a cancel timeout (must be rejected until the fault is
  cleared).

## 7. Camera input

- `cv_bridge` for images, `image_geometry.PinholeCameraModel` for geometry.
  Parameter `image_is_rectified`:
  - raw: use `K` and `D`; supported distortion models `plumb_bob` and
    `rational_polynomial`; others rejected with a clear message
    (`shortcut: no fisheye in v1, add via cv2.fisheye if requested`);
  - rectified: use `P[:3,:3]` as the camera matrix with zero distortion. The
    image is then in the rectified optical frame. v1 requires `R` = identity
    (monocular rectification); otherwise refuse with a message
    (`shortcut: stereo-rectified R unsupported, rotate the result by R when
    someone needs it`).
- Reject uncalibrated `CameraInfo` (`K` all zero). Reject non-default binning or
  ROI in v1 (`shortcut: binning/ROI unsupported, scale K when someone needs it`).
- Images use sensor-data QoS. `CameraInfo` is **not** time-synchronised: the
  node keeps the latest message (its subscription uses reliable,
  transient-local QoS when the driver offers it, so a latched message arrives
  late-joining) and the image callback uses it if `frame_id` and image size
  match. A changed `CameraInfo` mid-run aborts the run (`NO_CAMERA_INFO` with a
  message), since the intrinsics are fixed per run.

## 8. Capture timing

- **One clock domain.** Image stamps and TF stamps must come from the same
  clock (`use_sim_time` consistent across node, robot and camera; a camera
  driver stamping with its device clock must be configured to use the host
  clock). Checked in the preflight right after goal acceptance and per
  capture: if image stamps and the node clock differ by more than a threshold
  *(tune)*, the run is **aborted** (`ABORTED`, `TF_UNAVAILABLE`, message naming
  the clock), not warned about. (It can't be an admission rejection: that
  would need an image first and rejections carry no result, §5.)
- **Settle:** after motion, wait until the TF pose of the tool is stable over
  a window *(tune)* **and** the robot's measurement is fresh: the latest TF for
  the tool must be newer than the settle start (catches frozen
  `joint_states`).
- Only frames whose **acquisition stamp** is after settle completed are used.
- **Every consensus frame is paired with TF at its own stamp**, waited for
  asynchronously with a deadline. Never substitute the latest TF. If TF at a
  stamp would need extrapolation or doesn't arrive, that frame is dropped; too
  many drops reject the capture.
- Launch tests: delayed TF, frozen joint states, clock jump (sim time reset)
  mid-run → the run aborts cleanly, not silently wrong.

## 9. Results and TF publishing

- Result YAML: both transforms, quality report, config snapshot, timestamps.
- **Camera subtree rule.** The solve yields `T_parent_optical`. Drivers publish
  their own rigid subtree `root → … → optical`. The node looks up
  `T_root_optical` and stores
  `T_parent_root = T_parent_optical × inverse(T_root_optical)`.
  The result launch publishes `parent → root` with
  `static_transform_publisher`.
- **Ownership checks before publishing:**
  - every link of the chain `root → optical` must be published on
    `/tf_static` (checked by listening to `/tf_static` briefly), not on `/tf`;
  - `root` must have **no existing parent** in TF (from a URDF mounting joint,
    a driver, or a previous calibration). If it has one, refuse and tell the
    user which frame to drop or which `root` to use instead;
  - **cycle check:** `root` must not be an ancestor of `parent` (walk
    `parent`'s ancestors in the TF tree); otherwise refuse.
- Parent = `base_frame` (eye-to-hand) or `tool_frame` (eye-in-hand).
- Depth and other driver frames follow automatically, as long as the driver's
  internal extrinsics are right (documented as an assumption).

## 10. Simulation

- **Homography renderer** (ours, deterministic, fast): board image warped into
  the camera at the ground-truth pose with noise, blur, brightness variation.
  Used by `handeye_autopilot.core` tests and by a synthetic camera node for launch tests and
  demos. Limitations documented: no lens-shading realism, no 3D occlusion, no
  realistic motion blur.
- **Gazebo (`ros_gz`)**: optional realistic demo later (v1.x), not a CI
  dependency.

## 11. UI

1. **v1: RViz2 + command line.** RViz config: safety box (display only, §3.4),
   envelope, candidates coloured by status, camera frustum, detected board,
   annotated image. Copy-paste commands in the docs.
2. **v1.x: rqt plugin in Python** (`python_qt_binding`).
3. Not planned: custom RViz C++ panel, web UI.

## 12. Testing

- `handeye_autopilot.core`: pytest, no ROS. End-to-end run on the homography renderer must
  recover ground truth for both modes; explicit **transform-direction tests**
  per mode (a swapped inverse must fail a test); import-purity test.
- **OpenCV fixtures:** the same stored board images run through detection on
  OpenCV 4.6 and 4.10 (Jazzy and Lyrical CI) and must give identical corner
  IDs and board-frame coordinates, and image corners within a committed pixel
  tolerance (catches ChArUco legacy-pattern changes).
- Node: `launch_testing` with the synthetic camera, a stub `compute_ik`
  server, a fake MoveGroup action server; cancel, timing and TF-ownership
  tests from §6, §8, §9.
- MoveIt integration: Panda mock-hardware demo end to end in CI (Jazzy at
  least).
- CI: GitHub Actions, matrix {Jazzy on `ubuntu:noble`, Lyrical on
  `ubuntu:resolute`, Rolling on whichever Ubuntu its binaries are published
  for (`ubuntu:noble` as of 2026-11-26)} containers,
  `ros-tooling/setup-ros@v0.7` + `ros-tooling/action-ros-ci@v0.4` with
  `no-symlink-install`; the checkout used by the gate and the consumer and
  action-ros-ci's own checkout are pinned to **one** commit (the PR head SHA,
  passed as `ref` to both); dependency gate (§4); `ruff`; `mypy --strict` on
  `handeye_autopilot.core`. The `pymoveit2` API smoke test runs (not skips) on
  every leg (`HANDEYE_AUTOPILOT_REQUIRE_DEPS=1`).
- **Core on several OpenCV versions:** a pip-based job runs the ROS-free
  tests on OpenCV 4.10 and 4.12 (4.12 ships in no distro yet), so detection
  changes are checked against every API variant without waiting for ROS
  images. Test-only; the released packages still use apt OpenCV.
- **Downstream consumer (CI only):** `ci/interface_consumer/` is a minimal
  package outside the released set (an ignore marker keeps bloom and colcon
  away from `ci/`). After the main build, CI builds it in a separate workspace
  overlaid on the install space; its test imports the interfaces and
  round-trips a `Calibrate` result containing `GateResult`s through rclpy
  serialization.

## 13. Risks

- `pymoveit2` is a single-maintainer project. Mitigation: we use a small part
  (MoveGroup goals, cancel); if it breaks on a distro, replacing it is a
  contained change in one module.
- MoveIt `compute_ik` behaviour differs across distros (timeout handling).
  CI on all three.
- Author new to ROS 2: the custom surface is one action and two messages; it
  gets a design review before node code is written.
