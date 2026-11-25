# Checkpoint 1 — Architecture and interfaces (rev 2, after Codex review)

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
per-capture rejection, accuracy gates, verification, and simulation tests.
Solving uses OpenCV + SciPy. Interop (v1.1): export observations in
`industrial_calibration`'s format so users can cross-check with its solver.

## 2. Reuse map

| Need | Use (exists) | We write |
|---|---|---|
| Image conversion | `cv_bridge` | Nothing |
| Camera model, raw vs rectified | `image_geometry` `PinholeCameraModel` | Validation of `CameraInfo` (§7) |
| Board detection | OpenCV `findChessboardCornersSB`, ArUco/ChArUco detectors | Compatibility shim for OpenCV 4.6 (24.04) and 4.10 (26.04) |
| Board pose per frame, both planar solutions | OpenCV `solvePnPGeneric` with `SOLVEPNP_IPPE` | Nothing |
| Board printouts | OpenCV `generateImage` | A short script |
| Intrinsics | `camera_calibration` (image_pipeline) | Nothing |
| Initial hand-eye solve | OpenCV `calibrateHandEye`, `calibrateRobotWorldHandEye` | Nothing |
| Refinement and uncertainty | SciPy `least_squares` (robust loss, Jacobian) | Residual function |
| Rotations | SciPy `Rotation` | Nothing |
| Quasi-random sampling | SciPy `qmc.Halton` | Nothing |
| Image + info sync | `message_filters` `ApproximateTimeSynchronizer` | Nothing |
| Robot pose at a stamp | `tf2_ros` Buffer, async wait with deadline | Nothing |
| Motion (plan + execute, cancel) | `pymoveit2` `MoveIt2` | Goal parameters, result checks |
| Reachability + collision | `moveit_msgs/srv/GetPositionIK` on `compute_ik` | A direct client (~20 lines; pymoveit2's IK call sets `is_diff=false`, see §3.1) |
| Collision scene (meshes, point clouds) | MoveIt planning scene, octomap | Docs and demo config only |
| Exposure | Driver's own ROI auto-exposure, set via rclpy `AsyncParameterClient` | ROI update + driver parameter-name map (checkpoint 2 §L) |
| Stray-marker rejection | OpenCV `solvePnPRansac` + ChArUco interpolation | Glue (checkpoint 2 §D4) |
| Publish result | `tf2_ros static_transform_publisher` from a launch file | Launch file |
| Visualisation | RViz2, `MarkerArray`, `interactive_markers`, OpenCV drawing | Marker builders |
| Run recording | `ros2 bag record` (launch option) | Observations YAML for offline re-solve |
| Start / cancel / progress | ROS 2 action | One action definition |
| Manual capture trigger | `std_srvs/srv/Trigger` | Nothing |
| Robots for demos and CI | UR (`ur_robot_driver` mock hardware), Panda (`moveit_resources_panda`) | Demo launch files |
| Camera simulation | Gazebo + `ros_gz` (optional, realistic) | Lightweight homography renderer for deterministic CI (§10) |

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

- Goal per planned pose: position + orientation constraints on `ik_link` with
  explicit tolerances *(tune)*, finite planning time, and **both** velocity and
  acceleration scaling set in (0, 1] from parameters (a zero scaling means full
  speed in MoveIt; parameter validation rejects 0).
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
| `handeye_autopilot_interfaces` | `ament_cmake` | One action, one message. Declares `rosidl_default_generators` (build), `rosidl_default_runtime` (exec), `member_of_group rosidl_interface_packages`. |
| `handeye_autopilot_demos` | `ament_python` | UR and Panda demos. Only this package depends on robot configs. |

`package.xml` declares all system deps via rosdep keys (`python3-numpy`,
`python3-scipy`, `python3-opencv`, `python3-yaml`, `cv_bridge`,
`image_geometry`, `tf2_ros`, `message_filters`, `moveit_msgs`, `pymoveit2`,
`interactive_markers`, `std_srvs`). CI builds and tests a normal **installed**
(not symlink) build on each distro's own OS image with distro OpenCV.

## 5. The one custom interface

### `Calibrate.action`
```
# Goal
string mode                # "eye_to_hand" | "eye_in_hand"
string execution           # "moveit" | "manual"
string profile             # parameter profile name; empty = node parameters
---
# Result
uint8  error_code          # NONE, INVALID_GOAL, BUSY, NO_CAMERA_INFO, TF_UNAVAILABLE,
                           # IK_SERVICE_MISSING, MOTION_FAILED, NOT_ENOUGH_SAMPLES,
                           # SOLVE_FAILED, CANCELED, INTERNAL
bool   transform_valid     # an estimate exists (true even when quality failed)
bool   passed              # all quality gates passed
geometry_msgs/TransformStamped camera_transform   # parent -> camera optical frame
geometry_msgs/TransformStamped target_transform   # board in tool (e2h) / base (eih)
handeye_autopilot_interfaces/QualityReport quality
string result_file
string message
---
# Feedback
string phase
uint32 accepted
uint32 rejected
string last_rejection
```

**Semantics:**
- Goal rejected (not accepted) if a run is already active or `mode`,
  `execution`, `profile` are invalid.
- Configuration is snapshotted at goal acceptance; parameter changes mid-run
  don't apply.
- `SUCCEEDED`: the run completed. `passed` may be false (quality gates failed)
  with `transform_valid` true.
- `ABORTED`: a runtime failure; `error_code` says which.
- `CANCELED`: cancel requested and completed (see §6).
- `~/capture` (`std_srvs/Trigger`, manual mode): success means "capture
  started"; the accept/reject outcome arrives in feedback. Returns
  `success=false` outside the manual capture phase or while a capture runs.

`QualityReport.msg`: per-gate value, limit, pass, and group (sanity /
precision / accuracy evidence); inlier count; reprojection stats; k-fold spread
of the camera transform (mm, deg); held-out reprojection; observability
measures. See checkpoint 2 §F for what each group does and does not prove.

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
  `MOTION_FAILED` and a message telling the operator to check the robot.
- Cancel during capture or IK: stop issuing new requests, drop outstanding
  results, report `CANCELED`.
- Launch tests cover cancel during motion, during capture, and during IK.

## 7. Camera input

- `cv_bridge` for images, `image_geometry.PinholeCameraModel` for geometry.
  Parameter `image_is_rectified`:
  - raw: use `K` and `D`; supported distortion models `plumb_bob` and
    `rational_polynomial`; others rejected with a clear message
    (`shortcut: no fisheye in v1, add via cv2.fisheye if requested`);
  - rectified: use `P` with zero distortion.
- Reject uncalibrated `CameraInfo` (`K` all zero). Reject non-default binning or
  ROI in v1 (`shortcut: binning/ROI unsupported, scale K when someone needs it`).
- Subscriptions use sensor-data QoS; `CameraInfo` is cached (some drivers
  publish it latched or rarely). `ApproximateTimeSynchronizer` with a small
  slop for image + info.

## 8. Capture timing

- `use_sim_time` must match across the node, robot and camera (checked at
  start: warn if the image stamps and the node clock disagree by more than a
  threshold).
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
- **Ownership checks before publishing:** the chain `root → optical` must be
  static (checked twice); `root` must have **no existing parent** in TF (from
  a URDF mounting joint, a driver, or a previous calibration). If it has one,
  refuse and tell the user which frame to drop or which `root` to use instead.
  A cycle check is implied by the same lookup.
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

1. **v1: RViz2 + command line.** RViz config: safety box (interactive),
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
  IDs and board-frame coordinates (catches ChArUco legacy-pattern changes).
- Node: `launch_testing` with the synthetic camera, a stub `compute_ik`
  server, a fake MoveGroup action server; cancel, timing and TF-ownership
  tests from §6, §8, §9.
- MoveIt integration: Panda mock-hardware demo end to end in CI (Jazzy at
  least).
- CI: GitHub Actions `ros-tooling/action-ros-ci`, matrix {Jazzy, Lyrical,
  Rolling}; `ruff`; `mypy` on `handeye_autopilot.core`.

## 13. Risks

- `pymoveit2` is a single-maintainer project. Mitigation: we use a small part
  (MoveGroup goals, cancel); if it breaks on a distro, replacing it is a
  contained change in one module.
- MoveIt `compute_ik` behaviour differs across distros (timeout handling).
  CI on all three.
- Author new to ROS 2: the custom surface is one action and one message; it
  gets a design review before node code is written.
