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
  are released on all three (pymoveit2 is not used since M4, §2).
- **Python only** (rclpy). The only CMake package is the interface package.
- **Conventions:** SI units, REP 103/105, ROS quaternion order (x,y,z,w).

## 1. Landscape and where this package fits

| Project | What it does well | Why it doesn't cover this need |
|---|---|---|
| `easy_handeye2` | Simple GUI, OpenCV solvers, publishes result | Samples collected by hand; no pose planning, motion or quality gates; not on the build farm |
| `industrial_calibration` + `industrial_calibration_ros2` (ROS-I) | Ceres solver, covariance, noise and homography analysis, several target types, RViz panel | Data collection is manual (save on trigger); no pose generation or motion; not on the build farm; C++ with no Python API, so we can't depend on it from a farm-released Python package |
| `moveit_calibration` (ros2 branch) | RViz plugin, automatic sampling around a pose | Unreleased for ROS 2, 59 open issues |
| `robot_calibration` | Released; full kinematic calibration | Different scope (robot kinematics), heavier setup |
| `pymoveit2` | Released rclpy wrapper for MoveGroup, IK, planning scene | A building block, not a calibration tool. Planned for reuse; dropped in M4 because its API differs between distro releases (§2) |

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
| Robot pose at a stamp | `tf2_ros` Buffer, `lookup_transform` with a timeout (waits on the run's worker thread while TF arrives on the executor's others) | Nothing |
| Motion (plan + execute, cancel) | `move_group`'s `MoveGroup` and `ExecuteTrajectory` actions and `compute_cartesian_path` and `compute_fk` services (`moveit_msgs`), through rclpy clients. pymoveit2 was dropped in M4: Lyrical's release lacks `get_goal_future` (probe finding 9), and what the node needs from it is a few request fields | `ros/moveit_client.py`: request construction, goal bookkeeping (status, planned trajectory, cancel, goals still active), result checks |
| Reachability + collision | `moveit_msgs/srv/GetPositionIK` on `compute_ik` | A direct client in `ros/moveit_client.py` (pymoveit2's IK call sets `is_diff=false`, see §3.1) |
| Collision scene (meshes, point clouds) | MoveIt planning scene, octomap | Docs and demo config only |
| Exposure | Driver's own ROI auto-exposure, set via rclpy `AsyncParameterClient` | ROI update + driver parameter-name map (checkpoint 2 §L) |
| Stray-marker rejection | OpenCV `solvePnPRansac` + ChArUco interpolation | Glue (checkpoint 2 §D4) |
| Publish result | `tf2_ros` `StaticTransformBroadcaster` | `publish_result`: a small node that re-checks ownership on the live TF tree, then broadcasts; `result.launch.py` runs it |
| Visualisation | RViz2, `MarkerArray`, OpenCV drawing | Marker builders |
| Run recording | `ros2 bag record` (launch option) | Observations YAML for offline re-solve (`core/observations.py`); result YAML (`core/results.py`); `resolve` CLI (`cli/resolve.py`) |
| Start / cancel / progress | ROS 2 action | One action definition; M4: `ros/calibrate_cli.py`, a small action client (`ros2 run handeye_autopilot calibrate`), because `ros2 action send_goal` waits forever when the goal reply is lost and then cannot cancel |
| Manual capture trigger, finish, clear fault | `std_srvs/srv/Trigger` | Nothing |
| Node parameters and profiles | rclpy parameters; YAML profile files | A loader mapping parameters and profiles onto the `core` config dataclasses (§5a) |
| Camera subtree and TF tree queries | `tf2_ros` Buffer (`all_frames_as_yaml`), a `/tf_static` subscription | Subtree walk + ownership checks (§9) |
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
| Plan + execute to joints | `moveit_msgs/action/MoveGroup` | `move_action` |
| Straight tool path (checked moves) | `moveit_msgs/srv/GetCartesianPath`, then `moveit_msgs/action/ExecuteTrajectory` | `compute_cartesian_path`, `execute_trajectory` |
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

### 3.2 Motion (MoveIt actions)

- Goal per planned pose: a **joint goal** equal to the IK solution kept for
  that candidate (checkpoint 2 §C5), so execution reaches the same arm
  configuration the ordering was computed for (a pose goal could pick another
  IK branch and a long wrist flip). Finite planning time, and **both** velocity
  and acceleration scaling set in (0, 1] from parameters (a zero scaling means
  full speed in MoveIt; parameter validation rejects 0).
- If planning to the joint goal fails from the actual current state, the
  candidate is skipped and a spare is used.
- Success requires: goal accepted, terminal status `SUCCEEDED`, MoveIt error
  code `SUCCESS`, **and** the measured joints and TF pose within tolerance of
  the target after settling. A planning-only `move_group` can report success
  without moving, and an out-of-limit joint goal is clamped to the soft limit
  and still reported `SUCCESS` (probe, `docs/probes/`); the end-state check
  catches both.
- `not_started` vs `failed` is decided from evidence, not the error code: a
  planning failure comes back as `ABORTED` with the generic `FAILURE` code
  (probe). `not_started` needs all of: `ABORTED`, an **empty planned
  trajectory** in the MoveGroup result (nothing existed to execute), and
  measured joints unchanged from the start (within tolerance). Anything else,
  or joints unknown, = `failed` (a motion that moved and came back must not
  pass as a safe skip). The result's `executed_trajectory` is always empty on
  Jazzy (probe), so it is not used.
- **Checked short moves are Cartesian.** Bootstrap, centring and recovery moves
  (`Move.cartesian`) carry `waypoints`: tool poses along the straight tool
  path, at most 1° / 5 mm apart *(tune)*. The driver converts each through the
  fixed `ik_link → tool_frame` transform (§3.1) and passes them all to the
  Cartesian planner, so a TF-only TCP follows the checked path too (the
  planner interpolates the IK link; with only the end pose given, a rotation
  about an offset TCP would cut a chord, ~4.6 mm at mid-path for 20° at
  300 mm; measured 4.5 mm vs 0.014 mm with 1° waypoints, probe). The core
  checks the limits at every waypoint and at the start. The carried object's
  box check always uses the `L_max` ball around the tool origin (plus half the
  board diagonal in eye-to-hand), never the rough estimate (a bootstrap
  estimate misplaced a 100 mm camera offset by 40 mm in review). That ball is
  unchanged by rotation about the tool origin and sweeps a convex hull along
  the straight tool path, so the waypoint check is exact for the convex box.
  Waypoint spacing is capped at 5°.
- The driver reports the measured TF tool pose with every completed motion;
  the next move is checked from that pose, and a measured pose outside the
  limits stops the run (`motion_failed`). Orientation from the seed can exceed
  the endpoints' by half a waypoint segment, so the orientation limit is
  checked with that margin; the path's start is checked like its waypoints.
- **M3 driver contract** (from the M2 reviews): report `ok` only with a fresh
  measured TF tool pose (the core falls back to the target only for
  simulation); send fresh joints with every reply, `not_started` included
  (spares are ordered from them); require Cartesian fraction 1.0; the residual
  TCP deviation (~0.01 mm with 1° waypoints) is far inside the box margins;
  the bootstrap return to the seed is checked against the nominal seed, so
  the end-pose tolerance must stay well inside the step safety factor. The driver
  requires the whole path (`compute_cartesian_path` must return fraction 1.0
  before the trajectory goes to `execute_trajectory`; a
  shorter path is refused before anything is sent, which is the `not_started`
  evidence for these moves; probe) and sets a joint-space jump threshold
  (MoveIt's default 0 disables the check). The end check is the measured TF
  tool pose against `tool_pose`; the planner may reach it on a different IK
  branch than `Move.joints`, which is only a reference. Planned poses are
  joint goals (only the end pose is captured; MoveIt's planning scene guards
  the path).
- `MoveGroup` reports the goal as executing while it is still planning;
  "executing" is not evidence of motion.
- The safety box, if enabled, is also sent as a `path_constraints` position
  constraint so intermediate motion stays inside it (option; can make planning
  slower, default on when the box is on). That constrains the tool link only.
  When `box_check_moving_object` is on, a planned move is sent as a plan-only
  `MoveGroup` goal; the planned trajectory is checked (joints interpolated at
  ≤ 0.01 rad, FK through `compute_fk`, the carried object's `L_max` ball and
  the tool origin inside the box, 1 cm inside its faces for the controller's
  spline between points, at every sample) and only then executed with
  `ExecuteTrajectory`. A plan that leaves the box is a `not_started` skip
  (nothing was sent). As built in M4, replacing the planning-scene walls
  planned earlier: walls would also have to be excluded from collisions with
  the robot itself (an allowed-collision matrix over objects we don't own).
  `shortcut:` sampled, not swept.
- Cartesian paths are checked by the node too, before anything is sent:
  fraction exactly 1.0 (NaN refused), finite joints, no joint step between
  path points above `moveit.revolute_jump_threshold` (Jazzy's MoveIt accepts
  the threshold field but does not apply it), and, because MoveIt returns the
  path re-timed and resampled (a branch flip can be spread over small steps),
  the tool path itself: FK of the trajectory (joints interpolated at ≤ 0.01
  rad) must stay within the end-pose tolerances of the straight waypoint path,
  and every FK sample must pass the same limits as the waypoints (box with the
  carried object's ball, seed envelope, orientation limit), exactly: the core
  checked the nominal waypoints, and the planned path may deviate within the
  tolerance (M4 review). A failed check is `not_started`.
- Every reply carries fresh measured joints: a motion whose end joints cannot
  be read is `failed`, and so is a refused move when no fresh joints arrive
  after it (the robot driver has stopped reporting).
- Before any `ExecuteTrajectory` goal (Cartesian moves, box-checked planned
  moves) every trajectory point is re-checked against the current planning
  scene (`check_state_validity` at the points and between them, joints
  interpolated at ≤ 0.01 rad, at most 8 requests in flight: a service keeps
  only ~10 queued requests and drops the rest unanswered, seen with the real
  `move_group` in M4; typically well under a second): unlike a `MoveGroup` goal, an
  `ExecuteTrajectory` goal does not re-check a path planned earlier.
  `shortcut:` MoveIt does not monitor scene changes during an
  `ExecuteTrajectory` execution, and a scene update that arrives during this
  check is treated the same way: obstacles that appear
  then or mid-motion are the operator's, as with any MoveIt Cartesian motion
  (accepted in the M4 review).
- A goal in flight is also cancelled when the run becomes invalid (clock
  reset, CameraInfo change), not only noticed after it ends.
- Joint measurements count only when received after the request **and**
  stamped after it (a replayed or queued message is not evidence), and only if
  finite.

### 3.3 Validation chain per candidate

1. Our robot-agnostic filters (numpy): safety box (toggleable), seed-relative
   envelope, orientation limit from the seed, camera visibility. Visibility
   needs a current estimate of both unknowns; before the preliminary solve only
   the probe moves (§checkpoint 2 B) run, so no visibility prediction is needed
   there.
2. `compute_ik` (§3.1). If the service is absent, only manual mode is allowed.
3. Optional plan-only check (MoveGroup `plan_only`), off by default. Not
   built in M4 (`shortcut:` a planning failure at execution time is already a
   safe skip with a spare; add the check if skips waste too much time).

Every rejection is recorded with its reason.

### 3.4 Safety box

`bounds.enabled: true|false`. Pose + size in a named frame (`session.box_frame`, empty = the base frame;
the node resolves it into the base frame through TF once at goal start,
`TF_UNAVAILABLE` if missing; the core gets a posed box in the base frame). Constrains the `tool_frame` origin, optionally the carried object
too (board corners for eye-to-hand, the camera origin for eye-in-hand). Defined in parameters,
shown in RViz as a marker (`shortcut: not editable in RViz, add an interactive
marker if users ask`). The envelope around the seed and the orientation limit
from the seed (checkpoint 2 §C4) always apply; the box adds a user-placed
volume on top. Disabled box = limits come from the envelope, orientation
limit, `compute_ik` and visibility. With the box on, the seed must be inside
it (else `seed_rejected`), and checked short moves are limit-checked at every
waypoint (§3.2).

The collision warning checks both representations of the carried board or
camera: attached collision objects in the planning scene
(`get_planning_scene`) on a link rigid with the tool (the tool frame, the IK
link and links fixed below them), and URDF collision geometry on the links
named in `moveit.carried_links` (the robot's own hand geometry is not the
carried object, so URDF geometry counts only when named). Warn only if
neither exists (as built in M4: a log warning at the start of a MoveIt-mode
run).

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
| `handeye_autopilot` | — | `python3-numpy`, `python3-scipy`, `python3-opencv`, `python3-yaml`, `rclpy`, `cv_bridge`, `image_geometry`, `tf2_ros`, `geometry_msgs`, `sensor_msgs`, `std_srvs`, `moveit_msgs`, `handeye_autopilot_interfaces`; M3: `std_msgs`, `builtin_interfaces`, `tf2_msgs`, `visualization_msgs`, `ament_index_python`, `launch`, `launch_ros`, `ros2bag`, `tf2_ros_py`, `rosgraph_msgs`; M4: `action_msgs`, `shape_msgs` (`pymoveit2` removed) | `python3-pytest`; M3: `launch_testing`, `launch_testing_ros` |
| `handeye_autopilot_demos` | — | `handeye_autopilot`, `handeye_autopilot_interfaces`, `launch`, `launch_ros`, `moveit_resources_panda_moveit_config`, `ur_moveit_config`, `ur_robot_driver`; M4: `ur_description`, `moveit_configs_utils`, `moveit_ros_move_group`, `moveit_ros_perception`, `moveit_planners_ompl`, `moveit_kinematics`, `moveit_simple_controller_manager`, `moveit_ros_visualization`, `controller_manager`, `joint_state_broadcaster`, `joint_trajectory_controller`, `robot_state_publisher`, `rviz2` and the message and Python keys its nodes import | `python3-pytest`; M4: `launch_testing`, `launch_testing_ros` |

**Dependency gate (CI, every PR, every distro;
`ci/scripts/check_deps.sh`):** keys are listed with `rosdep keys --from-paths
.` **without `-i`** (with `-i` rosdep also drops every key already installed in
the sourced ROS underlay), minus the names of this repo's own packages; for
each key, `rosdep resolve` must name the `apt` installer
and `apt-cache policy` must show an install candidate, checked **before**
anything is installed (so already-installed keys can't hide a pip-only or
missing key); then `rosdep install` runs. Known gaps go in
`ci/known_missing.txt` (distro, key, date, reason): the gate warns instead of
failing, and fails once the package appears so the entry gets removed. CI builds and tests a normal
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
  name (`Z`). `header.stamp` of both = node time when the final solve and its
  k-fold verification finished (one core call, as built in M3).
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
definition. **Frozen at the end of M3** (exercised by the node and its
tests, no changes were needed); a later change bumps the interface package
version.

## 5a. Node surface (M3)

One node, `handeye_autopilot` (`ros2 run handeye_autopilot calibrate_node`).

**Topics in:** `image` (`sensor_msgs/Image`, sensor-data QoS) and
`camera_info` (§7), both relative names to remap; `/tf`, `/tf_static`;
`/joint_states` (only for `~/clear_fault`).
**Topics out:** `~/annotated_image` (detected corners, outline, rejection
reason), `~/markers` (`MarkerArray`: board outline at the last accepted
capture, in the optical frame).
**Action:** `~/calibrate` (`Calibrate`). **Services:** `~/capture`,
`~/finish`, `~/clear_fault`, `~/cancel` (M4) (`std_srvs/Trigger`).
`~/finish` is manual mode only (refused in MoveIt mode: the plan decides
when to solve). `~/cancel` stops the active run, whichever it is, like an
action cancel, from any terminal: an
action client whose goal-accepted reply was lost (DDS discovery under load,
seen in M4 tests) has no goal handle to cancel with. Such a goal ends
`ABORTED` with `error_code` `CANCELED` (the action protocol allows the
`CANCELED` state only after an action cancel request).

**Parameters** (defaults in `config/default.yaml`, all snapshotted at goal
acceptance):
- frames: `base_frame`, `tool_frame`, `board_frame`; the optical frame comes
  from `CameraInfo.header.frame_id`; `camera_root_frame` (empty = the topmost
  ancestor of the optical frame, §9);
- `image_is_rectified` (§7); `output_dir` (each run writes
  `<output_dir>/<UTC time>/result.yaml` and `observations.yaml`);
- `board.*` = `BoardSpec` fields; `capture.*`, `solver.*`, `session.*` = the
  fields of `CaptureConfig`, `SolverConfig`, `SessionConfig` (one parameter per
  field, same name, generated from the dataclasses so they cannot drift);
- `timing.*`: `settle_window`, `settle_translation`, `settle_rotation`,
  `settle_timeout`, `tf_timeout`, `clock_tolerance`, `camera_info_timeout`,
  `frame_timeout` *(tune)*;
- `profiles_dir`: a profile `P` is the file `<profiles_dir>/P.yaml`, a
  mapping with the same groups (`board`, `capture`, `solver`, `session`,
  `timing`, frames) overriding the node parameters for one run. Unknown
  profile (no file) = rejected at admission; a file that fails to parse or
  validate = accepted, then `ABORTED` with `INVALID_GOAL`;
- `moveit.*` (M4, MoveIt mode): `group_name` and `joint_names` (required:
  the SRDF group and its joints in goal order), `ik_link` (empty = tool
  frame), `ik_timeout` (whole s, 0 = server default), `planning_time`,
  `velocity_scaling` and `acceleration_scaling` (in (0, 1]), `motion_timeout`,
  `cancel_timeout`, `service_timeout`, end-state tolerances
  (`joint_tolerance`, `pose_tolerance_translation`, `pose_tolerance_rotation`),
  `cartesian_max_step`, `revolute_jump_threshold`, `box_path_constraint`,
  `carried_links` (§3.4); `session.box_frame` (§3.4).
  The MoveIt names (`compute_ik`, `compute_fk`, `move_action`,
  `compute_cartesian_path`, `execute_trajectory`) are relative: remap them;
- `testing.allow_fault_injection` (default false): adds `~/inject_fault`,
  which sets the motion fault latch, so the latch's admission rule is testable
  before M4 produces real motion faults. Never enabled by the shipped launch
  files.

**Manual mode run:** preflight (§7–8: `CameraInfo`, one image, clock domain,
TF `base → tool`) → phase `manual_capture`: the operator jogs the robot and
calls `~/capture` for each pose (settle, N frames, TF per frame, `core`
capture gates; the outcome arrives in feedback) → the run solves when
`session.target_captures` captures are accepted or the operator calls
`~/finish` (refused below the fold-aware minimum: at least `solver.folds`
captures and `solver.min_captures` in every k-fold training split, i.e.
⌈min_captures × folds / (folds − 1)⌉; the same bound validates
`target_captures`) → solve, verify,
gates (`core.quality.calibrate`) → result YAML, observations YAML, action
result. Phase strings as in `Calibrate.Feedback`.

**MoveIt mode run (M4):** the same preflight, then MoveIt checks: all six
MoveIt interfaces (the five above plus `check_state_validity`) found within `moveit.service_timeout` (`compute_ik` missing
= `IK_SERVICE_MISSING`, any other = `MOTION_FAILED`), the fixed `ik_link →
tool_frame` transform (§3.1, else `TF_UNAVAILABLE`), `/joint_states` for the
group. Then the core session drives the run (02-features §B): seed capture at
the robot's current pose → bootstrap → centring → planning → moves and
captures → solve, verify, gates, with feedback phases as the session reports
them (`verifying` during the k-fold refits). A weak plan is logged and shown in
feedback (`last_rejection_detail`) before execution. Each `Move` is executed
and judged as §3.2 says; a `Capture` is the manual capture step (settle,
frames, TF per frame) with the measured joints. Session failures map to
result codes: `motion_failed`, `recovery_failed` → `MOTION_FAILED` (the
message carries why the motion was judged failed); `seed_rejected`,
`board_lost`, `bootstrap_failed`, `not_enough_samples` → `NOT_ENOUGH_SAMPLES`,
with the session's advice in the message. A cancel during a motion cancels
the goal and waits `moveit.cancel_timeout` for its terminal status
(`CANCELED`), else `MOTION_FAULT` and the latch (§6); a motion past
`moveit.motion_timeout` is cancelled the same way (`MOTION_FAILED` once the
stop is confirmed). With the safety box enabled the node publishes it on
`~/markers` (namespace `safety_box`) when a run starts.

**Synthetic camera node** (`synthetic_camera`, tests and demos): publishes
`image` + `camera_info` at a rate, rendering the board with `core.render` at
the pose given by the true mounts (parameters, never published as TF) and the
latest `base → tool` TF; publishes its own driver subtree
`camera_link → camera_optical_frame` on `/tf_static`, like a real driver, so
the subtree rule (§9) is exercised.

**Operator CLI (M4):** `ros2 run handeye_autopilot calibrate --mode … --execution …`
sends the goal with an ID it chose and prints progress and the result. If no
goal reply arrives within `--accept-timeout`, it cancels that goal ID through
the action's own cancel service (the run may have started; a goal ID never
names another operator's run), waits for its result, and exits 2; Ctrl-C
cancels the goal and waits for its result, exit 130 (before the goal is sent,
nothing is sent); exit 0 only for SUCCEEDED with every gate passed, 1
otherwise, 3 if rejected.

**Launch files:** `calibrate.launch.py` (node + parameters, `record:=true`
adds `ros2 bag record` of the camera, TF, joint-state and annotated-image
topics into `bag_dir`, default the output directory; the bag spans the launch,
not one run); `result.launch.py result_file:=…` runs `publish_result`, which
refuses a result whose ownership checks failed, **re-checks the live TF tree**
(the tree may have changed since the run) and then publishes
`parent → camera_root` as a static transform.

**Run timing details (as built):** an image counts for a capture when it
*arrives* after the settle (so a camera on another clock is still seen and
refused by the clock check, not silently filtered) and was *acquired* after
it; the clock check compares each image's stamp with the node clock when it
arrived; a backwards clock jump aborts the run and clears the TF and image
history; a cancel or a clock jump during the solve still wins; parameters are
snapshotted at admission; every capture's images must match the run's
`CameraInfo` frame and size.

## 6. Concurrency and cancellation

- `MultiThreadedExecutor`. Callback groups: a reentrant group for TF, image
  and camera-info subscriptions; a separate group for the action server and
  services; the run executes in the action's execute callback on an executor
  worker thread and waits only on events, TF lookups and images, each with a
  deadline; it never spins (as built in M3).
- The action server installs a cancel callback that **accepts** cancellation
  (rclpy's default rejects it).
- Cancel during motion: cancel the active motion goal through its rclpy goal
  handle
  (`MoveGroup` for planned moves, `ExecuteTrajectory` for Cartesian ones),
  then wait (with deadline) for its terminal status; only then report
  `CANCELED`. A cancel acknowledgement is not proof the robot stopped; if the
  terminal status doesn't arrive by the deadline, report `ABORTED` with
  `MOTION_FAULT` and a message telling the operator to check the robot.
- **Motion fault latch.** After `MOTION_FAULT` the node refuses every new goal
  until the operator calls `~/clear_fault` (`std_srvs/Trigger`). Also a
  `MOTION_FAULT`: a goal request the server did not answer within 10 s (it may
  still accept it later; if it does, the node cancels it at once and tracks
  it), and a goal that ended without a terminal status. Clearing
  succeeds only when no motion goal from this node (`MoveGroup` or
  `ExecuteTrajectory`, including a late-accepted one) is still active **and**
  fresh `joint_states` (newer than the call) show all joint velocities below a
  threshold over a window *(tune)*, with fresh samples for **every joint ever
  reported** (a stopped arm stream must not clear on a gripper's samples);
  otherwise it returns `success=false` with the reason. The latch survives goal completion and is logged loudly.
- Cancel during capture or IK: stop issuing new requests, drop outstanding
  results, report `CANCELED`.
- **Shutdown (Ctrl-C, SIGTERM).** The node's own signal handler ends an active
  run first, which cancels its motion goal and waits for the stop while the
  executor still runs, and only then shuts down (rclpy's default handler
  would end the context at once and the cancel could never be sent).
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
  node keeps the latest message and uses it if `frame_id` and image size
  match. Its subscription is best-effort + volatile (as built in M3), which is
  compatible with every publisher QoS; camera drivers re-send `CameraInfo`
  with each image, so a latched copy is not needed. A changed `CameraInfo` mid-run aborts the run (`NO_CAMERA_INFO` with a
  message), since the intrinsics are fixed per run.

## 8. Capture timing

- **One clock domain.** Image stamps and TF stamps must come from the same
  clock (`use_sim_time` consistent across node, robot and camera; a camera
  driver stamping with its device clock must be configured to use the host
  clock). Checked in the preflight right after goal acceptance and per
  capture: if image stamps and the node clock differ by more than a threshold
  *(tune)*, the run is **aborted** (`ABORTED`, `TF_UNAVAILABLE`, message naming
  the clock), not warned about.
  `shortcut:` the tolerance is in node-clock seconds; with simulation faster
  than real time, callback queueing scales with the sim rate (150 ms of
  queueing at 4× is 0.6 sim s), so raise `timing.clock_tolerance` for fast
  sims; revisit if a fast-sim user hits false aborts. (It can't be an admission rejection: that
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
    `/tf_static` (checked by listening to `/tf_static` briefly) and never on
    `/tf`;
  - `root` must have **no existing parent** in TF (from a URDF mounting joint,
    a driver, or a previous calibration). If it has one, refuse and tell the
    user which frame to drop or which `root` to use instead;
  - **cycle check:** `root` must not be an ancestor of `parent` (walk
    `parent`'s ancestors in the TF tree); otherwise refuse.
- Parent = `base_frame` (eye-to-hand) or `tool_frame` (eye-in-hand).
- The node runs the checks when the run finishes and writes the outcome to the
  result YAML (`publish`: parent, root, transform, the camera subtree at that
  time; or `ok: false` with the reason). Only after the run's final verdict is
  success **and every quality gate passed** does it create a `.publishable`
  marker next to the file (a failed-gate calibration is kept for inspection;
  `result.launch.py allow_failed:=true` publishes it deliberately). The result
  launch publishes only a result with `ok` **and** the marker, after
  re-checking the live tree (no parent on the root, no cycle, no chain frame
  with any parent on `/tf`, subtree unchanged). The calibration itself is
  never lost to an ownership problem.
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
  `ubuntu:resolute`, Rolling on `ubuntu:resolute` from the `ros2-testing`
  apt repository (its binaries had not synced to main as of 2026-11-26)}
  containers,
  `ros-tooling/setup-ros@v0.7` + `ros-tooling/action-ros-ci@v0.4` with
  `no-symlink-install`; the checkout used by the gate and the consumer and
  action-ros-ci's own checkout are pinned to **one** commit (the PR head SHA,
  passed as `ref` to both); dependency gate (§4); `ruff`; `mypy --strict` on
  `handeye_autopilot.core`. The `moveit_msgs` interface test (the request and
  result fields the node uses) runs (not skips) on every leg
  (`HANDEYE_AUTOPILOT_REQUIRE_DEPS=1`).
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

- `pymoveit2` was a single-maintainer dependency; it was replaced in M4 by
  direct clients on MoveIt's own interfaces (one module, `ros/moveit_client.py`).
  Remaining risk: MoveIt message fields change between releases (the
  `GetCartesianPath` jump thresholds already differ). Mitigation: optional
  fields are set only where they exist, and the interface test checks the
  fields used on every distro.
- MoveIt `compute_ik` behaviour differs across distros (timeout handling).
  CI on all three.
- Author new to ROS 2: the custom surface is one action and two messages; it
  gets a design review before node code is written.
