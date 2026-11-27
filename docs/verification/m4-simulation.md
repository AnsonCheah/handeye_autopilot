# M4 verification: MoveIt mode in simulation

What M4 built, and how you can check it yourself on a laptop (no robot, no
camera). Real-hardware evidence comes in M5 (`docs/results/`).

## What runs

| Piece | What it is |
|---|---|
| Robot | Panda or UR5e on `ros2_control` mock hardware: commanded joints become measured joints, no physics |
| MoveIt | `move_group` with the robot's own config; serves `compute_ik`, planning, Cartesian paths, the planning scene |
| Scene | a table, the carried board (eye-to-hand) or camera (eye-in-hand) attached to the hand, the fixed one as a world object, and a point-cloud pillar MoveIt must avoid (octomap) |
| Camera | `synthetic_camera`: renders the ChArUco board from the **true** mounts (never published as TF), stamped with the robot pose it shows |
| Calibration | `calibrate_node` in `execution: moveit`: seed check, bootstrap, centring, planning, moves and captures, solve, k-fold verification, quality gates |

## Run it

Build once (README "Try it"), then each of these runs unattended and ends with
`DEMO PASSED` and the error against the true mount; the launch exits 0 on a
pass, 1 on a failure:

```bash
ros2 launch handeye_autopilot_demos demo.launch.py robot:=panda mode:=eye_to_hand rviz:=true
ros2 launch handeye_autopilot_demos demo.launch.py robot:=panda mode:=eye_in_hand rviz:=true
ros2 launch handeye_autopilot_demos demo.launch.py robot:=ur mode:=eye_to_hand rviz:=true
ros2 launch handeye_autopilot_demos demo.launch.py robot:=ur mode:=eye_in_hand rviz:=true
```

What to look for in RViz:
- the robot makes small checked moves around the seed first (bootstrap), then
  larger planned moves to varied poses;
- the camera panel shows each capture with its detected corners in green and
  the board outline, or the rejection reason in red;
- the translucent blue box is the safety box; the pillar of points is the
  obstacle;
- at the end the terminal prints the quality report and the error against the
  truth.

Publish a result to TF and look at it. The publisher re-checks the camera
driver's live TF subtree, so the camera must still be running: start the demo
with `auto:=false`, send the goal yourself, then publish (README "Try it",
step 4):

```bash
ros2 launch handeye_autopilot_demos demo.launch.py robot:=panda mode:=eye_to_hand auto:=false rviz:=true
ros2 action send_goal --feedback /handeye_autopilot/calibrate \
  handeye_autopilot_interfaces/action/Calibrate "{mode: eye_to_hand, execution: moveit}"
ros2 launch handeye_autopilot result.launch.py result_file:=<result.yaml from the goal's result>
ros2 run tf2_ros tf2_echo panda_link0 camera_link
```

In RViz the camera frames now hang off the robot's base at the calibrated
pose. Stop a run from any terminal with
`ros2 service call /handeye_autopilot/cancel std_srvs/srv/Trigger`.

## Results (Jazzy, this machine)

RESULTS_TABLE

## What the tests prove

`colcon test` runs these on every distro in CI:

| Test | Shows |
|---|---|
| `test_node_moveit.py` (fake MoveIt) | full MoveIt-mode runs in both modes, one with a TCP known only to TF; a planning failure is a safe skip (a spare replaces it) |
| | **planning-only success caught**: MoveIt reports SUCCEEDED but nothing moved → `MOTION_FAILED` (for both MoveGroup and ExecuteTrajectory) |
| | **cancel without a terminal state** → `ABORTED / MOTION_FAULT`; new goals rejected until `~/clear_fault` succeeds (both actions) |
| | cancel during motion, during IK; Ctrl-C to the node cancels the motion before it exits; a late-accepted goal is cancelled and keeps the fault latched |
| | Cartesian paths with a joint jump, a NaN fraction, a swing off the straight line, an invalid scene, or a limit broken on the measured path are never executed |
| | the carried object's box check on planned paths (plan, check through FK, then execute) |
| `test_panda_moveit.py` (real MoveIt) | `compute_ik` timeout contract on this distro; **collision rejection** by `compute_ik`; **only the attached board collides** → rejected, accepted once detached; the Panda demo end to end |
| `test_session.py` | the bootstrap failure names the visibility cap when the seed view is too tight |

## Not covered yet

- Real arm and camera (M5): optics, lighting, real robot timing.
- Gazebo-level physics: mock hardware reaches every commanded joint exactly.
- Obstacles that appear while an `ExecuteTrajectory` motion runs (MoveIt does
  not monitor them either; 01-architecture §3.2).
