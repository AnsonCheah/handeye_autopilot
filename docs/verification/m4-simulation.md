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
`DEMO PASSED` and the error against the true mount. With RViz the demo then
keeps running (Ctrl-C ends it) so you can look around and publish the result;
with `rviz:=false` (or `stay:=false`) the launch stops and exits 0 on a pass,
1 on a failure, which is what CI runs:

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
driver's live TF subtree, so the camera must still be running: after an RViz
demo it is (the runner prints the exact command), or start the demo with
`auto:=false` and send the goal yourself (README "Try it", step 4):

```bash
ros2 launch handeye_autopilot result.launch.py result_file:=<the printed result.yaml>
ros2 run tf2_ros tf2_echo panda_link0 camera_link
```

On Ctrl-C, `move_group` (MoveIt itself, not this package) may print
`process has died ... exit code -11`: it crashes while shutting down on Jazzy,
after the run is over. It does not affect the result.

In RViz the camera frames now hang off the robot's base at the calibrated
pose. Stop a run from any terminal with
`ros2 service call /handeye_autopilot/cancel std_srvs/srv/Trigger`.

## Results (Jazzy, this machine)

Each run unattended from the seed pose; "error" is the calibrated camera mount
against the true one (only a simulation knows it). All four passed every
quality gate.

| Robot | Mode | Captures (accepted / rejected) | Reprojection RMS | Held-out RMS | Error vs truth | Calibration time |
|---|---|---|---|---|---|---|
| Panda | eye-to-hand | 60 / 1 | 0.24 px | 0.24 px | 0.03 mm, 0.006° | 3.8 min |
| Panda | eye-in-hand | 96 / 1 | 0.19 px | 0.19 px | 0.03 mm, 0.004° | 5.9 min |
| UR5e | eye-to-hand | 60 / 1 | 0.24 px | 0.24 px | 0.05 mm, 0.007° | 3.9 min |
| UR5e | eye-in-hand | 96 / 1 | 0.19 px | 0.19 px | 0.02 mm, 0.005° | 6.9 min |

Eye-in-hand needs more bootstrap rounds (6 vs 3) before the first usable
estimate, hence more captures. The sub-0.1 mm errors reflect a perfect
simulated robot and camera (exact joints, an ideal pinhole renderer); real
hardware (M5) will be dominated by robot accuracy and optics. Videos of the
Panda eye-to-hand and UR5e eye-in-hand runs are kept outside the repository
(4x speed: RViz on top, the camera and each capture as the calibration saw it
below).

## Different mounts

The four demos use one mount each. [mount-sweep.md](mount-sweep.md) moves the
board or camera mount by up to ±200 mm and ±90° per axis, on both robots and
in both modes. 66 of 72 robot runs pass within 0.11 mm and 0.014°; in the other
6, MoveIt reports the board or the camera in collision with the robot before
the calibration starts (four mounted on the tool, two the moved fixed board).

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
