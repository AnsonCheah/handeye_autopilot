# handeye_autopilot

Automated, quality-gated hand-eye calibration (eye-to-hand and eye-in-hand)
for ROS 2 Jazzy, Lyrical and Rolling. Work in progress; see the design plan in
[docs/design/](docs/design/README.md).

| Package | Contents |
|---|---|
| `handeye_autopilot` | ROS-free core (`handeye_autopilot.core`) and the ROS 2 node |
| `handeye_autopilot_interfaces` | `Calibrate` action, `QualityReport` and `GateResult` messages |
| `handeye_autopilot_demos` | Panda and UR mock-hardware demos |

## Try it in simulation (M4 preview)

No robot or camera needed: a robot on mock hardware, MoveIt, and a synthetic
camera that renders the calibration board. The full new-user guide comes with
1.0 (M6); this is the short version.

**1. Build** (ROS 2 Jazzy shown; Lyrical and Rolling work the same):

```bash
cd ~/workspaces                       # the colcon workspace that contains this repo
rosdep install --from-paths handeye_autopilot -i -y
colcon build
source install/setup.bash
```

**2. Run a demo.** The robot moves to a seed pose, then calibrates on its own:
seed check, bootstrap (small checked moves to get a first estimate), centring,
planned poses, solve, verification and quality gates. It ends with
`DEMO PASSED` and the error against the true mount, which only a simulation
knows. A run takes 4–6 minutes.

```bash
ros2 launch handeye_autopilot_demos demo.launch.py robot:=panda mode:=eye_to_hand rviz:=true
```

Swap `robot:=panda` for `robot:=ur` (a UR5e) and `mode:=eye_to_hand` (camera
fixed, board on the robot) for `mode:=eye_in_hand` (camera on the robot, board
on the table). In RViz you see the robot, the planning scene (table, the
carried board or camera, the point-cloud pillar MoveIt must avoid), the safety
box, and the camera image with the detected corners. With `rviz:=false` the
same run works headless (this is what CI runs).

With RViz the demo keeps running after `DEMO PASSED` (Ctrl-C ends it) and
prints the command that publishes the result to TF (step 4). A
`move_group ... exit code -11` line on Ctrl-C is MoveIt crashing during its
own shutdown, after the run; it is harmless.

**3. Look at the result.** The run writes `result.yaml` (transforms, every
quality gate, the configuration) and `observations.yaml` to
`~/.ros/handeye_autopilot/demo_<robot>_<mode>/<time>/`.

**4. Drive it yourself, and publish the result to TF.** `auto:=false` starts
everything but sends no goal, and keeps running. The robot must start where
the board is fully visible and tilted; the Panda starts there already (its
ready pose is the demo's seed). In a second terminal:

```bash
ros2 run handeye_autopilot calibrate --mode eye_to_hand --execution moveit
```

When it finishes, publish the calibrated camera mount (the camera driver, here
the synthetic camera, must still be running: the publisher re-checks its TF
subtree):

```bash
ros2 launch handeye_autopilot result.launch.py result_file:=<path to result.yaml>
ros2 run tf2_ros tf2_echo panda_link0 camera_link
```

`calibrate` prints progress and the result, and Ctrl-C cancels the run
(waiting until MoveIt confirms the robot stopped). It is
`ros2 action send_goal` plus what an operator needs: if the node's
goal-accepted reply is lost (DDS can drop it under load) it cancels its own
goal instead of waiting forever. From any other terminal the active run
(whoever started it) stops with
`ros2 service call /handeye_autopilot/cancel std_srvs/srv/Trigger`.

`result.launch.py` refuses a result whose quality gates failed, or one whose
camera frames are already owned by something else in TF.
`execution: manual` instead lets you jog the robot yourself and call
`ros2 service call /handeye_autopilot/capture std_srvs/srv/Trigger` at each
pose.

## Build and test

```bash
cd ~/workspaces            # the colcon workspace that contains this repo
rosdep install --from-paths handeye_autopilot -i -y
colcon build
colcon test --return-code-on-test-failure && colcon test-result --verbose
```

License: Apache-2.0.
