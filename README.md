# handeye_autopilot

Automated, quality-gated hand-eye calibration (eye-to-hand and eye-in-hand)
for ROS 2 Jazzy, Lyrical and Rolling. Work in progress; see the design plan in
[docs/design/](docs/design/README.md).

| Package | Contents |
|---|---|
| `handeye_autopilot` | ROS-free core (`handeye_autopilot.core`) and the ROS 2 node |
| `handeye_autopilot_interfaces` | `Calibrate` action, `QualityReport` and `GateResult` messages |
| `handeye_autopilot_demos` | Panda and UR mock-hardware demos |

## Build and test

```bash
cd ~/workspaces            # the colcon workspace that contains this repo
rosdep install --from-paths handeye_autopilot -i -y
colcon build
colcon test --return-code-on-test-failure && colcon test-result --verbose
```

License: Apache-2.0.
