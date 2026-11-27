Reviewed `m4-moveit` at `1827075`.

### Pass 8 findings

- **RESOLVED — Cancel with a remapped action.** The fallback rebuilds a handle for its own goal UUID on the original action client. Both cancel and result requests follow the remap. [calibrate_cli.py:52](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/calibrate_cli.py:52) · [test_node_moveit.py:747](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_moveit.py:747)
- **RESOLVED — False `stopped` message for `MOTION_FAULT`.** An in-memory check confirmed that `ABORTED / MOTION_FAULT` now prints `CHECK THE ROBOT`. [calibrate_cli.py:72](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/calibrate_cli.py:72)
- **PARTIAL — Two-operator regression.** The test sends Ctrl-C after a fixed sleep. It can pass without proving that the second goal reached admission or that the fallback sent a cancel request. [test_node_moveit.py:723](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_moveit.py:723)

### Other M4 findings

No remaining blockers found.

- **SHOULD-FIX — The fallback can omit an unconfirmed-stop warning.** A cancel reply timeout is reported as an unknown goal. If the server says the goal already ended, the CLI skips the result, which could contain `MOTION_FAULT`. Both paths were verified in memory. [calibrate_cli.py:60](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/calibrate_cli.py:60) · [calibrate_cli.py:81](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/calibrate_cli.py:81)
- **SHOULD-FIX — Ctrl-C during discovery waits for discovery to finish.** If discovery times out, the CLI exits 1 despite the interrupt. It submits no goal. [calibrate_cli.py:89](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/calibrate_cli.py:89)

**Notes.** The planned-path box-check docstring describes linear controller interpolation, while the implementation comment correctly describes splines and the 1 cm margin. [node.py:1141](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:1141) · [node.py:1159](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:1159)

114 targeted tests passed and 41 integration tests collected. The sandbox blocked live DDS verification. All four final-code Jazzy demos passed in the supplied results. No files changed. [m4-simulation.md:55](/home/anson/workspaces/handeye_autopilot/docs/verification/m4-simulation.md:55)

M4 merge: CLEAR