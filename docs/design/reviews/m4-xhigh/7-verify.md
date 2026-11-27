Reviewed `m4-moveit` at `66de8dd`. No files changed.

### Pass 6 findings

- **RESOLVED — Lost goal reply hang.** The documented CLI bounds the acceptance wait, requests cancellation on timeout, and exits. The fallback has a separate blocker below. [README.md:55](/home/anson/workspaces/handeye_autopilot/README.md:55) · [calibrate_cli.py:85](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/calibrate_cli.py:85)
- **PARTIAL — Fresh joints on skipped moves.** The driver now requests fresh joints after a refusal. If the read times out, it still returns the earlier sample or `None`, so spare ordering can still use stale joints. **SHOULD-FIX.** [node.py:1188](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/node.py:1188) · [session.py:408](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/core/session.py:408)
- **RESOLVED — Midpoint collision test and TF instructions.** The regression still checks a midpoint-only collision. Both guides keep the camera running with `auto:=false`. [test_node_moveit.py:340](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_moveit.py:340) · [README.md:49](/home/anson/workspaces/handeye_autopilot/README.md:49) · [m4-simulation.md:39](/home/anson/workspaces/handeye_autopilot/docs/verification/m4-simulation.md:39)

### New findings

- **BLOCKER — The CLI can cancel another operator’s calibration.** A second operator can press Ctrl-C while their busy rejection is still pending. The fallback cancels whichever run is active, so the first operator’s goal ends `ABORTED / CANCELED`. I reproduced this in memory with the rejection reply pending. [calibrate_cli.py:85](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/calibrate_cli.py:85) · [node.py:273](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/node.py:273) · [node.py:343](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/node.py:343)
- **SHOULD-FIX — CLI interrupt handling.** Ctrl-C while waiting for discovery can still be followed by goal submission. The service fallback can exit as soon as cancellation is requested, before the stop confirmation promised in the README. [calibrate_cli.py:54](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/calibrate_cli.py:54) · [calibrate_cli.py:67](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/calibrate_cli.py:67) · [README.md:67](/home/anson/workspaces/handeye_autopilot/README.md:67)
- **SHOULD-FIX — `~/finish` acknowledges MoveIt captures.** It promises to solve after the current capture, but the MoveIt loop ignores the request and continues. [node.py:330](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/node.py:330) · [node.py:828](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/node.py:828)
- **SHOULD-FIX — UR planning scene in RViz.** HEAD leaves semantic-description publication disabled. The Jazzy RViz log shows the planning-scene display failing to load the robot model. [ur_robot.launch.py:33](/home/anson/workspaces/handeye_autopilot/handeye_autopilot_demos/launch/ur_robot.launch.py:33) · [RViz log:7](/home/anson/.ros/log/rviz2_113132_1795771538684.log:7)

### Notes

The accepted scene-recheck window can exceed the documented “few tens of ms” because batches run sequentially. 42 targeted checks passed and 37 integration tests collected. Full launches require filesystem writes. [01-architecture.md:218](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:218) · [moveit_client.py:374](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/moveit_client.py:374)

M4 merge: BLOCKED