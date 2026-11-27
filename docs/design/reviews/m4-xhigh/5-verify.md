Reviewed `m4-moveit` at `25061c3`. No files changed.

### Pass 4 findings

- **ACCEPTED — Pre-dispatch scene updates.** Both `ExecuteTrajectory` paths now check interpolated joint samples. An in-memory check rejected a trajectory whose endpoints were valid and midpoint was invalid. The remaining scene-update window is explicitly accepted. [moveit_client.py:356](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/moveit_client.py:356) · [01-architecture.md:211](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:211)
- **RESOLVED — Cartesian FK skipped seed-relative limits.** Every FK sample now passes the session’s box, carried-object, seed-envelope and orientation checks. [node.py:1098](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:1098) · [session.py:299](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:299)
- **RESOLVED — Cartesian success without fresh joints.** A missing post-settle joint measurement now returns `MOTION_FAILED`. [node.py:1043](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:1043)

### Remaining findings

- **BLOCKER — A dropped Jazzy goal reply can leave the operator’s CLI hung while calibration dispatches motion.** The prior test log shows this after ordinary DDS discovery. HEAD adds a delay only to the test. The documented CLI waits indefinitely for acceptance and installs its Ctrl-C cancellation handler afterward. [Test log:54](/home/anson/workspaces/log/test_2026-11-27_16-29-47/handeye_autopilot/stdout_stderr.log:54) · [test_node_moveit.py:549](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_moveit.py:549) · [Jazzy send_goal.py:135](/opt/ros/jazzy/lib/python3.12/site-packages/ros2action/verb/send_goal.py:135)
- **SHOULD-FIX — Add a midpoint-only collision regression.** The committed fake rejects every state, so the test would still pass if the recheck reverted to trajectory points alone. [test_node_moveit.py:274](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_moveit.py:274) · [ros_harness.py:324](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/ros_harness.py:324)
- **SHOULD-FIX — Correct the demo’s TF publishing instructions.** The unattended demo shuts down the synthetic camera. The next documented command requires that camera’s live TF subtree and refuses to publish without it. [m4-simulation.md:39](/home/anson/workspaces/handeye_autopilot/docs/verification/m4-simulation.md:39) · [demo.launch.py:44](/home/anson/workspaces/handeye_autopilot/handeye_autopilot_demos/launch/demo.launch.py:44) · [publish_result.py:101](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/publish_result.py:101)

**Notes.** 37 targeted tests passed and 33 MoveIt and demo tests collected. Full ROS launches require filesystem writes, so the four demo combinations remain unverified at HEAD in this pass.

M4 merge: BLOCKED