Reviewed `m4-moveit` at `fab2f73`. No files changed.

### Pass 5 findings

- **PARTIAL · BLOCKER · Lost goal reply.** `~/cancel` can stop an active run without a goal handle. The documented Jazzy CLI still waits indefinitely for acceptance before installing its Ctrl-C cancellation handler. The prior log shows calibration proceeding after the reply was lost. Calling the service stops the run but leaves the CLI waiting for acceptance. [node.py:343](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:343) · [README.md:55](/home/anson/workspaces/handeye_autopilot/README.md:55) · [Jazzy send_goal.py:141](/opt/ros/jazzy/lib/python3.12/site-packages/ros2action/verb/send_goal.py:141) · [Test log:64](/home/anson/workspaces/log/test_2026-11-27_16-29-47/handeye_autopilot/stdout_stderr.log:64)
- **RESOLVED · Midpoint collision regression.** The test rejects a path whose endpoints are valid and whose midpoint is invalid, then accepts it with the obstacle off the path. [test_node_moveit.py:340](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_moveit.py:340)
- **RESOLVED · TF publishing instructions.** Both guides use `auto:=false` to keep the camera’s TF subtree live for publishing. [README.md:49](/home/anson/workspaces/handeye_autopilot/README.md:49) · [m4-simulation.md:39](/home/anson/workspaces/handeye_autopilot/docs/verification/m4-simulation.md:39)

### Other M4 findings

- **SHOULD-FIX · Fresh joints on skipped moves.** A refused Cartesian path returns the joint sample taken before planning, or no sample if that read timed out. The driver contract requires fresh joints for ordering spares after `not_started`. [node.py:988](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:988) · [node.py:1009](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:1009) · [01-architecture.md:167](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:167)
- **SHOULD-FIX · Demo results.** The verification guide still contains `RESULTS_TABLE` where the four demo outcomes belong. [m4-simulation.md:56](/home/anson/workspaces/handeye_autopilot/docs/verification/m4-simulation.md:56)

**Notes.** The accepted scene-recheck window can exceed the documented “few tens of ms.” An in-memory check took 285 ms for 201 samples with simulated 2 ms replies. 37 targeted tests passed and 35 integration tests collected. The read-only sandbox prevented full demo launches. [01-architecture.md:218](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:218) · [moveit_client.py:374](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/moveit_client.py:374)

M4 merge: BLOCKED