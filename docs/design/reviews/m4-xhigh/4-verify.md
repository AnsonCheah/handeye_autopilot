Reviewed `m4-moveit` at `3f2cb3f`. No files changed.

### Prior findings

| Finding in `3-verify.md` | Status at HEAD |
|---|---|
| Scene changes before dispatch | **PARTIAL.** Requests are sent together, but each point gets its own scene check. [moveit_client.py:346](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/moveit_client.py:346) |
| Cartesian FK box violation | **RESOLVED.** Every FK sample now checks the TCP and, when enabled, the carried-object bound. [node.py:1099](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:1099) · [node.py:1366](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:1366) |
| Demo exit | **RESOLVED.** Runner failure propagates to launch. CI checks the verdict, the runner’s clean exit, and the launch exit code. [demo.launch.py:44](/home/anson/workspaces/handeye_autopilot/handeye_autopilot_demos/launch/demo.launch.py:44) · [test_panda_moveit.py:220](/home/anson/workspaces/handeye_autopilot/handeye_autopilot_demos/test/test_panda_moveit.py:220) |
| Carried-geometry warning | **RESOLVED.** URDF geometry counts only on explicitly named links, including the tool link. Attached objects on rigid links also count. [node.py:868](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:868) |

### Remaining findings

- **BLOCKER — A pre-dispatch scene update can still leave a colliding path approved.** A point-cloud update can invalidate an early point while later requests are being processed. An in-memory replay approved the path 41 ms after that update. The recheck also queries only returned trajectory points, so an obstacle reported during FK can intersect the path between points even if the scene stays fixed throughout the recheck. Both paths can reach `ExecuteTrajectory`. [moveit_client.py:353](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/moveit_client.py:353) · [node.py:970](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:970) · [node.py:1002](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:1002)
- **SHOULD-FIX — Cartesian FK skips the seed-relative limits.** The FK check applies path tolerances and the box. An offset-TCP replay passed with an FK sample 0.186 mm outside the seed envelope. [session.py:249](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:249) · [node.py:1089](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:1089)
- **SHOULD-FIX — Cartesian success can lack fresh joint evidence.** When the final joint read times out, the driver can return `MOTION_OK` and the session substitutes the target’s IK joints, which can represent a different branch. [node.py:1040](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:1040) · [session.py:283](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:283)

**Notes.** Sampled planned-path box checks and scene changes during execution are documented shortcuts. 37 tests passed and 31 MoveIt and demo tests collected. The read-only sandbox blocked full ROS launches, so the four demo combinations remain unverified at HEAD. [01-architecture.md:188](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:188) · [01-architecture.md:208](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:208)

M4 merge: BLOCKED