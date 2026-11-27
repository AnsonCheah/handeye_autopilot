# M4 Codex reviews (`xhigh`)

One full review (`1-review`) and eight verification passes (`2-` … `9-verify`),
each with its prompt. Pass 9: **"M4 merge: CLEAR"**; its should-fixes (the
CLI's fallback warning when a stop is not confirmed or the cancel is not
answered, Ctrl-C during discovery, a stronger two-operator test, a docstring)
are fixed in the merge commit.

Recurring themes, all fixed with tests:
- **motion evidence**: goals accepted after the reply deadline, results without
  a terminal status, stale or non-finite joint states, `ok` without fresh
  joints, refused moves without fresh joints;
- **paths MoveIt returns resampled**: a joint-jump check alone misses a branch
  flip spread over small steps, so the measured tool path (FK) must follow the
  straight waypoint path and pass every limit; the carried object's box check
  on planned paths (plan only, check through FK, then execute);
- **scene re-check before `ExecuteTrajectory`**, between trajectory points
  too, with a bounded number of requests in flight (the real `move_group`
  dropped the rest: found by re-running the Panda demo, not by the fake);
- **stopping**: Ctrl-C to the node cancels its motion before it exits; a run
  that becomes invalid cancels its goal in flight; `~/cancel`; an operator CLI
  that cancels its own goal by ID when the goal reply is lost (DDS dropped it
  under load in a test), never another operator's run, and follows a remapped
  action name;
- **design items due in M4**: the box in a named frame, the carried-geometry
  warning, MoveGroup vs ExecuteTrajectory failure tests on each action.

Accepted (documented in 01-architecture §3.2 as `shortcut:`): planned-path box
checks are sampled with a 1 cm margin for the controller's splines; planning
scene changes during, or just before, an `ExecuteTrajectory` motion are not
monitored (as for any MoveIt Cartesian motion).
