Reviewed `mount-sweep` at `8e6f0be` against `main`. All 29 focused Jazzy tests passed. ROS launch tests and the robot sweep were reviewed statically.

### Pass 6 dispositions

- **RESOLVED** — A gripper-only message now exposes the 680 ms gap with `/clock` stalled. Reverting the fix fails the new test. [node.py:1531](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/node.py:1531), [test_still_watch.py:92](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_still_watch.py:92)
- **RESOLVED** — Both cancellation waits log `MOTION_FAULT`, and `STATUS_UNKNOWN` explicitly warns that the stop is unconfirmed. [run_demo.py:113](/home/anson/workspaces/handeye_autopilot_demos/handeye_autopilot_demos/run_demo.py:113), [run_demo.py:149](/home/anson/workspaces/handeye_autopilot_demos/handeye_autopilot_demos/run_demo.py:149)
- **RESOLVED** — `--help` distinguishes the calibration result wait from the extra 1,500 s launch allowance. [sweep_mounts.py:203](/home/anson/workspaces/handeye_autopilot_demos/handeye_autopilot_demos/sweep_mounts.py:203)
- **ACCEPTED** — The cancel fallback assumes a single-client demo, and the round-trip fake’s 300 ms dwell assumes normal executor scheduling. [Review README.md:26](/home/anson/workspaces/handeye_autopilot/docs/design/reviews/mount-sweep-xhigh/README.md:26)
- **OPEN · SHOULD-FIX** — After a lost goal reply, the runner can exit on the `~/cancel` acknowledgement while motion is still stopping and RViz stay mode keeps the stack up. [run_demo.py:102](/home/anson/workspaces/handeye_autopilot_demos/handeye_autopilot_demos/run_demo.py:102)
- **OPEN · SHOULD-FIX** — The round-trip test still passes if the watch closes at the action result. The fake returns to the start before aborting. [ros_harness.py:429](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/ros_harness.py:429)

### Additional findings

- **BLOCKER** — With `/clock` stalled and joint feedback delayed by one second, queued pre-goal samples satisfied the watch and both post-result reads. I reproduced `not_started` after a 400 ms displacement and return, while the samples showing motion were still in transit. Calibration can dispatch a spare, violating §3.2’s freshness guarantee. [node.py:962](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/node.py:962), [01-architecture.md:153](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:153)
- **SHOULD-FIX** — An unrelated gripper publisher using wall time on `/joint_states` can turn a stationary simulated arm’s plan rejection into `MOTION_FAILED`. Its timestamp dominates the end-gap calculation despite continuous arm feedback. [node.py:1531](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/node.py:1531)
- **SHOULD-FIX** — Every watch test still passed when I replaced the maximum stamp with the last arrival’s stamp. With that regression, an older arm message arriving after the newer gripper message hides the 680 ms gap. [test_still_watch.py:92](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_still_watch.py:92)
- **NIT** — A timed-out run that returns `MOTION_FAULT` now logs the fault, but the sweep report still says `no result (see launch.log)`. Its parser only recognizes `finished:`. [sweep_mounts.py:41](/home/anson/workspaces/handeye_autopilot_demos/handeye_autopilot_demos/sweep_mounts.py:41)

Merge: BLOCKED