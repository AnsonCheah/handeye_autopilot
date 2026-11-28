Reviewed `mount-sweep` at `4f94502` against `main`. All 30 focused Jazzy tests passed. ROS launch tests and the robot sweep were reviewed statically.

### Pass 7

- **ACCEPTED** — The stalled-clock case with feedback delayed beyond 250 ms and the mixed-clock stationary rejection are documented in §3.2. [01-architecture.md:157](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:157)
- **ACCEPTED** — The single-client cancellation fallback and the fake’s reliance on normal executor scheduling remain documented. [Review README.md:30](/home/anson/workspaces/handeye_autopilot/docs/design/reviews/mount-sweep-xhigh/README.md:30)
- **RESOLVED** — The new test fails if the watch tracks the last arrival’s stamp instead of the newest stamp. Cancelled-run result text now reaches the sweep report. [test_still_watch.py:99](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_still_watch.py:99) · [sweep_mounts.py:157](/home/anson/workspaces/handeye_autopilot/handeye_autopilot_demos/handeye_autopilot_demos/sweep_mounts.py:157)
- **OPEN · SHOULD-FIX** — After a lost goal reply, even a single-client runner exits on the `~/cancel` acknowledgement. RViz stay mode leaves the stack running while the stop is still unconfirmed. [run_demo.py:102](/home/anson/workspaces/handeye_autopilot/handeye_autopilot_demos/handeye_autopilot_demos/run_demo.py:102)
- **OPEN · SHOULD-FIX** — The round-trip test still passes if the watch closes at the action result. The fake returns to its starting pose before aborting. [ros_harness.py:429](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/ros_harness.py:429)

### Additional findings

- **BLOCKER** — Timely, genuine samples can still arrive too late for the watch. In an in-memory replay with an advancing clock, nine samples recorded the excursion and each arrived within 240 ms. Two newer stationary samples overtook them, so `INVALID_MOTION_PLAN` returned `not_started` and calibration could dispatch a spare. [node.py:1234](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:1234)
- **BLOCKER** — Both ROS-free OpenCV CI jobs fail during test collection. The new watch test imports the ROS node unconditionally, which raises `ModuleNotFoundError: action_msgs`. [test_still_watch.py:4](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_still_watch.py:4) · [ci.yml:41](/home/anson/workspaces/handeye_autopilot/.github/workflows/ci.yml:41)
- **SHOULD-FIX** — A mistyped `--cases rx+900` runs zero cases and exits 0. When mixed with valid names, the typo silently omits that case from the verdict. [sweep_mounts.py:214](/home/anson/workspaces/handeye_autopilot/handeye_autopilot_demos/handeye_autopilot_demos/sweep_mounts.py:214)
- **SHOULD-FIX** — The robot sweep’s documented mount-accuracy claim includes the moved eye-to-hand board, but its truth check compares only the camera. A passing result with an exact camera pose and a 20 mm board error still passes that check. [mount-sweep.md:44](/home/anson/workspaces/handeye_autopilot/docs/verification/mount-sweep.md:44) · [run_demo.py:193](/home/anson/workspaces/handeye_autopilot/handeye_autopilot_demos/handeye_autopilot_demos/run_demo.py:193)
- **SHOULD-FIX** — The demo lever-bound test derives its expected value using the production `board_centre` helper. Replacing that helper with the board corner reproduces the original error and leaves the test green. [test_mounts.py:54](/home/anson/workspaces/handeye_autopilot/handeye_autopilot_demos/test/test_mounts.py:54)

Merge: BLOCKED