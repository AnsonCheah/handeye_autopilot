Reviewed `m2-planning` at `faf0fe0` read-only. All 95 targeted tests passed, including both simulated calibration modes. MoveIt verification used the committed Jazzy results.

### Confirmed

- Candidate filtering and session motion checks both use the tool-centred lever-bound ball, including the board half-diagonal in eye-to-hand mode. [planning.py:191](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/planning.py:191) · [planning.py:331](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/planning.py:331) · [session.py:245](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:245)
- `not_started` uses the reported pose for the next move and ends the run with `motion_failed` when that pose is outside the limits. [session.py:219](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:219)
- The translation regression distinguishes the checks. It allows 36.38 mm with the facing check and 80.78 mm with the outline check alone. [test_planning.py:436](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_planning.py:436)
- The probe rejected a stationary TCP trajectory and a planning-failure record with 0.2 rad of joint displacement. Jazzy recorded 20° for both paths and identical before-and-after joint readings for the failed goal. [m2_moveit_probe.py:308](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:308) · [Jazzy results](/home/anson/workspaces/handeye_autopilot/docs/probes/m2_moveit_jazzy.yaml:47)

### BLOCKER

- **A capture can report an out-of-bounds tool pose and the session still issues a move.** I reproduced this with three rendered frames. The motion reply put the tool 0.2 mm inside the box, then all three accepted frames put it 0.2 mm outside. The session issued a Cartesian return to seed using the cached in-bounds pose. [session.py:177](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:177) · [session.py:189](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:189)

### SHOULD-FIX

- **The probe accepts a canceled goal as planning-failure evidence.** Changing the recorded result to `CANCELED / PREEMPTED` with zero planned points and unchanged joints still passed. [m2_moveit_probe.py:306](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:306)
- **The TCP probe checks rotation magnitude alone.** A mocked trajectory ending 20° about Y passed the check for the requested 20° about X. [m2_moveit_probe.py:165](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:165) · [m2_moveit_probe.py:199](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:199)

### Notes

- A native call can delay the test alarm. The watchdog fires five seconds after the deadline. [m2_sim.py:101](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/m2_sim.py:101)
- M3 still needs fresh motion measurements, seed-return tolerances, and clearance for residual Cartesian TCP deviation. Lyrical and Rolling probe evidence remains pending. [M3 driver contract](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:164) · [Probe status](/home/anson/workspaces/handeye_autopilot/docs/probes/README.md:11)

M2 merge: BLOCKED