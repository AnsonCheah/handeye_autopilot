Reviewed `m2-planning` HEAD `0407ba5` read-only. All 120 targeted tests passed, including both simulated calibration modes. MoveIt verification used the committed Jazzy results. Lyrical and Rolling evidence is pending. [docs/probes/README.md:11](/home/anson/workspaces/handeye_autopilot/docs/probes/README.md:11)

### Confirmed

- Pre-estimate translations check the printed face. A 220 mm shift that kept the outline in frame but showed the back was rejected. The 110 mm retry passed. [bootstrap.py:122](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/bootstrap.py:122)
- The tool origin uses the exact box boundary. A tool 10 mm inside passed with a 20 mm carried-object margin when the camera was 110 mm inside. [planning.py:177](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/planning.py:177)
- Both offset-TCP paths require fraction `1.0`, `SUCCESS`, and at least two points. Injected failure and one-point responses were rejected. [m2_moveit_probe.py:314](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:314)

### BLOCKER

- **A rough estimate can authorize a carried-camera box violation.** In a rendered eye-in-hand run, bootstrap pose errors of at most 1.05 mm made a 100 mm camera offset fit as 60.7 mm. The core approved a Cartesian centring move with 24 mm of predicted clearance. The true camera endpoint was 15.3 mm outside the box. [session.py:245](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:245)
- **`not_started` discards the measured tool pose.** With the cached pose 0.1 mm inside the box and the reply reporting 0.1 mm outside, the core issued another bootstrap move using the cached start. [session.py:219](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:219)

### SHOULD-FIX

- The translation regression passes with the facing check removed. The outline check alone selects the same 31.25 mm step. [test_planning.py:438](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_planning.py:438)
- The probe accepts two stationary points as the dense 20° trajectory. It also accepts a planning-failure record with 0.2 rad of joint displacement. [m2_moveit_probe.py:314](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:314) · [m2_moveit_probe.py:303](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:303)
- A native mutex wait delayed a 1 s test deadline to 1.25 s. The watchdog gives a stuck call another five seconds. [m2_sim.py:101](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/m2_sim.py:101)

### M3 notes

Use fresh TF and joints in motion replies. Account for measured seed-return error and residual TCP deviation when checking Cartesian paths. Jazzy measured 0.014 mm of TCP deviation, which matters at zero box clearance. [01-architecture.md:158](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:158) · [m2_moveit_jazzy.yaml:237](/home/anson/workspaces/handeye_autopilot/docs/probes/m2_moveit_jazzy.yaml:237)

M2 merge: BLOCKED