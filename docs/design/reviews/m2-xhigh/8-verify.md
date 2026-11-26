Reviewed `m2-planning` at `9cadf52`. The selected capture record’s TF now sets the next Cartesian start pose and triggers `motion_failed` if it is outside the limits. The probe rejects `CANCELED / PREEMPTED` and a 20° rotation about Y when X was requested. The committed Jazzy results pass both checks. [session.py:181](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:181) · [m2_moveit_probe.py:319](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:319) · [m2_moveit_probe.py:337](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:337)

**BLOCKER**

- **Capture consensus can hide a newer out-of-bounds pose.** In a three-frame rendered bootstrap capture, the selected frame put the tool 0.2 mm inside the box and the last frame put it 0.2 mm outside. The session issued a Cartesian return to seed using the earlier pose. [capture.py:185](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/capture.py:185) · [session.py:181](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:181)

**SHOULD-FIX**

- The probe accepts a successful move to `FAR` whose measured joints stayed at `READY`. [m2_moveit_probe.py:314](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:314)
- The probe passes when both unreachable IK timeout calls return `done: false`, leaving the timeout comparison unverified. [m2_moveit_probe.py:355](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:355)

**Notes**

- 149 tests passed, including both simulated calibration modes. The YAML round trip passed with in-memory I/O because the sandbox blocks temporary files. A native call can defer the test alarm until the five-second watchdog grace period expires. [m2_sim.py:101](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/m2_sim.py:101)
- M3 still needs fresh motion measurements, seed-return tolerances, and clearance for residual TCP deviation. Lyrical and Rolling probe runs are pending. [M3 driver contract](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:164) · [Probe status](/home/anson/workspaces/handeye_autopilot/docs/probes/README.md:11)

M2 merge: BLOCKED