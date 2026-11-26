Reviewed `m2-planning` at `436db6c` read-only. All 118 targeted tests passed, including both simulated calibration modes. MoveIt verification used the committed Jazzy run. Lyrical and Rolling evidence is pending. [Probe results](/home/anson/workspaces/handeye_autopilot/docs/probes/README.md:9)

- **Rotation cap confirmed.** A 60° step with a 1.5 safety factor passes. Larger products are rejected. [config.py:227](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/config.py:227)
- **Printed-side check partly confirmed.** Pre-estimate rotations and rough-estimate paths check the board face. Pre-estimate translations still check only the outline. [bootstrap.py:104](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/bootstrap.py:104) · [bootstrap.py:125](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/bootstrap.py:125)
- **Offset-TCP acceptance confirmed.** The probe rejects an unrounded fraction of `0.99996` and requires dense-path deviation below 0.5 mm and one-tenth of the endpoint-only deviation. Jazzy recorded 0.014 mm versus 4.518 mm. [m2_moveit_probe.py:314](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:314) · [m2_moveit_jazzy.yaml:230](/home/anson/workspaces/handeye_autopilot/docs/probes/m2_moveit_jazzy.yaml:230)

### BLOCKER

- **Bootstrap can move the board out of view while its outline stays in frame.** With a rendered checkerboard tilted 65° at 450 mm, the session accepted the seed and dispatched a 220 mm lateral translation. The endpoint showed the board’s back, and the next capture returned `no_board`. [bootstrap.py:109](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/bootstrap.py:109) · [session.py:408](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:408)

### SHOULD-FIX

- **The offset-TCP probe can accept a failed plan.** A dense response with `FAILURE`, fraction `1.0`, and one stationary point reporting 0 mm deviation passed validation. [m2_moveit_probe.py:314](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:314)
- **The carried-object margin rejects valid tool endpoints.** With a 20 mm margin, the filter rejected a tool 10 mm inside the box even though the carried camera was 110 mm inside. [planning.py:175](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/planning.py:175)
- **The test deadline can still overrun inside native code.** A 1 s deadline raised after 1.25 s in a native mutex wait. The watchdog allows a stuck call five extra seconds. [m2_sim.py:101](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/m2_sim.py:101)

### M3 notes

Use fresh measured tool poses and joints in motion replies. Account for seed-return error in the next bootstrap check and for residual Cartesian TCP deviation at box faces with zero clearance. [01-architecture.md:158](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:158)

M2 merge: BLOCKED