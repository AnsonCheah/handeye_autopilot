Reviewed `m2-planning` at `39cc046`. Confirmed that an out-of-bounds seed frame rejects the seed, the newest measured pose supplies the next Cartesian start, and centring checks visibility from that measured start. The committed Jazzy probe passes, and replaying it with unchanged Cartesian end joints fails. [session.py:185](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:185) · [session.py:321](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:321) · [session.py:516](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:516) · [m2_moveit_probe.py:338](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:338)

### BLOCKER

- **Bootstrap can command a back-facing endpoint.** In a rendered eye-to-hand run with exact robot motion, the inflated visibility check passed at 22.5°, 45°, 67.5°, and 90°. The commanded 78.75° endpoint fell between samples, showed the board’s back, and returned `no_board`. Check the commanded endpoint explicitly. [session.py:432](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:432) · [bootstrap.py:132](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/bootstrap.py:132)

### SHOULD-FIX

- **The Cartesian probe accepts contradictory motion evidence.** A replay with `SUCCEEDED`, MoveIt `FAILURE`, and 0.02 rad of joint displacement passed the 50 mm case. The impossible-path case also passed after I substituted changed end joints. [m2_moveit_probe.py:334](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:334)
- **Seed rejection can give the wrong recovery advice.** With the box disabled, a frame 6 mm from the seed violated a 5 mm envelope. The result advised moving or enlarging the safety box. [session.py:321](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:321)

### Notes

- All 95 planning and session tests passed, including both simulated modes. Native calls can defer the test alarm until the five-second watchdog grace expires. [m2_sim.py:101](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/m2_sim.py:101)
- Fresh motion measurements, seed-return tolerance, and residual TCP clearance remain M3 driver items. The review used the committed Jazzy recording. Lyrical and Rolling probes are pending. [01-architecture.md:164](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:164) · [Probe status](/home/anson/workspaces/handeye_autopilot/docs/probes/README.md:11)

M2 merge: BLOCKED