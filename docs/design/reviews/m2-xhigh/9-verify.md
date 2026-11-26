Reviewed `m2-planning` at `c3fd725`. After seed initialization, every frame with a measured tool pose is checked, and the newest timestamp sets the next Cartesian start. The probe requires the successful move’s joints within 0.01 rad of `FAR` and both unreachable IK calls to complete with `NO_IK_SOLUTION`. The committed Jazzy run passes. [session.py:184](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:184) · [m2_moveit_probe.py:314](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:314) · [m2_moveit_probe.py:366](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:366)

### BLOCKER

- **The seed capture can authorize motion from outside the safety box.** The per-frame check skips the seed capture, and initialization overwrites the newest pose with the consensus frame’s pose. In a rendered three-frame capture, the selected pose was 0.2 mm inside and the newest was 0.2 mm outside. The session emitted a bootstrap move. [session.py:187](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:187) · [session.py:315](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:315)

### SHOULD-FIX

- **Centring checks visibility from the consensus pose.** With 1 mm of drift by the newest frame, the checked path cleared a 44.4 px margin, but the path from the measured start reached 44.0 px. The session emitted the move. [session.py:509](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:509)
- **The probe accepts a Cartesian no-op.** The 50 mm move still passed when I substituted unchanged end joints in its `SUCCEEDED` record. [m2_moveit_probe.py:334](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:334)

### Notes

- 268 tests passed, including both simulated modes. The read-only runs skipped 15 tests requiring temporary files. Native calls can defer the test alarm until the five-second watchdog grace period expires. [m2_sim.py:101](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/m2_sim.py:101)
- Fresh motion measurements, seed-return tolerance, and residual TCP clearance remain M3 driver items. Lyrical and Rolling probe runs are pending. [01-architecture.md:164](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:164) · [docs/probes/README.md:11](/home/anson/workspaces/handeye_autopilot/docs/probes/README.md:11)

M2 merge: BLOCKED