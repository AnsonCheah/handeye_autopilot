Reviewed `m2-planning` at `55afb73`. No blockers found.

- Bootstrap now checks the commanded endpoint alongside the inflated samples, both before and after the rough solve. A geometry replay rejected the 78.75° endpoint that passed the 90° inflated check and selected 39.375°. [bootstrap.py:98](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/bootstrap.py:98) · [session.py:448](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:448)
- Seed rejection distinguishes box, envelope, and orientation failures. The committed Jazzy probe passed, and replay rejected both `SUCCEEDED` with `FAILURE` and a refused path with 0.02 rad of joint displacement. [session.py:330](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:330) · [m2_moveit_probe.py:337](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:337)

### Should-fix

- **Halved retries bypass the visibility check.** After a simulated `no_board` at 20°, the session sent the 10° retry even though its sampled path reached 19.92 px against a 20 px image margin. [session.py:458](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:458)
- **The Cartesian probe verifies joint displacement alone.** A replay passed the 50 mm case with `SUCCEEDED` / `SUCCESS` and only 0.02 rad of wrist motion. Verify the measured tool endpoint against the requested pose. [m2_moveit_probe.py:344](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:344)

### Notes

- All 97 planning and session tests passed, including both simulated modes. The capture YAML round trip passed with in-memory I/O. Native calls can defer the test alarm until the five-second watchdog grace expires. [m2_sim.py:101](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/m2_sim.py:101)
- Fresh motion measurements, seed-return tolerance, and residual TCP clearance remain M3 driver items. MoveIt verification used the committed Jazzy recording. Lyrical and Rolling remain pending. [01-architecture.md:164](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:164) · [Probe status](/home/anson/workspaces/handeye_autopilot/docs/probes/README.md:11)

M2 merge: CLEAR