Verified `m2-planning` at `391d78c`. All 91 planning, session, and import-purity tests passed. MoveIt evidence comes from the committed Jazzy run. Lyrical and Rolling are pending. [Probe results](/home/anson/workspaces/handeye_autopilot/docs/probes/README.md:11)

| Prior finding | Status | Evidence |
|---|---|---|
| (2) Cartesian start omitted from sag check | **RESOLVED** | The start gets the same clearance check as the waypoints. [session.py:198](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:198) |
| NEW-B Planning estimate margin | **RESOLVED** | Candidate filtering applies the execution margin before IK. [planning.py:335](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/planning.py:335) |
| NEW-C Fresh joints on `not_started` | **RESOLVED** | The reply updates the joints used for spare ordering. [session.py:219](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:219) · [session.py:341](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:341) |
| (8) Native call defers deadline | **PARTIAL** | The watchdog kills a stuck process at deadline + 60 s. [m2_sim.py:101](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/m2_sim.py:101) |
| Orientation between waypoints | **RESOLVED** | The half-segment margin rejects the original ±179.6° case. [session.py:254](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:254) · [test_session.py:346](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_session.py:346) |
| Empty offset-TCP trajectory | **RESOLVED** | Both variants must contain trajectory points. [m2_moveit_probe.py:314](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:314) |

### Remaining M2 findings

- **BLOCKER — Bootstrap visibility can approve the wrong rotation.** With an accepted 120° step and `step_safety=3`, the inflated 360° check interpolates as zero rotation. After 13 exact synthetic captures, the session dispatched the 120° move and put the entire board behind the camera. [session.py:410](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/session.py:410) · [transforms.py:57](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/transforms.py:57)
- **SHOULD-FIX — Enforce the test deadline in native code.** A native call deferred a 1 s alarm to 1.14 s in verification. The 120 s acceptance test has the same gap. [m2_sim.py:101](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/m2_sim.py:101) · [m2.yaml:7](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/spec/m2.yaml:7)
- **SHOULD-FIX — Validate the offset-TCP probe’s result.** A fraction of `0.99996` rounds to `1.0`, and the probe accepts 5 mm dense-path deviation against a 4.52 mm endpoint-only baseline. [m2_moveit_probe.py:193](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:193) · [m2_moveit_probe.py:314](/home/anson/workspaces/handeye_autopilot/tools/probes/m2_moveit_probe.py:314)

### M3 contract notes

§3.2 records fresh measured TF and joints, full Cartesian fraction, and seed-return tolerance. The driver needs to account for residual TCP deviation at box faces, where clearance can be zero. §6 requires terminal cancellation status for both motion actions or `MOTION_FAULT`. [01-architecture.md:163](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:163) · [01-architecture.md:346](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:346)

M2 merge: BLOCKED