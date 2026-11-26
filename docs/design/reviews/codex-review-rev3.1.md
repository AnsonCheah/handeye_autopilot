# Codex review — verification of rev 3.1 + M0 skeleton, 2026-11-26

**Hold M0 sign-off.** On pull requests, the dependency gate and downstream consumer can run against a different revision from the main build. The local Jazzy dependency gate passed, and all 9 tests passed against the existing install with the `pymoveit2` smoke test required. Fresh three-distro CI remains unverified.

### Prior findings

| Finding | Status | Why |
|---|---|---|
| M0 dependency gate | PARTIAL | The manifest table, action dependencies, M0 scope, and per-key apt checks are in place. The documented `-i` enumeration skips underlay keys, and CI can build a different revision from the one it gates. |
| M1a fixture outcomes | RESOLVED | The plan separates successful masking, insufficient support, two boards, and genuine IPPE ambiguity, with exact IDs and board coordinates and a committed pixel tolerance required. |
| M1b milestone ownership | RESOLVED | Bootstrap and timestamp cases moved to M2 and M3. M1b specifies a ROS-free capture record, a versioned YAML format, and a committed replay tolerance as an exit criterion. |
| M0 wire contract | PARTIAL | The canonical files, constants, frames, stamps, and NaN convention are explicit. The held-out capture population and RMS pooling still need a precise definition. |
| Action rejection semantics | RESOLVED | Admission rejection carries no result, clock preflight aborts an accepted goal with `TF_UNAVAILABLE`, and M2 probes MoveGroup terminal outcomes. |
| Observability metrics | PARTIAL | Pair selection, axis spread, scaling, and check stages are specified. The exact Jacobian weighting and empty-axis behavior remain ambiguous. |
| M0 downstream proof | RESOLVED | CI defines a separate installed-interface consumer with a nested `GateResult` round trip, records the image and action versions, and requires the smoke test on every leg. |
| M0 interface count | RESOLVED | The deliverables and exit criteria name `Calibrate`, `QualityReport`, and `GateResult`. |

### New findings

- **BLOCKER** [`.github/workflows/ci.yml:43`](/home/anson/workspaces/handeye_autopilot/.github/workflows/ci.yml:43) — Checkout defaults to the PR merge commit, while `action-ros-ci` independently fetches the PR head branch. A dependency or consumer test can therefore validate different code from the main build. Use one checkout for the gate, installed `colcon` build and test, and consumer. [Action checkout implementation](https://github.com/ros-tooling/action-ros-ci/blob/v0.4/dist/index.js#L29564-L29584)
- **SHOULD-FIX** [`01-architecture.md:181`](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:181) — The documented `rosdep keys … -i` excludes packages in the sourced ROS underlay. Document the script’s actual approach: enumerate without `-i`, subtract only source package names, and gate before installing project dependencies. [rosdep implementation](https://github.com/ros-infrastructure/rosdep/blob/master/src/rosdep2/main.py#L501-L526)
- **SHOULD-FIX** [`Calibrate.action:5`](/home/anson/workspaces/handeye_autopilot/handeye_autopilot_interfaces/action/Calibrate.action:5) — The blanket `ABORTED` wording conflicts with the documented terminal outcomes. Spell out `CANCELED` for completed cancellation, `ABORTED` for runtime failures, and `SUCCEEDED` with `passed=false` for a completed solve that fails quality gates.
- **SHOULD-FIX** [`03-milestones.md:57`](/home/anson/workspaces/handeye_autopilot/docs/design/03-milestones.md:57) — The exit criterion calls for goal and feedback serialization, but the consumer only constructs them. Serialize and deserialize both and assert their fields.
- **SHOULD-FIX** [`02-features.md:186`](/home/anson/workspaces/handeye_autopilot/docs/design/02-features.md:186) — The new record requires a tool pose and two IPPE poses even for rejections that occur before those values exist. Define nullable fields for rejected records and require the solver inputs on accepted records.
- **SHOULD-FIX** [`02-features.md:221`](/home/anson/workspaces/handeye_autopilot/docs/design/02-features.md:221) — Specify the formula using the raw pixel-residual Jacobian, including the square root of the IRLS weights, the `f_scale` convention, and the empty qualifying-axis case. At [§G](/home/anson/workspaces/handeye_autopilot/docs/design/02-features.md:257), explicitly pool corner errors from all held-out accepted captures in successful folds and retain the observability-failure condition in [`QualityReport.msg`](/home/anson/workspaces/handeye_autopilot/handeye_autopilot_interfaces/msg/QualityReport.msg:26).

`ros_ws/install/setup.bash` is the correct action output path. Local `catkin_pkg` discovery found exactly the three release packages, and both bloom and rosdep use that discovery API, so the `ci/` markers exclude the consumer. [Action workspace path](https://github.com/ros-tooling/action-ros-ci/blob/v0.4/dist/index.js#L29419-L29421) · [bloom discovery](https://github.com/ros-infrastructure/bloom/blob/master/bloom/packages.py#L76-L77) · [rosdep discovery](https://github.com/ros-infrastructure/rosdep/blob/master/src/rosdep2/catkin_packages.py#L25-L32)

**Applied in the same change:** all new findings (CI pins one ref for checkout and action-ros-ci; §4 gate wording; terminal-status wording in Calibrate.action; consumer round-trips goal/feedback; nullable capture-record fields; Jacobian formula, empty-axis rule, held-out pooling).
