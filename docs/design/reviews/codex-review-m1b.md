# Codex review — M1b (solver and verification), 2026-11-26

**BLOCKER**

- [quality.py:155](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/quality.py:155) — The full-data outlier mask leaks held-out information into fold training and removes accepted outliers from held-out scoring. It also changes modulo assignments for later captures. Build folds from all accepted captures in acquisition order, run outlier rejection independently on each training split, and score every accepted held-out capture in successful folds. Use the full-data solution only for fold sensitivity. Extend the injected-outlier test to verify fold membership and held-out scoring.

**SHOULD-FIX**

- [solver.py:161](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/solver.py:161) — §E4 specifies median board depth. The code uses Euclidean range to the board origin, which changes the rotation-column scaling for off-axis views. Use the median optical-axis `z` of the predicted board origins, reject nonpositive or nonfinite depth, and test with an off-axis board. The soft-L1 row weights, division by σ, and timing of the observability checks otherwise match §E4.
- [solver.py:117](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/solver.py:117) — `refine` returns `res.x` even when SciPy reports failure, so an unconverged fold can count as successful. Check convergence and finiteness, surface a documented `SolveError`, and test that a failed refinement increments `folds_failed`.
- [config.py:98](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/config.py:98) — Offline config accepts values that break the math. For example, `min_pair_rotation: 0` lets identical rotations divide by zero, and `sigma_px: 0` invalidates Jacobian scaling. Validate integer counts and finite, physically meaningful ranges at config load.
- [test_solver.py:100](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_solver.py:100) — The robot-systematic test asserts gate passage only, so disabling the injection still passes. Use matched clean and biased runs and assert the shift in the affected transform. A constant right-multiplied tool-pose bias shifts `Z` in eye-to-hand and `X` in eye-in-hand.
- [test_solver.py:124](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_solver.py:124) — The YAML test constructs a rotation but checks only translation, so a quaternion-order regression can pass. Parameterize both modes and assert both transforms’ parent, child, translation, and rotation. The current serializer uses the correct parents and `xyzw` order.

**NIT**

- [quality.py:186](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/quality.py:186) — Held-out RMS is correctly excluded from overall `passed`, but its informational row still compares against the training RMS ceiling and can print `PASS`. Show held-out RMS and max as measurements without a pass label.

The OpenCV mappings and adapter directions are correct in both modes, and both test oracles generate poses independently of the adapter. The 12-parameter tangent update and the initial fit → outlier removal → rotation recheck → single refit order match §E. Nonfinite gate values fail, and any counted fold failure fails the run while retaining the full-data transform. The new modules map to the reuse table and keep `core` ROS-free.

**Verification:** 50 tests passed. The read-only sandbox prevented the four filesystem-dependent round-trip and CLI tests and the full `colcon` run.

---

## Verification pass (after fixes)

### Prior findings

- **RESOLVED** — Folds use all accepted captures in acquisition order, reject outliers within each training split, and score the injected outlier when held out.
- **RESOLVED** — Observability uses median optical-axis depth, rejects invalid depth, and has an off-axis regression test.
- **RESOLVED** — Unsuccessful refinement or nonfinite parameters raise `SolveError`. The regression test confirms failed folds are counted.
- **PARTIAL** — Config validation catches the original zero-value cases, but scalar fields accept YAML booleans and the derived loss scale can overflow or underflow. Reject booleans and require `sigma_px * f_scale_sigmas` to be finite and positive.
- **RESOLVED** — The robot-bias test compares matched clean and biased runs and checks the affected transform in both modes.
- **RESOLVED** — The YAML test checks both transforms’ frame IDs, translation, and quaternion order in both modes.
- **RESOLVED** — Held-out RMS and max print as measurements without thresholds or pass labels.

### New findings

- **SHOULD-FIX** — `core/quality.py:119` — When `folds` exceeds the accepted capture count, empty folds train on all captures and count as successful. Reject that fold count with a clear verification failure while retaining the full-data transform.
- **SHOULD-FIX** — `cli/resolve.py:36` — Malformed YAML escapes as `yaml.YAMLError`, and falsey non-mappings such as `[]` silently select defaults. Catch `yaml.YAMLError` and require a mapping, treating only an empty document as defaults.
- **NIT** — `docs/design/01-architecture.md:275` — The interface summary omits the new empty comparison. Document the empty comparison, NaN limit, and unused `passed` flag for measurement rows.

60 tests passed. The read-only sandbox prevented the five filesystem-dependent tests and a full `colcon` run.

**Applied after the verification pass:** booleans and a degenerate loss scale rejected in SolverConfig; fewer captures than folds fails verification (transform kept); resolve rejects malformed or non-mapping config YAML; GateResult measurement rows documented in 01-architecture §5.
