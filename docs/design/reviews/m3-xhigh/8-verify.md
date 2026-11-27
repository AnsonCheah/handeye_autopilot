Reviewed `m3-node` at `5d4fa65` read-only. Twenty-three targeted Jazzy tests passed. The launch suite requires writable temporary storage.

### Pass 7 verification

| Finding | Status | Evidence at HEAD |
|---|---|---|
| Host-clock rollback during YAML writing | **ACCEPTED** | `30c4242` explicitly retains post-solve clock sampling. [node.py:405](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:405) |
| Best-effort `/tf` bypasses ownership checks | **RESOLVED** | Both ownership trackers subscribe best-effort. [node.py:140](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:140) · [publish_result.py:66](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/publish_result.py:66) |
| Conflicting static extrinsics | **RESOLVED** | Both ownership checks reject distinct values for the same static edge. [tf_tools.py:230](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/tf_tools.py:230) · [publish_result.py:83](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/publish_result.py:83) |
| One rail sample clears the fault latch | **RESOLVED** | Each known joint needs five samples spanning the window. The service keeps collecting until the deadline. [node.py:327](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:327) |
| Auto-finish can make verification impossible | **RESOLVED** | Target validation requires a nonempty holdout and enough training captures in every fold. [params.py:160](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/params.py:160) · [params.py:217](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/params.py:217) |
| Normal robot motion can abort a healthy goal | **RESOLVED** | The dynamic tracker stores edges only. After 20 seconds of seven-joint motion at 60 Hz, TF lag was 17 ms and the goal reached manual capture. [tf_tools.py:149](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/tf_tools.py:149) |
| `~/finish` can make verification impossible | **RESOLVED** | The service uses the same fold-aware minimum, 10 captures by default. [node.py:272](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:272) · [node.py:485](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:485) |
| Too few inliers returns `SOLVE_FAILED` | **RESOLVED** | `too_few_captures` maps to `NOT_ENOUGH_SAMPLES`. [node.py:505](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:505) |

### Remaining M3 findings

- **BLOCKER — Preflight can falsely abort at startup.** With both drivers already streaming on the same clock, a frame with 200 ms of camera latency predated the calibration node’s local TF history. Preflight kept retrying that stamp and returned `ABORTED / TF_UNAVAILABLE` even though newer frames had valid TF before the deadline. An immediate retry reached manual capture. Reproduced on Jazzy. [node.py:526](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:526) · [node.py:548](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:548)
- **SHOULD-FIX — A camera replacement can leave publication disabled until node restart.** The calibration node retains the previous driver’s static extrinsics and treats the replacement’s value as a live conflict, even with just one active publisher. [tf_tools.py:157](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/tf_tools.py:157) · [node.py:733](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:733)
- **SHOULD-FIX — The frozen interface documentation has two discrepancies.** The `~/finish` description and default config still give eight as the threshold. Action transforms are stamped after YAML writing, past the specified solve-completion time. [01-architecture.md:383](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:383) · [default.yaml:25](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/config/default.yaml:25) · [01-architecture.md:308](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:308) · [node.py:450](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:450)

### Notes

The `CameraInfo` snapshot-to-pin and TF drain callback windows remain. [node.py:524](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:524) · [publish_result.py:110](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/publish_result.py:110)

M3 merge: BLOCKED