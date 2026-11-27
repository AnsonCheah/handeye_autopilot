Reviewed `m3-node` at `30c4242` read-only. Sixteen targeted Jazzy tests passed, along with the in-process rail regression. The launch suite requires writable temporary storage and was skipped.

### Pass 6 findings

| Finding | Status | Evidence at HEAD |
|---|---|---|
| Wall-clock rollback during YAML writing | **ACCEPTED** | HEAD deliberately accepts the post-solve sampling gap. The final verdict compares wall time with the highest sampled time. [node.py:395](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:395) |
| Best-effort `/tf` bypasses ownership checks | **RESOLVED** | Both ownership trackers now subscribe best-effort. [node.py:136](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:136) · [publish_result.py:65](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/publish_result.py:65) |
| Conflicting static extrinsics | **RESOLVED** | Both checks refuse differing values observed on the same static camera-chain edge. [tf_tools.py:223](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/tf_tools.py:223) · [publish_result.py:82](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/publish_result.py:82) |
| One rail sample clears the fault latch | **RESOLVED** | Each recently reported joint needs its own sample count and time span. [node.py:325](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:325) |
| Auto-finish can make verification impossible | **PARTIAL** | The reported eight-capture, five-fold case is rejected. A target smaller than the fold count can still pass validation. [params.py:211](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/params.py:211) · [quality.py:119](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/quality.py:119) |

### Remaining findings

- **BLOCKER — Normal robot motion can abort a healthy goal.** The dynamic `/tf` tracker retains distinct poses indefinitely and scans the growing history under a lock. In a live Jazzy probe, 20 seconds of seven-joint motion at 60 Hz left TF 1.75 seconds behind. A fresh goal then ended `ABORTED / TF_UNAVAILABLE` despite the camera and robot sharing a clock. [tf_tools.py:144](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/tf_tools.py:144) · [node.py:136](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:136) · [node.py:542](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/node.py:542)
- **SHOULD-FIX — Verification can still be impossible at finish.** A target of 12 with 15 folds is accepted and guarantees all folds fail. With defaults, `~/finish` also accepts eight or nine captures even though ten are needed to give every fold eight training captures. [params.py:211](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/params.py:211) · [node.py:266](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:266) · [quality.py:119](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/quality.py:119)
- **SHOULD-FIX — Too few inliers returns the generic solve error.** When outlier removal leaves too few samples, the action returns `SOLVE_FAILED` where the design specifies `NOT_ENOUGH_SAMPLES`. [solver.py:211](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/solver.py:211) · [node.py:501](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:501) · [02-features.md:345](/home/anson/workspaces/handeye_autopilot/docs/design/02-features.md:345)

### Notes

The `CameraInfo` snapshot-to-pin and TF drain callback windows remain. With an injected latch, a healthy slower rail can also be refused as soon as the faster arm fills the combined window. [node.py:518](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:518) · [publish_result.py:110](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/publish_result.py:110) · [node.py:329](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:329)

M3 merge: BLOCKED