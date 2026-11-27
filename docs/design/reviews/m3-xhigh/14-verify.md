Reviewed `m3-node` at `34225ec`. Eighteen Jazzy helper tests passed, and targeted action probes ran in-process. A fresh launch-suite run requires writable temporary storage.

### Pass 13 findings

- **RESOLVED — Short simulator rewind.** With 150 ms TF latency, the next goal reached manual capture and accepted its first capture after a 0.95 s rewind. The synthetic camera drops future TF, and preflight retries newer images. [synthetic_camera.py:154](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/synthetic_camera.py:154) · [node.py:576](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:576)
- **RESOLVED — Reset during the final preflight TF lookup.** The post-loop check produced `ABORTED / TF_UNAVAILABLE` with the clock-reset diagnostic. [node.py:589](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:589)
- **RESOLVED — Verification feedback and result stamp.** `PHASE_VERIFYING` was active at k-fold entry. Both action transforms use the timestamp taken after verification, as the schema now specifies. [quality.py:155](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/quality.py:155) · [node.py:514](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:514) · [Calibrate.action:37](/home/anson/workspaces/handeye_autopilot/handeye_autopilot_interfaces/action/Calibrate.action:37)

### Remaining M3 findings

- **BLOCKER — A queued image can falsely abort preflight after a camera resolution change.** New `CameraInfo` was already in place when the goal started. One queued old-resolution image caused `ABORTED / NO_CAMERA_INFO`, although a matching image arrived 150 ms later, within the preflight timeout. The next goal reached manual capture with the same configuration. [node.py:550](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:550) · [node.py:568](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:568)
- **SHOULD-FIX — Clock retries delay cancellation.** After a rewind and pause, a queued old-epoch image kept the goal active for 4.77 s after cancellation was accepted. The retry loop skips the cancellation check. [node.py:576](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:576)
- **SHOULD-FIX — The verification regression misses the original bug.** It checks that `SOLVING` precedes `VERIFYING`, which also held when `VERIFYING` was emitted after k-fold verification. [test_node_manual.py:1155](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_manual.py:1155)

**Notes.** Host-clock sampling during YAML writes and the documented `result.launch.py` exit status are **ACCEPTED**. [12-verify.md:15](/home/anson/workspaces/handeye_autopilot/docs/design/reviews/m3-xhigh/12-verify.md:15) · [result.launch.py:9](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/launch/result.launch.py:9)

M3 merge: BLOCKED