Reviewed `m3-node` at `41e8f9d`. All 14 Jazzy helper tests passed. Both manual calibration modes passed in-process with YAML writes bypassed. The full launch suite requires writable temporary storage.

### Pass 16 findings

- **RESOLVED — Stale `CameraInfo` after a TF wait.** With 350 ms TF latency, HEAD retried and pinned the new 320×240 metadata. `2bdca23` reproduced `ABORTED / NO_CAMERA_INFO`. [node.py:587](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:587)
- **RESOLVED — Image-first regression.** The test now sends a genuine new-resolution pair with metadata delayed by 150 ms and asserts that preflight pins it. It passes on HEAD and fails with `accef57`’s preflight. [test_node_manual.py:1188](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_manual.py:1188)
- **RESOLVED — Queued old-resolution regression.** Injection now waits for preflight feedback. The test passes on HEAD and fails with `34225ec`’s preflight. [test_node_manual.py:1169](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_manual.py:1169)

### Remaining M3 findings

No blockers found.

- **SHOULD-FIX — Add coverage for a metadata change during the TF wait.** Both committed resolution tests still pass with `2bdca23`’s preflight. The targeted probe catches the regression. [test_node_manual.py:1169](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_manual.py:1169)
- **SHOULD-FIX — Preserve capture counts on output errors.** On a read-only output volume, feedback reported 10 accepted captures and the `ABORTED / INTERNAL` result reported zero. [node.py:365](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:365)

**Notes.** Host-clock sampling during YAML writes and the documented launch exit status remain **ACCEPTED**. A narrow callback window remains between the metadata recheck and pin. [16-verify.md:15](/home/anson/workspaces/handeye_autopilot/docs/design/reviews/m3-xhigh/16-verify.md:15) · [node.py:589](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:589)

M3 merge: CLEAR