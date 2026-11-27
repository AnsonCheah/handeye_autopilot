Reviewed `m3-node` at `accef57`. Fourteen Jazzy helper tests passed. Action probes ran in-process, with result writes bypassed for the k-fold test. The full launch suite requires writable temporary storage.

### Pass 14 findings

- **RESOLVED — Queued old-resolution image.** With the new `CameraInfo` already cached, preflight skipped the old image and reached manual capture when a matching image arrived 150 ms later. [node.py:575](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:575)
- **RESOLVED — Clock retries delay cancellation.** The retry loop checks cancellation on every pass. The probe returned `CANCELED` in 50 ms. [node.py:571](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:571)
- **RESOLVED — Verification regression misses the original bug.** The test now asserts `PHASE_VERIFYING` at k-fold entry and passed during a calibration. [test_node_manual.py:1155](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_manual.py:1155)

### Remaining M3 findings

- **BLOCKER — A new-resolution image can pin the old `CameraInfo`.** After a resolution change, the new image arrived 150 ms before its matching, identically stamped `CameraInfo`. Preflight pinned the cached metadata before checking dimensions and kept using it as matching pairs arrived. The goal ended `ABORTED / NO_CAMERA_INFO`. The next goal reached manual capture on the same stream. [node.py:545](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:545) · [node.py:563](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:563) · [node.py:583](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:583)
- **SHOULD-FIX — The queued-image regression also passed against `HEAD^`.** The live camera keeps streaming, so the test can reach manual capture without preflight ever inspecting the old frame. [test_node_manual.py:1169](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_manual.py:1169)

**Notes.** Host-clock sampling during YAML writes and the documented launch exit status remain **ACCEPTED**. [14-verify.md:15](/home/anson/workspaces/handeye_autopilot/docs/design/reviews/m3-xhigh/14-verify.md:15)

M3 merge: BLOCKED