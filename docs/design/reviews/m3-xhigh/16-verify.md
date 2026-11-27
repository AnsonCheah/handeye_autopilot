Reviewed `m3-node` at `2bdca23`. Fourteen Jazzy helper tests passed, and targeted action probes ran in-process. The k-fold probe bypassed result writes. The full launch suite requires writable temporary storage.

### Pass 15 findings

- **RESOLVED — New-resolution image arrives first.** With matching `CameraInfo` arriving 150 ms later, HEAD pinned the new metadata and reached manual capture. `accef57` aborted. [node.py:569](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:569)
- **PARTIAL — Queued old-resolution regression.** Pausing the stream exposed the original bug on `34225ec`. The same test passed on that revision when execution started 250 ms after goal acceptance, because preflight counted the injected frame as already cached. [test_node_manual.py:1175](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_manual.py:1175) · [node.py:541](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:541)

### Remaining M3 findings

- **BLOCKER — Preflight can pin stale CameraInfo after waiting for TF.** With 350 ms robot TF latency, a valid new-resolution pair arrived 155 ms into the old frame’s lookup. Preflight pinned the old metadata when that lookup succeeded. The goal ended `ABORTED / NO_CAMERA_INFO`. The next goal reached manual capture on the same stream. [node.py:587](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:587) · [node.py:605](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:605)
- **SHOULD-FIX — The new image-first regression reuses the old CameraInfo.** It sends an 8×8 image, re-sends the cached metadata, and resumes the original stream. It passed on the unfixed parent. [test_node_manual.py:1193](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_manual.py:1193)

### Notes

Host-clock sampling during YAML writes and the documented launch exit status remain **ACCEPTED**. [15-verify.md:14](/home/anson/workspaces/handeye_autopilot/docs/design/reviews/m3-xhigh/15-verify.md:14)

M3 merge: BLOCKED