Reviewed `m3-node` at `492883f`. Fourteen Jazzy helper tests and the startup regression test passed. The full launch suite requires writable temporary storage.

### Pass 8 verification

| Finding | Status | Evidence at HEAD |
|---|---|---|
| Startup frame predates TF history | **RESOLVED** | Preflight retries with newer images. The startup regression test passed. [node.py:553](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:553) · [test_node_manual.py:904](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_node_manual.py:904) |
| Camera replacement leaves a stale extrinsics conflict | **RESOLVED** | A replacement value on the same edge is usable. Conflict detection now runs in the fresh result publisher. [node.py:131](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:131) · [publish_result.py:68](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/publish_result.py:68) |
| `~/finish` threshold and result stamps | **PARTIAL** | The fold-aware threshold is documented. Stamps now precede YAML writing but still follow k-fold verification, which runs after the final fit. [01-architecture.md:383](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:383) · [default.yaml:25](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/config/default.yaml:25) · [node.py:498](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:498) · [quality.py:148](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/quality.py:148) |

### Remaining M3 findings

- **BLOCKER — Preflight aborts with TF latency inside the configured timeout.** Each retry drops the image after 200 ms and picks the newest one. On Jazzy, a 30 Hz camera and 400 ms robot TF delay on the same clock produced `ABORTED / TF_UNAVAILABLE`, even though an exact-stamp lookup succeeded in 406 ms within the configured 500 ms. [node.py:553](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:553) · [default.yaml:30](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/config/default.yaml:30)
- **SHOULD-FIX — Recalibration can require a node restart to publish.** After the operator stops the previous result broadcaster, the calibration node’s cached static mount can still make it write `publish.ok: false`. A fresh TF buffer permits publication. [node.py:743](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:743) · [tf_tools.py:217](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/tf_tools.py:217)
- **SHOULD-FIX — Cancellation loses capture counts.** A Jazzy run reported one rejected capture in feedback and zero in the canceled result. [01-architecture.md:316](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:316) · [node.py:503](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:503) · [node.py:779](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:779)

### Notes

Host-clock sampling during YAML writing remains **ACCEPTED** per `30c4242`. The `CameraInfo` snapshot-to-pin and TF drain callback windows remain. [node.py:402](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:402) · [node.py:527](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:527) · [publish_result.py:110](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/publish_result.py:110)

M3 merge: BLOCKED