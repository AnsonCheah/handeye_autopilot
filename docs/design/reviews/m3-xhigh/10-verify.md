Reviewed `m3-node` at `d49c3c6`. Eighteen Jazzy helper tests passed. In-process probes exercised the action server, with YAML writes bypassed for the success-path probe. The full launch suite requires writable storage.

### Pass 9 verification

| Finding | Status | Evidence |
|---|---|---|
| Preflight aborts on TF latency within budget | **RESOLVED** | Each frame gets the full timeout. The 30 Hz camera and 400 ms TF delay probe reached manual capture. [node.py:568](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:568) |
| Recalibration requires a restart to publish | **PARTIAL · SHOULD-FIX** | Replacing a mount under the same parent works. Switching from eye-to-hand to eye-in-hand can still make the cached old parent veto publication after its broadcaster stops. [tf_tools.py:214](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/tf_tools.py:214) |
| Cancellation loses capture counts | **RESOLVED** | The abort path supplies counts, and verdict overrides preserve quality. An early waiter received `CANCELED` with counts matching feedback. [node.py:434](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:434) · [node.py:516](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/node.py:516) |
| `~/finish` threshold and action stamps | **PARTIAL · SHOULD-FIX** | The threshold is documented. Transform stamps still follow k-fold verification, which runs after the final fit. [01-architecture.md:383](/home/anson/workspaces/handeye_autopilot/docs/design/01-architecture.md:383) · [node.py:511](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:511) · [quality.py:162](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/quality.py:162) |

### Additional M3 findings

- **BLOCKER — Mixed outcomes wedge the action server.** On Jazzy, a successful run followed by a canceled run raised `ValueError` when the same log call switched severity. Cleanup skipped clearing the active slot, and the third goal was rejected as busy. [node.py:374](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:374)
- **BLOCKER — A normal simulator reset can break the next run.** With 150 ms robot TF latency, pre-reset messages in transit repopulated the cleared buffer. Jazzy rejected fresh TF as `TF_OLD_DATA`, and a new goal ended `ABORTED / TF_UNAVAILABLE`. [node.py:129](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:129) · [node.py:184](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/node.py:184)
- **SHOULD-FIX — A stopped camera can pass preflight.** A new goal reached manual capture using a cached image after the stream stopped. The clock check compares the stamp to its original arrival time. [node.py:537](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:537) · [node.py:599](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:599)

**Notes** — Host-clock sampling during YAML writing remains **ACCEPTED** per `30c4242`. The `CameraInfo` pin and TF drain callback windows remain. [node.py:555](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/ros/node.py:555) · [publish_result.py:110](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/ros/publish_result.py:110)

M3 merge: BLOCKED