# M3 Codex reviews (`xhigh`)

One full review (`1-review`) and 16 verification passes (`2-` … `17-verify`),
each with its prompt. Pass 17: **"M3 merge: CLEAR"**; its two should-fixes
(capture counts on internal errors, a test for a CameraInfo change during the
preflight TF wait) are fixed in the merge commit.

Recurring themes, all fixed with tests: cancel/finalization races and rclpy's
empty default result; clock resets (jump callback, in-flight TF after a reset,
host clock steps); CameraInfo pinning around camera changes; TF ownership at
publish time (live re-check, `.publishable` marker only for a SUCCEEDED run
whose gates passed, moving or conflicting subtree links); profile validation
(types, fold-aware minimum); fault-clear evidence; test harness realism
(frame–pose pairing, a synthetic camera that drops stale frames).

Accepted (documented): a host clock step while the result YAML is written
(after the solve; cannot change the result); `result.launch.py` exits 0 on a
refusal (use `ros2 run handeye_autopilot publish_result` for automation);
fast-sim clock tolerance (01-architecture §8); narrow callback-order windows
noted in the later passes.
