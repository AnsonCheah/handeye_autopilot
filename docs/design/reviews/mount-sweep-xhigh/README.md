# Mount sweep Codex reviews (`xhigh`)

One full review (`1-review`) and verification passes (`2-` … `n-verify`), each
with its prompt. The change: a mount sweep (core and robot level), the demo
staying up with RViz, and `INVALID_MOTION_PLAN` as a safe skip.

Fixed, with tests:
- **the skip's evidence** (most of the passes): a reply code plus unchanged
  joints at the end is not enough, since the robot could have moved and come
  back. The node now watches `/joint_states` from the goal until the reply's
  fresh joints: partial messages count, messages stamped before the start
  don't, full samples need strictly increasing stamps and gaps within 0.25 s
  in receive time and in the stamps' clock, the end included (01-architecture §3.2);
- **`lever_max` from the board centre** in eye-to-hand (02-features §B); the
  eye-to-hand robot sweep was re-run;
- **`setup` verdicts need evidence**: MoveIt's collision code plus the board or
  camera among the contacts of the state it rejected;
- sweep Ctrl-C and timeouts, the runner's cancel fallback, the core sweep also
  checking the board transform.

Accepted (documented in code or docs):
- motion between joint samples within 0.25 s, and publishers forging new
  stamped joint states (§3.2);
- the contact probe runs right after MoveIt refuses the start state; nothing
  was executed, so the robot is still in that state (`run_demo._log_contacts`);
- the runner's `~/cancel` fallback cancels whichever run is active; the demo's
  node serves only the runner (`run_demo._cancel_any`);
- in the round-trip integration test, the fake's 0.3 s dwell relies on normal
  executor scheduling; the direct `StillWatch` tests cover the rules
  deterministically;
- the FK launch starts before the sweep's worker threads exist; SIGINT is held
  off around `Popen` and unblocked in the child.
