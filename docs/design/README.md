# Design plan

Read in order:

1. [01-architecture.md](01-architecture.md): how the pieces fit, the reuse map, the robot hook (MoveIt `compute_ik` / MoveGroup), the one custom action, timing and TF rules.
2. [02-features.md](02-features.md): geometry, session flow, candidate planning, capture gates, solver, quality gates, verification, simulation.
3. [03-milestones.md](03-milestones.md): milestones M0–M6, acceptance criteria, release path to the ROS build farm.

`reviews/` holds the Codex review of each checkpoint (`codex-review-01..03.md`), a final review of the whole plan (`codex-review-final.md`) the pre-implementation review (`codex-review-pre-impl.md`, applied as rev 3.1) and its verification (`codex-review-rev3.1.md`, applied). Each review was written against an **earlier draft**; the documents above (rev 3.1) already apply its findings, so line and section numbers in a review may not match. Findings deliberately declined are marked as such in the documents (marker masking, exposure, the UR demo).

Implementation rule: this is a clean-room project. Implementation works from these documents only.
