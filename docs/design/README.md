# Design plan

Read in order:

1. [01-architecture.md](01-architecture.md): how the pieces fit, the reuse map, the robot hook (MoveIt `compute_ik` / MoveGroup), the one custom action, timing and TF rules.
2. [02-features.md](02-features.md): geometry, session flow, candidate planning, capture gates, solver, quality gates, verification, simulation.
3. [03-milestones.md](03-milestones.md): milestones M0–M6, acceptance criteria, release path to the ROS build farm.

`reviews/` holds the Codex review of each checkpoint. Each review was written against an **earlier draft**; the documents above already apply its findings, so section numbers in a review may not match.

Implementation rule: this is a clean-room project. Implementation works from these documents only.
