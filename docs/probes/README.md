# Probe results

Raw output of the throwaway probes in `tools/probes/`, committed so design
decisions point at observed behaviour (03-milestones, "probe ahead").

## M2 MoveIt probe (`m2_moveit_<distro>.yaml`)

Panda mock-hardware demo (`moveit_resources_panda_moveit_config demo.launch.py`)
driven through pymoveit2 with the `MoveGroup` action.

| Distro | MoveIt | pymoveit2 | Result |
|---|---|---|---|
| Jazzy | 2.12.4 | 5.1.0 | [m2_moveit_jazzy.yaml](m2_moveit_jazzy.yaml) |
| Lyrical | | | pending (`gh workflow run probe.yml` after merge) |
| Rolling | | | pending |

`status` is the action's `GoalStatus` (4 SUCCEEDED, 5 CANCELED, 6 ABORTED);
`error_code` is `MoveItErrorCodes.val`.

### Jazzy findings

1. **Success:** SUCCEEDED / `SUCCESS` (1), end joints at the goal.
2. **Planning failure** (goal in self-collision): ABORTED / `FAILURE` (99999),
   the generic code, not `GOAL_IN_COLLISION`; the robot did not move. The error
   code cannot tell a planning failure from an execution failure. The
   result's `planned_trajectory` is empty only when planning failed (0 points
   here, 30+ for every planned motion), while `executed_trajectory` is empty
   even after successful motions. So `not_started` = aborted + no planned
   trajectory + joints unchanged (01-architecture §3.2).
3. **Out-of-limit joint goal** (joint 4 = +1.0, limit 0.0873): SUCCEEDED /
   `SUCCESS`, but the arm stopped at the soft limit 0.0175. **Success does not
   mean the goal was reached**: compare measured end joints with the goal.
4. **Cancel mid-motion** (sent after the joints moved > 0.02 rad):
   CANCELED / `PREEMPTED` (−7) in about 0.04 s; on mock hardware the arm
   stops at once. While `MoveGroup` is planning, pymoveit2 already reports
   `EXECUTING`, so "executing" is not proof of motion.
5. **`move_group` ignores the request's start state** during execution and
   uses the current monitored state. Without `/joint_states` (the
   `joint_state_broadcaster` / `joint_trajectory_controller` packages are not
   dependencies of the Panda config) every goal aborts with
   `START_STATE_IN_COLLISION` (−10).
6. **`compute_ik` timeout:** whole seconds honoured (unreachable pose: 1.00 s,
   `NO_IK_SOLUTION` −31); a nanosec-only 0.5 s timeout is dropped (0.05 s).
   Confirms 01-architecture §3.1.
7. **Cartesian moves** (pymoveit2 `move_to_pose(cartesian=True)`, fraction
   threshold 1.0) run as `compute_cartesian_path` + the `ExecuteTrajectory`
   action. A reachable 5 cm shift: SUCCEEDED / `SUCCESS`. An impossible 3 m
   shift: the planner covered 13 % of the path and pymoveit2 refused it
   **before sending anything** (`submitted: false`, robot unmoved). MoveIt
   logs jump threshold 0 (jump check disabled) unless the caller sets one.
8. **Offset TCP (TF-only tool frame) on a Cartesian path:** a 20° rotation
   about a TCP 300 mm along `panda_link8` z, planned with
   `compute_cartesian_path` for `panda_link8`. End pose only: the TCP strays
   **4.52 mm** mid-path (the planner interpolates the link, cutting a chord).
   Dense waypoints (1° apart) converted from the TCP path: **0.014 mm**. This
   is why checked moves carry converted waypoints (01-architecture §3.2).
