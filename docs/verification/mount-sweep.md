# Mount sweep: very different board and camera mounts

Does the calibration recover the mount when the board (eye-to-hand) or the
camera (eye-in-hand) is bolted on very differently from the demo? Each case
moves that mount by up to **±200 mm and ±90° on each tool axis**
(01-architecture §10).

## How to run it

Core level (no ROS, perfect motion, ~20 s a case). CI runs three cases; this
runs all 15 offsets in both modes:

```bash
cd ~/workspaces/handeye_autopilot/handeye_autopilot
HANDEYE_FULL_SWEEP=1 python3 -m pytest -s test/test_mount_sweep.py
```

Real MoveIt on Panda and UR5e (mock hardware), both modes, the offsets in
`handeye_autopilot_demos/config/mount_sweep.yaml` (14 named + 4 random). 72
runs, about 2 hours at 3–4 at a time; more than 2 at a time can starve a
small machine (see the re-runs below):

```bash
ros2 run handeye_autopilot_demos sweep_mounts --robots panda ur --jobs 4
ros2 run handeye_autopilot_demos sweep_mounts --robots ur --modes eye_in_hand --cases rx+90 all+ --jobs 2
```

Each run gets its own ROS domain ID, so parallel runs never see each other's
topics ([ROS 2 domain ID](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html)).
Per case it writes the generated config, the launch log and the calibration
result under `~/.ros/handeye_autopilot/mount_sweep/<time>/`, plus
`results.yaml` and `results.md`.

How a case is built (`handeye_autopilot_demos/mounts.py`):
- the robot-side mount gets the offset: translation added, rotation about the
  tool axes;
- the fixed object (camera tripod or board stand) is moved so the camera sees
  the board at the seed exactly as in the demo. Choosing a good seed is the
  operator's job, and the sweep tests the calibration, not seed-finding;
- `lever_max` is set to the moved lever arm's largest component + 20 mm,
  which is what an operator would measure and enter. The lever arm runs from
  the tool to the board centre (eye-to-hand) or to the camera (eye-in-hand),
  the points the bootstrap bound uses (02-features §B);
- the pass criteria are the demo's: every quality gate, and within 3 mm / 0.5°
  of the true mount.

## Results (Jazzy, 2026-11-28)

**Core level:** all 30 cases (15 offsets × 2 modes) pass every gate, and both
the camera and the board transform match the truth. Worst: camera 1.35 mm /
0.14° (eye-in-hand tx+200), board 0.53 mm / 0.14°. The core simulation adds
robot pose noise, so its errors are larger than the robot runs'.

**Robots:** 66 of 72 pass, every one within **0.11 mm and 0.014°** of the true
mount. The other 6 are `setup`: MoveIt reports `START_STATE_IN_COLLISION`, and
its contact list for that state names the board or the camera against a robot
link, so the robot can't start with that arrangement. In four of them the
mounted object (board or camera on the tool) hits the arm. In two eye-in-hand
cases (Panda ry−90, UR rx−90) the moved fixed board lands inside the arm: the
sweep places it to keep the seed view, and with that camera mount the view is
only possible from inside the robot. The runner logs those contacts; a seed
failure without the board or camera among the failing state's contacts counts
as a failure. No run returned
a wrong mount.

Eight runs failed and passed when re-run. Seven are marked in the table; the
eighth (UR eye-to-hand rz−90, `INVALID_MOTION_PLAN`) was replaced by the
eye-to-hand re-sweep below.
- **5 × `INVALID_MOTION_PLAN`** in the first full sweep. MoveIt's own planner
  produced a path that failed its own collision check (`ValidateSolution`).
  The robot didn't move, but the node stopped the run. It is now a safe skip
  (a spare pose takes its place), but only when the joints were watched the
  whole time the goal was active and never left the start (01-architecture
  §3.2). A plan that moved the robot, even one that came back exactly, still
  ends the run (tests). One re-run (UR eye-in-hand rz−90) hit it, skipped the
  pose and passed.
- **2 × `CONTROL_FAILED`**: "couldn't receive full current joint state within
  1s" with four robot stacks on one machine.
- **1 × image late**: a camera image arrived 2.6 s after the pose it shows,
  over the demo's 2 s clock tolerance, while other tests shared the machine.
The last three are load from running so much at once, not the calibration.

The eye-to-hand rows are from a second sweep, re-run after a review found the
first one sized `lever_max` from the board's corner instead of its centre. All
eye-to-hand verdicts were the same as before, apart from the one late image
above.

| Robot | Mode | Case | Offset (mm; deg) | lever_max | Verdict | Error vs truth | Captures | Minutes | Notes |
|---|---|---|---|---|---|---|---|---|---|
| panda | eye_to_hand | tx+200 | 200, 0, 0; 0, 0, 0 | 0.31 | passed | 0.05 mm, 0.004° | 60 / 0 | 4.0 |  |
| panda | eye_to_hand | tx-200 | -200, 0, 0; 0, 0, 0 | 0.16 | passed | 0.05 mm, 0.005° | 60 / 1 | 3.8 |  |
| panda | eye_to_hand | ty+200 | 0, 200, 0; 0, 0, 0 | 0.22 | passed | 0.03 mm, 0.002° | 60 / 0 | 3.9 |  |
| panda | eye_to_hand | ty-200 | 0, -200, 0; 0, 0, 0 | 0.22 | passed | 0.05 mm, 0.002° | 60 / 1 | 3.9 |  |
| panda | eye_to_hand | tz+200 | 0, 0, 200; 0, 0, 0 | 0.36 | passed | 0.07 mm, 0.012° | 60 / 2 | 4.1 |  |
| panda | eye_to_hand | tz-200 | 0, 0, -200; 0, 0, 0 | 0.11 | passed | 0.04 mm, 0.006° | 60 / 0 | 3.7 |  |
| panda | eye_to_hand | rx+90 | 0, 0, 0; 90, 0, 0 | 0.11 | passed | 0.09 mm, 0.007° | 60 / 1 | 3.8 |  |
| panda | eye_to_hand | rx-90 | 0, 0, 0; -90, 0, 0 | 0.20 | passed | 0.02 mm, 0.002° | 60 / 1 | 4.1 |  |
| panda | eye_to_hand | ry+90 | 0, 0, 0; 0, 90, 0 | 0.19 | passed | 0.04 mm, 0.003° | 60 / 0 | 3.8 |  |
| panda | eye_to_hand | ry-90 | 0, 0, 0; 0, -90, 0 | 0.09 | passed | 0.06 mm, 0.004° | 60 / 1 | 3.8 | fixed camera 5 cm below the table top |
| panda | eye_to_hand | rz+90 | 0, 0, 0; 0, 0, 90 | 0.22 | passed | 0.05 mm, 0.004° | 60 / 1 | 4.1 |  |
| panda | eye_to_hand | rz-90 | 0, 0, 0; 0, 0, -90 | 0.16 | passed | 0.09 mm, 0.007° | 60 / 1 | 3.9 |  |
| panda | eye_to_hand | all+ | 200, 200, 200; 90, 90, 90 | 0.39 | passed | 0.09 mm, 0.008° | 96 / 1 | 6.6 |  |
| panda | eye_to_hand | all- | -200, -200, -200; -90, -90, -90 | 0.15 | setup |  |  | 0.2 | the mount collides before calibrating (START_STATE_IN_COLLISION: calibration_board <-> panda_link5; calibration_board <-> panda_link6) |
| panda | eye_to_hand | random1 | -149, -0, 41; -85, -63, 77 | 0.18 | passed | 0.04 mm, 0.009° | 60 / 0 | 3.9 |  |
| panda | eye_to_hand | random2 | -172, -148, 179; 22, -24, 2 | 0.29 | passed | 0.02 mm, 0.006° | 60 / 0 | 3.8 |  |
| panda | eye_to_hand | random3 | 65, -90, -145; 52, 31, 2 | 0.16 | passed | 0.05 mm, 0.003° | 60 / 2 | 3.9 |  |
| panda | eye_to_hand | random4 | 127, 20, 192; -53, 10, -3 | 0.40 | passed | 0.03 mm, 0.003° | 60 / 0 | 4.1 |  |
| panda | eye_in_hand | tx+200 | 200, 0, 0; 0, 0, 0 | 0.29 | passed | 0.09 mm, 0.005° | 96 / 0 | 6.1 | first run failed: INVALID_MOTION_PLAN, stopped the run before the fix; passed on re-run |
| panda | eye_in_hand | tx-200 | -200, 0, 0; 0, 0, 0 | 0.15 | setup |  |  | 0.3 | the mount collides before calibrating (START_STATE_IN_COLLISION: camera <-> panda_link5) |
| panda | eye_in_hand | ty+200 | 0, 200, 0; 0, 0, 0 | 0.22 | passed | 0.10 mm, 0.009° | 96 / 1 | 5.8 |  |
| panda | eye_in_hand | ty-200 | 0, -200, 0; 0, 0, 0 | 0.22 | passed | 0.06 mm, 0.005° | 96 / 0 | 5.8 | first run failed: INVALID_MOTION_PLAN, stopped the run before the fix; passed on re-run |
| panda | eye_in_hand | tz+200 | 0, 0, 200; 0, 0, 0 | 0.24 | passed | 0.04 mm, 0.004° | 96 / 0 | 5.6 | fixed board 7 cm below the table top |
| panda | eye_in_hand | tz-200 | 0, 0, -200; 0, 0, 0 | 0.20 | passed | 0.10 mm, 0.010° | 96 / 0 | 6.2 |  |
| panda | eye_in_hand | rx+90 | 0, 0, 0; 90, 0, 0 | 0.09 | passed | 0.11 mm, 0.010° | 96 / 0 | 5.9 |  |
| panda | eye_in_hand | rx-90 | 0, 0, 0; -90, 0, 0 | 0.09 | passed | 0.08 mm, 0.006° | 96 / 1 | 5.9 |  |
| panda | eye_in_hand | ry+90 | 0, 0, 0; 0, 90, 0 | 0.09 | passed | 0.10 mm, 0.007° | 96 / 1 | 5.9 |  |
| panda | eye_in_hand | ry-90 | 0, 0, 0; 0, -90, 0 | 0.09 | setup |  |  | 0.3 | the mount collides before calibrating (START_STATE_IN_COLLISION: calibration_board <-> panda_link2; calibration_board <-> panda_link3; calibration_board <-> panda_link4) |
| panda | eye_in_hand | rz+90 | 0, 0, 0; 0, 0, 90 | 0.09 | passed | 0.10 mm, 0.006° | 96 / 1 | 5.8 |  |
| panda | eye_in_hand | rz-90 | 0, 0, 0; 0, 0, -90 | 0.09 | passed | 0.04 mm, 0.006° | 96 / 0 | 5.7 |  |
| panda | eye_in_hand | all+ | 200, 200, 200; 90, 90, 90 | 0.29 | passed | 0.08 mm, 0.011° | 96 / 0 | 5.9 |  |
| panda | eye_in_hand | all- | -200, -200, -200; -90, -90, -90 | 0.22 | passed | 0.08 mm, 0.005° | 96 / 0 | 6.5 | first run failed: INVALID_MOTION_PLAN, stopped the run before the fix; passed on re-run |
| panda | eye_in_hand | random1 | -149, -0, 41; -85, -63, 77 | 0.10 | passed | 0.06 mm, 0.014° | 96 / 0 | 5.7 |  |
| panda | eye_in_hand | random2 | -172, -148, 179; 22, -24, 2 | 0.22 | passed | 0.08 mm, 0.009° | 96 / 2 | 5.8 |  |
| panda | eye_in_hand | random3 | 65, -90, -145; 52, 31, 2 | 0.15 | passed | 0.08 mm, 0.006° | 96 / 1 | 6.2 | first run failed: CONTROL_FAILED (joint states late under load); passed on re-run |
| panda | eye_in_hand | random4 | 127, 20, 192; -53, 10, -3 | 0.23 | passed | 0.04 mm, 0.006° | 96 / 0 | 6.0 | first run failed: CONTROL_FAILED (joint states late under load); passed on re-run |
| ur | eye_to_hand | tx+200 | 200, 0, 0; 0, 0, 0 | 0.22 | passed | 0.05 mm, 0.005° | 60 / 1 | 4.1 |  |
| ur | eye_to_hand | tx-200 | -200, 0, 0; 0, 0, 0 | 0.22 | passed | 0.10 mm, 0.008° | 60 / 0 | 3.8 |  |
| ur | eye_to_hand | ty+200 | 0, 200, 0; 0, 0, 0 | 0.11 | passed | 0.06 mm, 0.006° | 60 / 1 | 4.3 |  |
| ur | eye_to_hand | ty-200 | 0, -200, 0; 0, 0, 0 | 0.33 | passed | 0.05 mm, 0.008° | 60 / 1 | 4.1 | first run failed: image 2.6 s late under load, over the 2 s clock tolerance; passed on re-run |
| ur | eye_to_hand | tz+200 | 0, 0, 200; 0, 0, 0 | 0.31 | passed | 0.05 mm, 0.004° | 60 / 1 | 4.1 |  |
| ur | eye_to_hand | tz-200 | 0, 0, -200; 0, 0, 0 | 0.13 | passed | 0.02 mm, 0.006° | 60 / 1 | 4.4 |  |
| ur | eye_to_hand | rx+90 | 0, 0, 0; 90, 0, 0 | 0.20 | passed | 0.04 mm, 0.004° | 60 / 1 | 3.9 |  |
| ur | eye_to_hand | rx-90 | 0, 0, 0; -90, 0, 0 | 0.05 | passed | 0.04 mm, 0.005° | 60 / 0 | 4.0 | fixed camera 2 cm below the table top |
| ur | eye_to_hand | ry+90 | 0, 0, 0; 0, 90, 0 | 0.20 | passed | 0.08 mm, 0.007° | 60 / 0 | 3.9 |  |
| ur | eye_to_hand | ry-90 | 0, 0, 0; 0, -90, 0 | 0.13 | passed | 0.09 mm, 0.008° | 60 / 0 | 4.0 |  |
| ur | eye_to_hand | rz+90 | 0, 0, 0; 0, 0, 90 | 0.23 | passed | 0.03 mm, 0.004° | 60 / 0 | 3.9 |  |
| ur | eye_to_hand | rz-90 | 0, 0, 0; 0, 0, -90 | 0.12 | passed | 0.08 mm, 0.006° | 60 / 2 | 4.1 |  |
| ur | eye_to_hand | all+ | 200, 200, 200; 90, 90, 90 | 0.40 | passed | 0.10 mm, 0.007° | 60 / 2 | 4.5 |  |
| ur | eye_to_hand | all- | -200, -200, -200; -90, -90, -90 | 0.33 | passed | 0.11 mm, 0.009° | 60 / 0 | 3.9 |  |
| ur | eye_to_hand | random1 | -149, -0, 41; -85, -63, 77 | 0.16 | passed | 0.03 mm, 0.011° | 60 / 1 | 3.9 |  |
| ur | eye_to_hand | random2 | -172, -148, 179; 22, -24, 2 | 0.31 | passed | 0.07 mm, 0.007° | 60 / 0 | 3.8 |  |
| ur | eye_to_hand | random3 | 65, -90, -145; 52, 31, 2 | 0.28 | passed | 0.05 mm, 0.006° | 60 / 1 | 4.2 |  |
| ur | eye_to_hand | random4 | 127, 20, 192; -53, 10, -3 | 0.29 | passed | 0.04 mm, 0.005° | 60 / 0 | 3.9 |  |
| ur | eye_in_hand | tx+200 | 200, 0, 0; 0, 0, 0 | 0.22 | passed | 0.03 mm, 0.008° | 96 / 1 | 7.2 |  |
| ur | eye_in_hand | tx-200 | -200, 0, 0; 0, 0, 0 | 0.22 | passed | 0.01 mm, 0.001° | 96 / 0 | 6.9 |  |
| ur | eye_in_hand | ty+200 | 0, 200, 0; 0, 0, 0 | 0.29 | passed | 0.09 mm, 0.009° | 92 / 0 | 6.6 | first run failed: INVALID_MOTION_PLAN, stopped the run before the fix; passed on re-run |
| ur | eye_in_hand | ty-200 | 0, -200, 0; 0, 0, 0 | 0.15 | passed | 0.03 mm, 0.001° | 96 / 2 | 6.4 |  |
| ur | eye_in_hand | tz+200 | 0, 0, 200; 0, 0, 0 | 0.25 | passed | 0.02 mm, 0.004° | 96 / 0 | 6.0 | fixed board 9 cm below the table top |
| ur | eye_in_hand | tz-200 | 0, 0, -200; 0, 0, 0 | 0.19 | setup |  |  | 0.4 | the mount collides before calibrating (START_STATE_IN_COLLISION: camera <-> forearm_link; camera <-> wrist_1_link) |
| ur | eye_in_hand | rx+90 | 0, 0, 0; 90, 0, 0 | 0.09 | passed | 0.07 mm, 0.007° | 96 / 1 | 6.7 |  |
| ur | eye_in_hand | rx-90 | 0, 0, 0; -90, 0, 0 | 0.09 | setup |  |  | 0.4 | the mount collides before calibrating (START_STATE_IN_COLLISION: calibration_board <-> upper_arm_link) |
| ur | eye_in_hand | ry+90 | 0, 0, 0; 0, 90, 0 | 0.09 | passed | 0.04 mm, 0.005° | 96 / 2 | 6.5 |  |
| ur | eye_in_hand | ry-90 | 0, 0, 0; 0, -90, 0 | 0.09 | passed | 0.06 mm, 0.011° | 96 / 0 | 6.6 |  |
| ur | eye_in_hand | rz+90 | 0, 0, 0; 0, 0, 90 | 0.09 | passed | 0.07 mm, 0.011° | 96 / 0 | 7.1 |  |
| ur | eye_in_hand | rz-90 | 0, 0, 0; 0, 0, -90 | 0.09 | passed | 0.04 mm, 0.011° | 96 / 2 | 6.3 |  |
| ur | eye_in_hand | all+ | 200, 200, 200; 90, 90, 90 | 0.29 | passed | 0.09 mm, 0.009° | 96 / 2 | 6.8 |  |
| ur | eye_in_hand | all- | -200, -200, -200; -90, -90, -90 | 0.22 | passed | 0.02 mm, 0.003° | 96 / 1 | 6.8 |  |
| ur | eye_in_hand | random1 | -149, -0, 41; -85, -63, 77 | 0.17 | passed | 0.06 mm, 0.010° | 96 / 1 | 6.0 |  |
| ur | eye_in_hand | random2 | -172, -148, 179; 22, -24, 2 | 0.23 | passed | 0.03 mm, 0.004° | 96 / 1 | 5.9 |  |
| ur | eye_in_hand | random3 | 65, -90, -145; 52, 31, 2 | 0.14 | setup |  |  | 0.2 | the mount collides before calibrating (START_STATE_IN_COLLISION: camera <-> wrist_2_link) |
| ur | eye_in_hand | random4 | 127, 20, 192; -53, 10, -3 | 0.24 | passed | 0.07 mm, 0.011° | 96 / 1 | 6.0 |  |

Notes:
- **Offset** is in the tool frame: mm on x, y, z, then degrees about x, y, z.
  **Captures** are accepted / rejected.
- "below the table top": the synthetic camera renders no occlusion, so a
  camera under the table still sees the board. Those cases test the maths,
  not a mount you would build.
- The carried board or camera ignores collisions with its touch links (the
  hand), so a few passing mounts overlap the hand. Same caveat.
- Eye-in-hand usually takes longer (96 captures vs 60) because it needs more
  bootstrap rounds, as in the demos; a few eye-to-hand cases needed more rounds
  too (Panda all+: 96).
