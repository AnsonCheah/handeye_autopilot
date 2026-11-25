# Checkpoint 2 — Features and algorithms (rev 2, after Codex review)

Scope of v1: **eye-to-hand and eye-in-hand** hand-eye calibration. Numbers
marked *(tune)* come from simulation sweeps plus real-image fixtures, never from
earlier projects. Every "write" item ships with a test.

Cut from v1 after review (named so nobody re-adds them silently): CLAHE
retry, symmetric-checkerboard branch resolution, Jacobian covariance,
split-half (replaced by k-fold, §F). Exposure search (§L) and marker masking
(§D4) stay in v1 because the author asked for them, with the reviewed designs.

## A. Geometry (one model, both modes)

Notation: `G_i = base_T_tool` (from TF), `C_i = camera_T_board` (from PnP),
`X` = the published unknown, `Z` = the other unknown.

| Mode | `X` | `Z` | `A_i` |
|---|---|---|---|
| eye-to-hand | `base_T_camera` | `tool_T_board` | `inv(G_i)` |
| eye-in-hand | `tool_T_camera` | `base_T_board` | `G_i` |

Model: `A_i · X = Z · inv(C_i)`, i.e. the robot-predicted board pose is
`Ĉ_i = inv(X) · inv(A_i) · Z`.

The **mode adapter** is the only mode-specific code. It builds `A_i`, converts
inputs/outputs to and from OpenCV's conventions for the initial solvers
(including recovering `Z` when a method only returns `X`), and maps a desired
camera-relative board pose to a tool pose. Unit tests check each direction
against an independent ground-truth oracle (a swapped inverse must fail).

Intrinsics are fixed (from `CameraInfo`), not refined.

## B. Session flow

1. **Seed.** Operator jogs to a pose where the board is fully visible and
   **tilted away from face-on** by at least a minimum angle *(tune)*; the node
   checks it (§D gates) and says what to fix if not. The tilt requirement avoids
   planar pose ambiguity from the very first sample.
2. **Bootstrap with incremental excitation.** No estimate exists yet, so steps
   are derived from what is measured:
   - image-shift bound for a rotation `θ` about the tool:
     `shift ≲ f·θ·(1 + L_max / d)`, where `d` is the measured board depth,
     `f` the focal length, and `L_max` the configured upper bound on the lever
     arm (board-to-tool for eye-to-hand, camera-to-tool for eye-in-hand). `θ`
     is chosen so the shift stays inside the board's current image margin with a
     safety factor;
   - translation steps likewise from margin and depth.
   Moves start small; after each round the node solves and **grows the step
   only when the preliminary estimate is stable** (precision measure of §F
   below a threshold). Every bootstrap capture must pass all §D gates. If the
   board is lost, return to the last good pose and halve the step; after a
   limit, fail with a message naming `L_max` as the likely culprit.
   An optional rough initial guess (e.g. CAD) only sets the first step size; it
   is never trusted for visibility.
3. **Bounded centring.** Bring the board toward image centre at nominal depth in
   checked increments (each one detected before the next), not one jump.
4. **Candidate generation** (§C), **execution**, **final solve** (§E),
   **gates** (§F).

The state machine is one plain class in `handeye_autopilot.core`; the node drives it.

## C. Candidate generation (main contribution)

**Reuse:** SciPy `qmc.Halton`, OpenCV `projectPoints` (with distortion),
`image_geometry`, `compute_ik`.

1. **Sample** camera-relative board poses: board-centre pixel `(u, v)` →
   ray through the camera model (de-projection with distortion), depth along
   the ray, tilt about two in-plane axes, roll about the optical axis. The
   rotation pivot is the board centre.
2. **Visibility filter:** positive depth; printed face toward the camera;
   board outline (densely sampled along its edges, projected with distortion)
   inside the image with a margin **plus a term for the current estimate's
   uncertainty**; coverage band; minimum projected spacing of features and, for
   ChArUco, minimum projected marker-module width.
3. **Convert** to tool poses through the mode adapter.
4. **Robot-agnostic limits:** safety box, envelope, orientation limit from the
   seed.
5. **`compute_ik`** (checkpoint 1 §3.1). Keep the returned joint solution.
6. **Select for information, not just spread:** greedy selection over relative
   rotations (axis and angle relative to already selected poses), so rotation
   axes are spread. After selection, check the **scaled** information of the
   set (smallest singular value of the Jacobian with parameters scaled to
   metres/radians at the working distance); add poses until it clears a
   threshold *(tune)* or report the plan as weak.
7. **Spares:** plan length = target count × reserve factor.
8. **Ordering:** greedy tour by **joint-space distance** between the IK
   solutions. At execution time MoveIt plans from the actual current state, so
   a pose that fails then is skipped and a spare used.

## D. Capture and detection

**Reuse:** `findChessboardCornersSB` (already subpixel; no `cornerSubPix`),
OpenCV ChArUco detector (its own board-consistency checks), `solvePnPGeneric`
with `SOLVEPNP_IPPE`, `solvePnPRefineLM`.

**Targets in v1:** ChArUco, and checkerboards whose square counts are one odd
and one even **and** that pass an orientation-consistency fixture test in CI.
Symmetric checkerboards are rejected at config validation with a message
recommending ChArUco.

1. **Settle and per-frame TF** as in checkpoint 1 §8.
2. **Consensus:** take N frames *(tune)*, each with its own TF. Reject the
   capture if the frames' board poses disagree beyond a threshold. Keep **one**
   frame as the measurement (the one closest to the median pose), not an
   average, so every measurement is a real exposure paired with its own robot
   pose and correlated frames are not counted as independent data.
3. **Per-capture gates**, each with a reason code:
   - enough features, spread over the board (convex-hull area fraction) and not
     collinear;
   - image sharpness in the board region (variance of Laplacian) *(tune)*;
   - board inside the image with a border margin;
   - mean and worst-corner reprojection of the PnP fit, **after**
     `solvePnPRefineLM`;
   - **pose ambiguity:** IPPE gives two poses. Reject when the runner-up's error
     is within a noise-scaled factor of the best **and** the two poses differ by
     more than a pose threshold (nearly identical poses are not ambiguous).
     Applies in every phase; no exemptions.
   - TF at the frame stamp unavailable or extrapolated.
4. **Stray-marker masking (ChArUco), on by default.** Case: a marker with an
   ID inside the board's range but not on the board (a different-size marker
   farther away that looks the same size, a reflection, another board).
   OpenCV doesn't solve it for us: `checkBoard` (4.7+) discards the whole board
   instead of the marker, and Jazzy's OpenCV 4.6 has no check at all.
   **Reuse:** `cv2.solvePnPRansac` on all detected marker corners against
   their board coordinates; markers whose corners are RANSAC outliers are
   dropped. This tests *position on the board plane*, so a stray marker of
   similar apparent size still fails. Then **re-run ChArUco corner
   interpolation with inlier markers only** and continue with the normal gates;
   reject the capture if too few spread corners remain. Real-image fixture with
   a stray marker in view is part of the M5 test set; a synthetic one in M1.
5. **Record** per capture: the kept frame's corner IDs and pixels, board
   coordinates, its tool pose, both IPPE poses and errors, the reason if
   rejected.

## E. Solver

**Reuse:** OpenCV `calibrateRobotWorldHandEye` and `calibrateHandEye` (all
methods), SciPy `least_squares`, `scipy.stats.median_abs_deviation`.

1. **Initial guess:** run each OpenCV method through the mode adapter; score
   each by robot-predicted reprojection (`Ĉ_i`); keep the best.
2. **Refinement:** both unknowns (12 parameters, rotation as a local tangent
   update), minimising corner reprojection of `Ĉ_i` over all accepted captures;
   pixel noise scale σ *(tune from real fixtures)*; robust `soft_l1` loss with
   `f_scale = σ`-multiple *(tune)*.
3. **Outlier rejection:** per-capture RMS of robot-predicted reprojection;
   threshold `median + k·MAD` with a pixel floor; refit once without outliers.
   Rerun the observability check after removal; if too few captures remain or
   the set is degenerate, **fail**, don't continue.
4. **Observability:** before solving, check the spread of **relative**
   rotation axes and the rotation amplitude between captures; after solving,
   the scaled smallest singular value of the Jacobian. Fail with a message
   ("add rotation about another axis") rather than return a confident wrong
   answer.

## F. Quality gates and honest naming

Three groups, named for what they measure:

- **Data sanity** (broken data): minimum inliers, reprojection ceilings,
  observability.
- **Precision / stability** (how repeatable the estimate is from *this* data):
  k-fold refits (§G) give the spread of `X` across folds, reported as
  translation (mm) and rotation (SO(3) angle, degrees). Gate on it.
- **Accuracy** (closeness to truth): only from evidence the solve did not use:
  held-out reprojection (§G), the optional reference check, live verification.
  Docs state plainly that shared errors (intrinsics, robot kinematics, board
  print scale) bias every fold equally, so precision ≠ accuracy.

`passed` = sanity + precision gates; accuracy evidence is reported alongside.

## G. Verification (no leakage)

1. **k-fold by capture:** for each fold, the full training pipeline (outlier
   rejection included) runs on the other folds only. Held-out captures are
   scored with the **predicted** board pose `Ĉ_i` (no per-capture PnP fit).
   Each fold must pass observability; failed folds are reported, not hidden.
   The same refits feed the precision measure in §F.
2. **Offline re-solve:** `ros2 run handeye_autopilot resolve <run_dir>`.

Reference check (CAD) and live verify mode: v1.x, not v1.

## H. Simulation (and its limits)

- Renderer: warp the board texture to the image with the **true** poses;
  nonzero lens distortion is applied with a per-pixel remap from the camera
  model, so distortion handling is exercised. Noise, blur, brightness vary.
- **True vs reported:** the robot pose the pipeline reads is the true pose
  plus configurable noise and a configurable systematic error, so tests can
  show what the gates catch and what they can't.
- Geometry oracle in tests is written independently of the mode adapter.
- **Expected-failure tests:** parallel rotation axes, near-face-on ambiguous
  views, delayed/bad timestamps, wrong board square size, wrong `L_max`. Each
  must fail with the right reason.
- **Real-image fixtures:** a small set of real captures (from M5 hardware runs)
  is added to tests for detection gates and σ tuning. Simulation alone does not
  set thresholds.
- Stated limit: the renderer validates planar pinhole geometry with distortion
  and the integration; it does not validate optics realism, occlusion or
  motion blur.

## I. Visualisation

RViz2 + `MarkerArray` + `interactive_markers` + OpenCV drawing. We write marker
builders for the safety box, envelope, candidates by status, frustum, detected
board, and an annotated image with the rejection reason.

## J. Targets and printing

ChArUco (OpenCV dictionaries) and parity-asymmetric checkerboards in v1.
Board images: OpenCV `CharucoBoard.generateImage` for ChArUco; a short NumPy
function for checkerboards. YAML distinguishes **square counts** from
**inner-corner counts** explicitly. Docs: measure a printed square with
calipers.

## L. Exposure: point the driver's auto-exposure at the board (off by default)

**Reuse:** the camera driver's own auto-exposure with a region of interest,
set through rclpy `AsyncParameterClient`. No search algorithm of ours.
**Write:** after each detection, set the driver's auto-exposure ROI to the
board's bounding box (padded); a YAML map of each driver's ROI parameter names
(RealSense and ZED shipped). Before capture, wait for frames acquired after the
write. Restore the original ROI on exit, error or cancel. Cameras without an
ROI auto-exposure parameter: the feature reports "unsupported" and the run
continues; lighting-related rejections are counted in the report.
`shortcut: no own exposure search, add one only if ROI auto-exposure proves
insufficient on real runs.`

## K. Not in v1

TCP, fixture and intrinsic calibration; circle grids; ArUco/AprilTag grids;
markerless; multi-camera; rqt UI; PyPI; non-MoveIt motion; and the review cuts
listed at the top.
