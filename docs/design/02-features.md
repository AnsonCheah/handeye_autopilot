# Checkpoint 2 — Features and algorithms (rev 3.1, after pre-implementation Codex review)

Scope of v1: **eye-to-hand and eye-in-hand** hand-eye calibration. Numbers
marked *(tune)* come from simulation sweeps plus real-image fixtures, never from
earlier projects. Every "write" item ships with a test.

Cut from v1 after review (named so nobody re-adds them silently): CLAHE
retry, symmetric-checkerboard branch resolution, Jacobian covariance,
split-half (replaced by k-fold, §F), own exposure search (replaced by driver
ROI auto-exposure, §L). Exposure (§L) and marker masking (§D4) stay in v1
because the author asked for them, with the reviewed designs.

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

OpenCV argument mapping (OpenCV names transforms "a2b" = pose of `a` in `b` =
`b_T_a`):

| Call | Eye-in-hand inputs → output | Eye-to-hand inputs → output |
|---|---|---|
| `calibrateHandEye(gripper2base, target2cam)` → `cam2gripper` | `G_i`, `C_i` → `X = tool_T_camera` | `inv(G_i)`, `C_i` → `X = base_T_camera` |

`calibrateRobotWorldHandEye` returns both unknowns; its mapping is written by
the implementer **from a test first**: the ground-truth oracle generates
`G_i`, `C_i` for known `X`, `Z`, and the adapter's mapping for each mode is
accepted only when it recovers both. `Z` for `calibrateHandEye` seeds is
recovered as the rotation-averaged `Z = A_i · X · C_i` over captures.

Intrinsics are fixed (from `CameraInfo`), not refined.

## B. Session flow

1. **Seed.** Operator jogs to a pose where the board is fully visible and
   **tilted away from face-on** by at least a minimum angle *(tune)*; the node
   checks it (§D gates) and says what to fix if not. The tilt requirement avoids
   planar pose ambiguity from the very first sample.
2. **Bootstrap with incremental excitation.** No hand-eye estimate exists yet,
   but the current board pose in the camera (`C`, from PnP) is measured. Steps
   are checked **numerically, not by a closed-form bound** (small-angle
   formulas underestimate the shift away from the image centre):
   - the only unknown is the lever arm between the tool and the moving object
     (board for eye-to-hand, camera for eye-in-hand), bounded by the configured
     `L_max`. A candidate step (finite rotation about a tool axis, or a
     translation) is applied to the board corners for a set of worst-case lever
     arms: zero and `±L_max` along each tool axis, plus the 8 diagonal
     corners of that cube;
   - every resulting corner is projected with the full camera model
     (distortion included) via the mode adapter; the step is accepted only if
     all corners, for every lever arm, stay inside the image with the margin
     and stay in front of the camera, with a safety factor *(tune)*.
   - **Rounds are multi-axis from the start:** each round is ± a rotation
     about each of the three tool axes and ± a translation along each, so the
     first preliminary solve already has rotation about several axes.
   - After each round the node solves; the step **doubles only if** every
     capture in the round passed §D and the estimate of `X` changed less than
     a threshold *(tune)* since the previous round (k-fold is not used here:
     too few captures).
   - If the board is lost, return to the last good pose and halve that step;
     after a limit, fail with a message naming `L_max` as the likely culprit.
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

**Reuse:** `findChessboardCornersSB` (already subpixel; no `cornerSubPix`;
its starting corner varies, so corners are put in board order by handedness
and the colour of the first square),
OpenCV ChArUco detector (its own board-consistency checks), `solvePnPGeneric`
with `SOLVEPNP_IPPE`, `solvePnPRefineLM`.

**Targets in v1:** ChArUco, and checkerboards whose square counts are one odd
and one even **and** that pass an orientation-consistency fixture test in CI.
Symmetric checkerboards are rejected at config validation with a message
recommending ChArUco.

1. **Settle and per-frame TF** as in checkpoint 1 §8.
2. **Consensus:** take N frames *(tune)*, each with its own TF. Frame losses
   are bounded: at most `max_frames_without_tf` frames may lack TF at their
   stamp (else `TF_UNAVAILABLE`), and at most `max_failed_frames` may fail a
   gate (else the capture is rejected with the most common frame reason); at
   least `consensus_min_frames` must remain. Reject the capture if **any pair**
   of remaining frames disagrees beyond the translation/rotation limits (a
   bound from the median alone would allow pairs twice the limit apart). Keep
   **one** frame as the measurement (the medoid pose), not an average, so
   every measurement is a real exposure paired with its own robot pose and
   correlated frames are not counted as independent data.
3. **Per-capture gates**, each with a reason code. Order: the gates that need
   no pose (features, spread, collinearity, sharpness) run first, then PnP
   (a failed or non-finite fit is a `reprojection` rejection, never an
   exception), then reprojection, board outline, ambiguity:
   - enough features, spread over the board (convex-hull area fraction) and not
     collinear;
   - image sharpness in the board region (variance of Laplacian) *(tune)*;
   - **whole board** inside the image with a border margin: the board's
     outer outline, sampled densely and projected with the fitted pose and
     distortion (not just the detected corners, which can stay inside while
     the board is cropped);
   - mean and worst-corner reprojection of the PnP fit, **after**
     `solvePnPRefineLM`;
   - **pose ambiguity:** IPPE gives two poses. Reject when the runner-up's error
     is within a noise-scaled factor of the best **and** the two poses differ by
     more than a pose threshold (rotation **or** translation; nearly
     identical poses are not ambiguous). The comparison uses the two **raw**
     IPPE solutions' RMS errors; only the best is then refined with
     `solvePnPRefineLM` (refining both can pull them into one minimum).
     Applies in every phase; no exemptions. Its M1a fixture must be a view
     that **meets both conditions** (verified in the test, not assumed from
     "near face-on"); a near-face-on view that fails either condition is a
     separate fixture that must be accepted.
   - TF at the frame stamp unavailable or extrapolated.
4. **Stray-marker masking (ChArUco), on by default.** Case: a marker with an
   ID inside the board's range but not on the board (a different-size marker
   farther away that looks the same size, a reflection, another board).
   OpenCV doesn't solve it for us: `checkBoard` (4.7+) discards the whole board
   instead of the marker, and Jazzy's OpenCV 4.6 has no check at all.
   Order of operations:
   1. **Detect markers only** (ArUco detector), not the full board.
   2. **Duplicate IDs:** if an ID appears more than once, keep none of its
      copies for the RANSAC fit; after the fit, keep at most one inlier copy
      (the lowest-error one), else drop all. If fewer than `min_markers`
      unique IDs remain (e.g. two complete boards: every ID twice), fit on
      **all** copies instead; RANSAC treats the wrong copies as outliers.
      Fewer than `min_markers` detections in total: `marker_support`.
   3. **RANSAC:** `cv2.solvePnPRansac` on all marker corners against their
      board coordinates. Markers with any outlier corner are dropped. This
      tests **projection consistency** with one board pose: a stray marker
      whose projected position and shape agree with the board's plane (e.g.
      lying exactly where the board would continue) cannot be told apart, and
      the docs say so.
   4. **Support checks:** reject the capture if the inlier markers are too
      few or not spread over the board (convex-hull fraction), or if a
      **second** RANSAC on the outliers finds a consistent group of comparable
      size, counted in **distinct IDs** (two boards in view). The second fit
      uses only markers that do **not** fit the first pose (extra copies that
      do are repeats of the same board). `two_boards` is
      only returned on that geometric evidence, never on duplicate IDs alone,
      so scattered stray copies of board IDs are masked, not mistaken for a
      second board.
   5. **Interpolate ChArUco corners from the inlier markers only**, passing
      them explicitly so the detector does not re-detect the dropped markers
      (with an empty inlier set the capture is rejected before this call).
      Then `cornerSubPix` once more with a window kept inside the white border
      around the markers (~0.8 of its width): OpenCV's interpolated corners
      are ~0.5 px off on 4.6 and 4.10, and a window reaching into the marker
      bits biases them by 0.2–0.3 px (M1a measurement on rendered boards).
   6. Continue with the normal gates (§D3).
   **Expected outcomes, one fixture each (M1a):**
   - *masking succeeds:* one isolated stray marker, enough board markers left
     → the stray marker is dropped, the capture is **accepted**, and its
     corners match the clean-view corners;
   - *rejected after masking:* strays (or occlusion) leave too few or
     poorly spread inlier markers → rejected with the support-check code;
   - *two boards:* a second consistent group → rejected with the two-boards
     code.
   A real-image stray-marker fixture follows in M5.
5. **Record** per capture (the **capture record**, ROS-free; the M1a → M1b
   hand-off and the unit of the observations YAML):
   - `capture_id` (acquisition order), `stamp` (acquisition stamp, float s);
   - camera model: `K` (3×3), `D`, distortion model name, image size,
     rectified flag;
   - `ids`: corner IDs in ascending order; `object_points` (N×3, board frame,
     m) and `image_points` (N×2, px) in the same order;
   - `tool_pose`: `base_T_tool` paired with this frame's stamp (4×4);
   - `camera_T_board`: the refined board pose of the kept frame (4×4), the
     measurement the solver uses;
   - `ippe_poses`: both raw IPPE poses (4×4, best first) and `ippe_errors_px`
     (their RMS reprojection errors);
   - `accepted` and, if rejected, the `REJECT_*` code and detail.
   Accepted records must have every field, with exactly two IPPE poses
   (checked on construction and on load). Rejected records keep what existed
   for the frame that decided the rejection; later fields are `null`. `ids`
   is always a list (`[]` when no board was found); `object_points`,
   `image_points`, `camera_T_board`, `ippe_*` are `null` when they were never
   computed; `tool_pose` is `null` when TF was missing. The solver reads
   accepted records only.
   The **observations YAML** is a list of capture records plus run metadata
   (`mode`; `frames` with `base_frame`, `tool_frame`, `camera_frame`; the
   board definition) and a top-level `schema_version: 1` (an integer, bumped
   on any incompatible change; the loader refuses unknown or non-integer
   versions and missing metadata).

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
4. **Observability:** fail with a message ("add rotation about another
   axis") rather than return a confident wrong answer. Definitions (fixed so
   thresholds keep their meaning):
   - **Relative rotations:** over **all pairs** `i < j` of captures in the
     set, `R_ij = R_i⁻¹ R_j` of the moving rotations `A_i` (§A). Amplitude
     `θ_ij`, unit axis `a_ij`.
   - **Rotation amplitude** = `max θ_ij` (rad).
   - **Axis spread:** over pairs with `θ_ij` ≥ a minimum amplitude *(tune)*,
     `M = mean(a_ij a_ijᵀ)` (sign-free); with eigenvalues `λ1 ≥ λ2 ≥ λ3`,
     spread = `2·λ2` ∈ [0, 1] (0 = all axes parallel, which leaves the
     hand-eye problem underdetermined; 1 = axes spanning at least a plane
     evenly). With fewer than 2 qualifying pairs, spread = 0 (fails).
   - **Scaled smallest singular value:** at the solution, take the raw
     residual vector `r` (pixel errors, one element per corner coordinate)
     and its Jacobian `J_r` with respect to the 12 tangent parameters. Let
     `z_k = (r_k / s)²` with `s = f_scale` in pixels (`f_scale` = σ-multiple ×
     σ, §E2) and `w_k = ρ'(z_k)` for the `soft_l1` loss (`w_k = 1/√(1+z_k)`).
     Rows: `J_k = √w_k · J_r,k / σ`. Columns: translation as is, rotation
     columns divided by `d` (median board depth), so every column is per
     metre at the working distance. Reported value = smallest singular value
     of that `J` (unit 1/m).
   - **Where applied:** amplitude and spread before the first fit (on the
     accepted captures), again after outlier removal, and on each fold's
     training set; the singular value after the final fit and after each fold
     fit.

## F. Quality gates and honest naming

Three groups, named for what they measure:

- **Data sanity** (broken data): minimum inliers, reprojection ceilings,
  observability.
- **Fold sensitivity** (how much the estimate depends on which captures were
  used): from the k-fold refits (§G), with `k = 5` by default. For each fold,
  the translation distance and the SO(3) angle between that fold's `X` and the
  all-data `X`; the gate uses the **maximum** over folds, against fixed
  thresholds tuned in simulation. Overlapping training sets make these
  estimates correlated, so this is **not** a calibrated confidence interval
  and is never reported as one. If any fold cannot be solved or fails
  observability, the gate **fails** (the run cannot pass).
- **Accuracy evidence** (closeness to truth): in v1 only held-out
  reprojection (§G), i.e. evidence the solve did not use. Reported, not gated.
  Docs state plainly that shared errors (intrinsics, robot kinematics, board
  print scale) shift every fold the same way, so neither gate group detects
  them.

`passed` = sanity + fold-sensitivity gates; accuracy evidence is reported
alongside.

## G. Verification (no leakage)

1. **k-fold by capture:** captures are assigned to folds by index modulo `k`
   in acquisition order (deterministic; each fold spans the whole run). For
   each fold, the full training pipeline (outlier rejection included) runs on
   the other folds only. Held-out captures are scored with the **predicted**
   board pose `Ĉ_i` (no per-capture PnP fit). Each training set must pass
   observability; a fold whose training set fails to solve **or** fails
   observability counts as failed, is reported, and fails the
   fold-sensitivity gate (§F). The same refits feed that gate.
   **Held-out aggregates** pool every corner of every accepted held-out
   capture from the successful folds: RMS = √(mean of squared Euclidean
   corner errors, px); max = largest corner error.
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
- **Expected-failure tests** (faults the pipeline can detect), each owned by
  the milestone that builds the detecting code and failing with its committed
  reason code: genuinely ambiguous views and rejected-after-masking stray
  markers (M1a); parallel rotation axes (M1b); too small `L_max` (M2,
  bootstrap); delayed/bad timestamps (M3, TF and clock handling).
- **Known-blind-spot tests** (biases that stay internally consistent): wrong
  board square size, small systematic robot kinematic error, small intrinsic
  error. These runs are **expected to pass** the gates while the result is
  biased; the test asserts both, and records the size of the bias, so the docs
  can state what the gates cannot see.
- **Real-image fixtures:** a small set of real captures (from M5 hardware runs)
  is added to tests for detection gates and σ tuning. Simulation alone does not
  set thresholds.
- Stated limit: the renderer validates planar pinhole geometry with distortion
  and the integration; it does not validate optics realism, occlusion or
  motion blur.

## I. Visualisation

RViz2 + `MarkerArray` + OpenCV drawing. We write marker
builders for the safety box, envelope, candidates by status, frustum, detected
board, and an annotated image with the rejection reason.

## J. Targets and printing

ChArUco (OpenCV dictionaries) and parity-asymmetric checkerboards in v1.
Checkerboards need at least 4 squares per side (the detector fails on a
2-corner side). Board images: OpenCV `CharucoBoard.generateImage` for ChArUco;
a short NumPy function for checkerboards; the `board_image` tool writes the
print resolution into the PNG (`pHYs`) so a 100 % print has the stated square
size. YAML distinguishes **square counts** from **inner-corner counts**
explicitly. Docs: measure a printed square with calipers.

A detected checkerboard grid is accepted only if (≥ 97 % of) its corners are
true X-junctions with the board's colours: the four squares touching each
corner are sampled close to it (¼ square along each diagonal; samples outside
the image don't count). The M1a pose sweep found `findChessboardCornersSB`
occasionally returning a grid turned by 90° that uses the board edge against
the white margin as a corner row (those L-shaped "corners" fail: 83 % in the
fixture); this check rejects it as `no_board` instead of passing wrongly
ordered corners. A first version checked every square's centre, outer ring
included; real OpenCV sample prints, whose outer squares are cut short,
failed it, hence sampling next to the corners.

## L. Exposure: point the driver's auto-exposure at the board (off by default)

**Reuse:** the camera driver's own auto-exposure (AE) with a region of
interest, set through rclpy `AsyncParameterClient`. No search algorithm of
ours.

**Supported in v1: `realsense2_camera` only**, which exposes AE-ROI parameters
on its colour sensor. The ZED wrapper's documented ROI is a different control
(processing region), so ZED is marked unsupported until an AE-ROI endpoint is
verified (`shortcut: ZED unsupported, add when its AE ROI is verified`).
Other cameras: the feature reports "unsupported" and the run continues;
lighting-related rejections are counted in the report.

**Write:** a small YAML map per driver (parameter names, ROI coordinate
convention, valid range) and the update logic:
1. At goal start, read and remember the driver's AE enable state and ROI;
   enable AE if needed.
2. After a detection, map the board's padded bounding box from image pixels
   to the driver's ROI coordinates (sensor resolution; clipped to the valid
   range), write it, and check the parameter response succeeded.
3. **Convergence:** capture only after the mean intensity inside the board
   region changes by less than a threshold over M consecutive frames
   acquired after the write *(tune)*, with a timeout; on timeout the capture
   proceeds and is flagged.
4. Restore the remembered AE state and ROI on exit, error or cancel.

`shortcut: no own exposure search, add one only if ROI auto-exposure proves
insufficient on real runs.`

## K. Not in v1

TCP, fixture and intrinsic calibration; circle grids; ArUco/AprilTag grids;
markerless; multi-camera; rqt UI; PyPI; non-MoveIt motion; and the review cuts
listed at the top.
