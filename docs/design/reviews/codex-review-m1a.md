# Codex review — M1a (detection and synthetic data), 2026-11-26

**M1a is not ready to close.** I reproduced a checkerboard crash and confirmed that the two-board fixture bypasses the required second RANSAC. The capture record also diverges from the §D5 hand-off contract.

### BLOCKER

- [detection.py:164](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/detection.py:164) — Duplicate IDs alone trigger `TWO_BOARDS`. On 4.6, the fixture exits with “17 marker IDs seen twice” before either fit. The support-rejection fixture also exits before RANSAC, with only three usable detections. Require geometric evidence for `TWO_BOARDS`, count distinct inlier IDs in the second fit, and return `MARKER_SUPPORT` when there is insufficient evidence to fit. Add fixtures that reach the second fit and the post-fit support check, plus incoherent duplicates that must not yield `TWO_BOARDS`.
- [capture.py:97](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/capture.py:97) — IPPE runs before the feature and image gates. Exact collinearity produced NaN poses on 4.6. A failed solve can also escape the capture as an exception. Run the pre-fit gates first, then convert failed or nonfinite fits into a rejection. Route the collinear regression through `process_frame`.
- [capture.py:58](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/capture.py:58) — §D5 requires unavailable fields to serialize as `null`. Missing points, poses, and errors currently become `[]`. The consensus-count rejection can also discard an already processed frame, and the loader accepts records with a missing stamp or only one IPPE solution. Preserve the selected frame’s available fields, keep `ids: []` for no board, and validate the complete accepted-record contract on construction and load. [observations.py:32](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/observations.py:32) should also require an integer schema version and the specified mode and frame metadata.
- [boards.py:45](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/boards.py:45) — A parity-valid 3×4 checkerboard passes validation, then crashes `findChessboardCornersSB` on 4.6 because one inner dimension is two. Require at least four squares on each checkerboard axis and document the limit.

### SHOULD-FIX

- [gates.py:56](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/gates.py:56) — The border gate measures detected inner corners. A cropped board can pass when those corners remain inside the margin. Check the projected board outline with the refined pose and add a fixture whose inner corners clear the margin while the outline crosses it.
- [capture.py:133](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/capture.py:133) — Consensus bounds distance from the medoid, allowing two frames nearly twice the configured limit apart. It also silently drops failed frames when enough good frames remain. Define the allowed frame losses in §D2, enforce the disagreement limit across frame pairs, and test that the chosen stamp, corners, and **distinct** tool pose come from the same frame. The current pairing test gives every frame the same tool pose.
- [detection.py:143](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/detection.py:143) — The global median marker size and two-pixel minimum can put the refinement window into marker bits on small or foreshortened parts of the board. Bound it using local marker scale and skip refinement when no safe window fits. [pose.py:40](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/pose.py:40) also refines both IPPE branches without constraints, so they can converge to the same minimum. Preserve the raw hypotheses for the ambiguity comparison and specify which errors drive that comparison. The fixture assertion should exercise the translation alternative as well as rotation. [test_detection_fixtures.py:76](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_detection_fixtures.py:76)
- [test_detection_fixtures.py:29](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/test/test_detection_fixtures.py:29) — The coordinate assertion reuses the detector’s own coordinate table, and any nonempty sorted ID subset can pass. Commit independent expected IDs and board coordinates, compare stray-view pixels directly with clean-view pixels, and add an even-row ChArUco fixture and the opposite checkerboard parity layout. The current checkerboard ordering passed the stored rotated views.
- [cv_compat.py:104](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/cv_compat.py:104) — The class API worked on 4.10, and 4.12 documents the same `detectBoard` arguments. Its default marker-consistency check is a behavioral difference to cover with a partially masked board. Make the intended check explicit where supported and run the fixture suite on 4.12. Replace the test that requires OpenCV’s raw interpolation to retain a 0.5 px error with an assertion on final corner accuracy. [OpenCV 4.12 detector API](https://docs.opencv.org/4.12.0/d9/df5/classcv_1_1aruco_1_1CharucoDetector.html), [OpenCV 4.12 ChArUco parameters](https://docs.opencv.org/4.12.0/df/d01/structcv_1_1aruco_1_1CharucoParameters.html)
- [board_image.py:40](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/cli/board_image.py:40) — `--dpi` changes raster dimensions, but the PNG writer stores no physical scale. A 100% print can therefore have the wrong square size. Write PNG resolution metadata or supply explicit physical print dimensions, and test that contract.

### NIT

- [boards.py:116](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/boards.py:116) — Reuse the OpenCV dictionary’s `bytesList` length instead of inferring capacity from the name and guessing 1000. Core import-purity tests passed. The new modules fit the existing reuse rows, though the CLI and transform helpers should name their rows explicitly. Mark any retained duplicate-count or global-window heuristic with the required `shortcut:` limit and upgrade trigger.

**Done-when gaps.** The synthetic PNGs are present in the working tree but uncommitted. The spec records fixed seeds and individual poses, with no explicit pose ranges. Exact cross-version expectations and branch-exercising masking fixtures remain outstanding. The real-image probe and detection run still require the maintainer’s captures. [M1a acceptance criteria](/home/anson/workspaces/handeye_autopilot/docs/design/03-milestones.md:66)

**Verification.** OpenCV 4.6 passed 52 tests and 4.10 passed 31 focused tests. A direct comparison of the stored fixtures gave identical IDs, board coordinates, and detected pixels on both versions wherever corners were returned. The read-only sandbox prevented three file-writing tests, and I did not execute 4.12.

---

## Verification pass (after fixes)

### Prior findings

- **RESOLVED** Duplicate-ID rejection now requires a second geometric fit with distinct marker IDs. Fixtures exercise both fits, post-fit support rejection, and incoherent duplicates.
- **PARTIAL** Pre-fit gates now precede IPPE, and failed or nonfinite solves become rejections. The collinearity regression enters through `process_detection` and bypasses `process_frame`.
- **PARTIAL** Null fields, consensus frame retention, and integer schema version are fixed. Accepted records still admit malformed values such as a null IPPE pose or `accepted: "false"`, and metadata accepts non-string frame names.
- **RESOLVED** Checkerboards require at least four squares per axis, with documentation and a regression test.
- **RESOLVED** The border gate checks the refined pose’s projected outline and has a cropped-board regression.
- **RESOLVED** §D2 defines frame-loss limits. Consensus checks every pair, and the pairing test uses distinct tool poses.
- **RESOLVED** Refinement uses local marker scale and drops corners with undersized windows. Ambiguity uses raw IPPE RMS errors, with coverage for the translation condition.
- **RESOLVED** The fixtures check independent board coordinates and exact expected IDs, compare stray and clean pixels directly, and cover even-row ChArUco and both checkerboard parity layouts.
- **PARTIAL** The compatibility shim sets marker checking explicitly, and the fixtures cover masked input and final corner accuracy. CI targets 4.12, but I could only verify 4.6 locally.
- **RESOLVED** Printable PNGs include physical resolution metadata, with a test for the scale.
- **PARTIAL** Dictionary capacity comes from `bytesList`, and the refinement shortcut is documented. The CLI and transform helpers still lack explicit reuse-map references.
- **PARTIAL** The fixture spec now has seeded pose ranges and branch coverage. The PNGs and spec are still untracked, and this review did not verify cross-version parity.
- **OPEN** The real-image probe and detection run await the maintainer’s camera captures.

### New problems

- **SHOULD-FIX** [detection.py:243](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/detection.py:243) — Extra copies that already fit the primary pose enter the second fit. Duplicating all 17 marker detections at identical pixels reproduced `TWO_BOARDS` on 4.6. Build the second-fit pool from geometric outliers before deduplicating primary inliers.
- **SHOULD-FIX** [capture.py:154](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/core/capture.py:154) — A mixed-TF rejection records `frames[0].stamp` even when that frame had TF. Use the first missing-TF frame as the source and check its stamp in a mixed-TF regression.
- **SHOULD-FIX** [board_image.py:42](/home/anson/workspaces/handeye_autopilot/handeye_autopilot/handeye_autopilot/cli/board_image.py:42) — `--dpi 0` raises `ZeroDivisionError`. Reject nonpositive DPI, negative margins, and resolutions that round to zero pixels per square before rendering.

OpenCV 4.6 passed 73 ROS-free tests. Three filesystem-writing tests were deselected in the read-only sandbox.

**Applied after the verification pass:** second fit uses geometric outliers only (exact-duplicate regression test); mixed-TF rejection keeps the missing frame's stamp; CLI rejects non-positive dpi / negative margin / <10 px squares; loader rejects non-boolean `accepted`, null IPPE poses, non-string frame names; collinear regression now renders a real one-row view through `process_frame`; reuse-map rows name `cli/board_image.py` and `core/transforms.py`. Open: real-image probe (needs the maintainer's camera).
