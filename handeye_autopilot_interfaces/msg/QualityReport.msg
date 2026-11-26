# Quality of one calibration run (docs/design/02-features.md §F-G).
# ROS side uses SI units (m, rad); reprojection is in pixels.
# Fold sensitivity is NOT a confidence interval (folds overlap).
# Float metrics that were not computed are NaN (set explicitly by the node;
# message defaults are 0.0); a measured zero is 0.0.
# Gates that were not evaluated are absent from `gates`.

# Every gate with value, limit, group and outcome.
handeye_autopilot_interfaces/GateResult[] gates

# Data sanity
uint32 captures_accepted        # captures that entered the solve
uint32 captures_rejected        # rejected by per-capture gates
uint32 inliers                  # captures kept after outlier rejection
uint32 outliers                 # captures removed by outlier rejection
float64 reprojection_rms_px     # robot-predicted reprojection, all inliers
float64 reprojection_max_px     # worst corner over all inliers

# Observability
float64 relative_rotation_max_rad      # largest rotation between captures
float64 rotation_axis_spread           # 2*lambda2 of mean(a a^T) over relative axes, 0..1
float64 scaled_min_singular_value      # of the sigma- and metre-scaled Jacobian, 1/m

# Fold sensitivity (k-fold refits vs the all-data estimate; worst fold)
uint32 folds                    # k
uint32 folds_failed             # training set failed to solve or failed observability; >0 fails the run
float64 fold_translation_max_m
float64 fold_rotation_max_rad

# Accuracy evidence (held-out captures, scored with the predicted board pose):
# all corners of accepted held-out captures from successful folds, pooled
float64 heldout_reprojection_rms_px
float64 heldout_reprojection_max_px
