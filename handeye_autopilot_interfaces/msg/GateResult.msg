# One quality gate: what was measured, its limit, and whether it passed.
# Units are SI (m, rad) or pixels; `unit` says which.

string GROUP_SANITY=sanity
string GROUP_FOLD_SENSITIVITY=fold_sensitivity
string GROUP_ACCURACY_EVIDENCE=accuracy_evidence

string name        # e.g. "min_inliers", "fold_translation_max"
string group       # one of the GROUP_* constants
string unit        # "m", "rad", "px", "count", ...
string comparison  # "<=" (value must not exceed limit) or ">=" (must reach it)
float64 value
float64 limit
bool gated         # false for accuracy evidence: reported, never fails the run
bool passed
