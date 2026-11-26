"""Capture rejection codes; values match the REJECT_* constants in Calibrate.action.

The core can't import the interface package (no ROS), so a ROS-side test
checks that both lists stay identical.
"""

NO_BOARD = 'no_board'
FEW_FEATURES = 'few_features'
POOR_SPREAD = 'poor_spread'
COLLINEAR = 'collinear'
BLURRY = 'blurry'
NEAR_BORDER = 'near_border'
REPROJECTION = 'reprojection'
AMBIGUOUS_POSE = 'ambiguous_pose'
CONSENSUS = 'consensus'
TF_UNAVAILABLE = 'tf_unavailable'
MARKER_SUPPORT = 'marker_support'
TWO_BOARDS = 'two_boards'

ALL = (NO_BOARD, FEW_FEATURES, POOR_SPREAD, COLLINEAR, BLURRY, NEAR_BORDER, REPROJECTION,
       AMBIGUOUS_POSE, CONSENSUS, TF_UNAVAILABLE, MARKER_SUPPORT, TWO_BOARDS)
