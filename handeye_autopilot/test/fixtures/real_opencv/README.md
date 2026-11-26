# Real board images from OpenCV

Real photographs used as detection fixtures (03-milestones M1a, real-image
probe). Copied unmodified from the OpenCV repository, `samples/data/`, at commit
`a91b2c67fb4d80cd844abeba5dd42c39fbb3d227` (branch 4.x):

| Files | Board | Intrinsics |
|---|---|---|
| `left01..left14.jpg` (no 10) | checkerboard, 10 x 7 squares (9 x 6 inner corners), 25 mm | `left_intrinsics.yml` (calibrated from these images) |
| `choriginal.jpg` (from `aruco/`) | ChArUco 5 x 7 squares, 40 mm / 20 mm, `DICT_6X6_250` | `tutorial_camera_charuco.yml` (from a different session of the tutorial camera) |

License: OpenCV is distributed under the Apache License 2.0
(https://github.com/opencv/opencv/blob/4.x/LICENSE); these files are part of
that distribution. Copyright the OpenCV contributors.

What they cover: real lens, lighting, print and JPEG effects on detection, the
checkerboard corner order, and the gates on real data. What they don't: robot
poses, stray markers, or a known ground truth (the intrinsics were estimated
from the same kind of images). The maintainer's own captures (M5) add those.
