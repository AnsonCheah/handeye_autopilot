"""Board pose per frame: both planar solutions (02-features §D3).

Reuse: ``solvePnPGeneric`` with ``SOLVEPNP_IPPE`` (both solutions) and
``solvePnPRefineLM``. The two raw IPPE hypotheses are kept unrefined for the
ambiguity test (refining both can pull them into the same minimum); only the
best is refined for the measurement and the reprojection gate.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from numpy.typing import NDArray

from . import transforms as tf
from .camera import CameraModel


class PoseError(ValueError):
    pass


@dataclass
class PoseSolution:
    camera_T_board: NDArray[np.float64]
    rms_px: float
    worst_px: float
    mean_px: float


@dataclass
class BoardPose:
    best: PoseSolution                    # refined with solvePnPRefineLM
    hypotheses: list[PoseSolution]        # raw IPPE solutions, lowest error first


def _score(T: NDArray[np.float64], obj: NDArray[np.float64], img: NDArray[np.float64],
           camera: CameraModel) -> PoseSolution:
    err = np.linalg.norm(camera.project_object(T, obj) - img, axis=1)
    return PoseSolution(T, float(np.sqrt(np.mean(err ** 2))), float(err.max()),
                        float(err.mean()))


def solve(object_points: NDArray[np.float64], image_points: NDArray[np.float64],
          camera: CameraModel) -> BoardPose:
    """Raises PoseError when OpenCV finds no finite solution."""
    obj = np.ascontiguousarray(object_points, dtype=np.float64).reshape(-1, 3)
    img = np.ascontiguousarray(image_points, dtype=np.float64).reshape(-1, 2)
    if len(obj) < 4:
        raise PoseError(f'{len(obj)} points; PnP needs at least 4')
    try:
        n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj.reshape(-1, 1, 3), img.reshape(-1, 1, 2),
                                                 camera.K, camera.D, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error as e:
        raise PoseError(f'solvePnPGeneric failed: {e}') from e
    hyps = []
    for k in range(n):
        rvec, tvec = np.asarray(rvecs[k]), np.asarray(tvecs[k])
        if np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec)):
            hyps.append(_score(tf.from_rvec_tvec(rvec, tvec), obj, img, camera))
    if len(hyps) < 2:
        raise PoseError(f'IPPE returned {len(hyps)} finite solution(s), expected 2')
    hyps.sort(key=lambda s: s.rms_px)
    rvec, tvec = tf.to_rvec_tvec(hyps[0].camera_T_board)
    try:
        rvec, tvec = cv2.solvePnPRefineLM(obj.reshape(-1, 1, 3), img.reshape(-1, 1, 2),
                                          camera.K, camera.D, rvec.reshape(3, 1).copy(),
                                          tvec.reshape(3, 1).copy())
    except cv2.error as e:
        raise PoseError(f'solvePnPRefineLM failed: {e}') from e
    if not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
        raise PoseError('refined pose is not finite')
    best = _score(tf.from_rvec_tvec(rvec, tvec), obj, img, camera)
    return BoardPose(best, hyps)
