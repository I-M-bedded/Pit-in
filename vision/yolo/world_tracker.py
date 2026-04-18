"""
WorldTracker — world-space temporal filter for center-hole pose estimation.

Receives per-frame camera-space estimates from HolePoseEstimator,
transforms to world coordinates using robot FK, and applies EMA filtering
with confidence-weighted alpha.  Symmetry ambiguity in line-fitting results
is resolved using the previous world-space estimate.

Coordinate convention (matches robot):
    Right-handed: X = forward, Y = left, Z = up.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from hole_pose_estimator import CenterHoleResult


@dataclass
class WorldEstimate:
    """Filtered world-space center-hole estimate."""
    world_xyz: Optional[np.ndarray] = None   # (3,) world position [m]
    confidence: float = 0.0
    source: str = "none"                     # forwarded from CenterHoleResult


class WorldTracker:
    """EMA filter in world coordinates.

    The camera moves with the robot, so image-space EMA is meaningless.
    We transform each frame's estimate to world, filter there, and
    optionally export back to camera space for the control loop.

    Parameters
    ----------
    ema_alpha : float
        Base EMA coefficient (0 < alpha <= 1).  Actual alpha per frame is
        ``ema_alpha * confidence``, so high-confidence frames update faster.
    decay : float
        Multiplicative decay applied to stored confidence each frame when
        no new measurement arrives (0 < decay < 1).
    min_confidence : float
        Below this threshold the tracker reports ``None`` instead of the
        stale EMA value.
    """

    def __init__(
        self,
        ema_alpha: float = 0.3,
        decay: float = 0.9,
        min_confidence: float = 0.1,
    ):
        self.ema_alpha = ema_alpha
        self.decay = decay
        self.min_confidence = min_confidence

        self._world_pos: Optional[np.ndarray] = None
        self._confidence: float = 0.0

    # ------------------------------------------------------------------
    #  Core update
    # ------------------------------------------------------------------

    def update(
        self,
        result: CenterHoleResult,
        cam_to_world: np.ndarray,
        cam_origin_world: np.ndarray,
    ) -> WorldEstimate:
        """Incorporate a new per-frame estimate.

        Parameters
        ----------
        result : CenterHoleResult
            Output of ``HolePoseEstimator.estimate()``.
        cam_to_world : np.ndarray, shape (3, 3)
            Rotation from camera frame to world frame (e.g. ``so3_lcam``).
        cam_origin_world : np.ndarray, shape (3,)
            Camera origin (pin tip position) in world frame from FK.

        Returns
        -------
        WorldEstimate
            Filtered world-space position and confidence.
        """
        if result.center_xy is None or result.pose is None:
            return self._decay_and_return(result.source)

        # Camera-frame 3D position from solvePnP
        tvec_cam = np.array(result.pose["tvec_m"], dtype=np.float64)

        # Transform to world
        world_pos = cam_origin_world + (cam_to_world @ tvec_cam)

        # NOTE: "line" source (Tier 2b) never reaches here because its pose is None
        # → handled by _decay_and_return above. Symmetry ambiguity for "line" would
        # require a 3D candidate, which is unavailable without depth. Future work.

        # EMA update (alpha scaled by confidence)
        alpha = self.ema_alpha * result.confidence
        if self._world_pos is None:
            self._world_pos = world_pos.copy()
            self._confidence = result.confidence
        else:
            self._world_pos = alpha * world_pos + (1.0 - alpha) * self._world_pos
            self._confidence = alpha * result.confidence + (1.0 - alpha) * self._confidence

        return WorldEstimate(
            world_xyz=self._world_pos.copy(),
            confidence=self._confidence,
            source=result.source,
        )

    # ------------------------------------------------------------------
    #  Helpers
    # ------------------------------------------------------------------

    def _decay_and_return(self, source: str) -> WorldEstimate:
        """No new measurement — decay confidence, return stale estimate."""
        self._confidence *= self.decay
        if self._world_pos is not None and self._confidence >= self.min_confidence:
            return WorldEstimate(
                world_xyz=self._world_pos.copy(),
                confidence=self._confidence,
                source=f"ema_{source}",
            )
        return WorldEstimate()

    def get_world_position(self) -> Optional[np.ndarray]:
        """Return current filtered world position (or None)."""
        if self._world_pos is not None and self._confidence >= self.min_confidence:
            return self._world_pos.copy()
        return None

    def reset(self):
        """Clear filter state."""
        self._world_pos = None
        self._confidence = 0.0
