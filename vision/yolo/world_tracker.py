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
        ``ema_alpha * confidence ** conf_power`` — confidence is squared by
        default so low-conf Tier-2b frames barely move the filter.
    decay : float
        Multiplicative decay applied to stored confidence each frame when
        no new measurement arrives (0 < decay < 1).
    min_confidence : float
        Below this threshold the tracker reports ``None`` instead of the
        stale EMA value (output gate).
    update_min_confidence : float
        Frames below this confidence are treated as "no measurement" —
        we decay instead of blending them in.  Prevents noisy low-conf
        frames from dominating at 15 fps where unreliable detections pile up.
    conf_power : float
        Exponent applied to confidence before scaling alpha.  ``2.0`` by
        default — a 0.3-conf frame contributes 9x less than a 0.9-conf one.
    """

    def __init__(
        self,
        ema_alpha: float = 0.3,
        decay: float = 0.9,
        min_confidence: float = 0.1,
        update_min_confidence: float = 0.25,
        conf_power: float = 2.0,
        line_confidence_scale: float = 0.85,
        no_detection_decay: float = 1.0,
    ):
        self.ema_alpha = ema_alpha
        self.decay = decay
        self.min_confidence = min_confidence
        self.update_min_confidence = update_min_confidence
        self.conf_power = conf_power
        self.line_confidence_scale = line_confidence_scale
        self.no_detection_decay = no_detection_decay

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
            if not result.detections:
                return self._decay_and_return(result.source, self.no_detection_decay)
            return self._decay_and_return(result.source)

        confidence = self._effective_confidence(result)

        # Confidence gate: drop this frame as if no measurement arrived.
        # At 15 fps this prevents 15x/sec low-conf noise from eating the EMA.
        if confidence < self.update_min_confidence:
            return self._decay_and_return(result.source)

        # Camera-frame 3D position from solvePnP
        tvec_cam = np.array(result.pose["tvec_m"], dtype=np.float64)

        # Transform to world
        world_pos = cam_origin_world + (cam_to_world @ tvec_cam)

        # EMA update with non-linear confidence scaling — quadratic default
        # so a 0.9-conf conic frame dominates many 0.3-conf line frames.
        alpha = self.ema_alpha * (confidence ** self.conf_power)
        if self._world_pos is None:
            self._world_pos = world_pos.copy()
            self._confidence = confidence
        else:
            self._world_pos = alpha * world_pos + (1.0 - alpha) * self._world_pos
            self._confidence = alpha * confidence + (1.0 - alpha) * self._confidence

        return WorldEstimate(
            world_xyz=self._world_pos.copy(),
            confidence=self._confidence,
            source=result.source,
        )

    # ------------------------------------------------------------------
    #  Helpers
    # ------------------------------------------------------------------

    def _effective_confidence(self, result: CenterHoleResult) -> float:
        """Return tracker-side confidence after source-specific scaling."""
        confidence = float(result.confidence)
        if result.source == "line":
            confidence *= self.line_confidence_scale
        return confidence

    def _decay_and_return(self, source: str, decay: Optional[float] = None) -> WorldEstimate:
        """No new measurement — decay confidence, return stale estimate."""
        self._confidence *= self.decay if decay is None else decay
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
