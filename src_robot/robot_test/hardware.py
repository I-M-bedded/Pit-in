"""Hardware interface layer.

Lives outside the worker file so that ``jacobian_solver`` can depend on
the safety-limit callable without importing the PBVS workers, and so that
the orchestrator can construct/wire it directly.
"""

from typing import List

import numpy as np

from controller import ik


class HardwareInterface:
    """Convert q_cmd (m, rad) <-> encoder counts and clamp to joint limits."""

    def __init__(self, topik: ik.Topik, cfg):
        self.topik = topik
        self.cfg = cfg
        self.m2cnt = topik.m2cnt.copy()
        self.cnt2m = topik.cnt2m.copy()

    def q_to_cnt(self, q_cmd: np.ndarray) -> List[int]:
        return [int(np.round(q_cmd[i] * self.m2cnt[i])) for i in range(7)]

    def cnt_to_q(self, cnt: np.ndarray) -> np.ndarray:
        return np.array([cnt[i] * self.cnt2m[i] for i in range(7)])

    def get_velocity_profile(self, q_dot: np.ndarray) -> List[int]:
        vel = np.zeros(7)
        for i in range(7):
            vel[i] = abs(q_dot[i] * self.m2cnt[i])
        for i in range(7):
            vel[i] = min(vel[i], self.cfg.t_vel[i])
            vel[i] = max(vel[i], 100)
        return [int(v) for v in vel]

    def apply_safety_limits(self, q_cmd: np.ndarray) -> np.ndarray:
        q_safe = q_cmd.copy()

        sl = self.cfg.stage_limit
        q_safe[0] = np.clip(q_safe[0], -sl, sl)
        q_safe[1] = np.clip(q_safe[1], -sl, sl)

        q_safe[2] = np.clip(q_safe[2], -0.325, 0.065)
        q_safe[3] = np.clip(q_safe[3], -1.7, 1.025)
        q_safe[4] = np.clip(q_safe[4], -1.7, 1.318)

        q_safe[5] = max(q_safe[5], 0.0)
        q_safe[6] = max(q_safe[6], 0.0)

        return q_safe
