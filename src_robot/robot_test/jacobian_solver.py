from enum import Enum, auto
from typing import Tuple

import numpy as np

from controller import ik


_SIDE_JAC_CFG = {
    'left': {
        'fk_idx': 0,
        'jac_rows': (0, 1),
        'jac_cols': [0, 1, 2, 3],
        'wing_joint': 3,
    },
    'right': {
        'fk_idx': 1,
        'jac_rows': (2, 3),
        'jac_cols': [0, 1, 2, 4],
        'wing_joint': 4,
    },
}


class ReachMode(Enum):
    """Reachability classification for dual-pin targets."""

    BOTH = auto()
    SEQUENTIAL = auto()
    ONE_SIDE = auto()
    UNREACHABLE = auto()


class KinematicsSolver:
    """Kinematics and IK helpers shared by the PBVS controllers."""

    def __init__(self, topik: ik.Topik):
        self.topik = topik
        self.d2 = topik.d2
        self.d3 = topik.d3

    def _sync_topik(self, qm: np.ndarray):
        self.topik.qm = qm.copy()
        self.topik.fk()

    def fk_pin(self, qm: np.ndarray, side: str = 'left') -> np.ndarray:
        self._sync_topik(qm)
        idx = _SIDE_JAC_CFG[side]['fk_idx']
        return np.array([self.topik.x[idx][0], self.topik.x[idx][1]])

    def fk_dual(self, qm: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        self._sync_topik(qm)
        pin_l = np.array(self.topik.x[0][:2], dtype=float)
        pin_r = np.array(self.topik.x[1][:2], dtype=float)
        return pin_l, pin_r

    def fk_lpin(self, qm: np.ndarray) -> np.ndarray:
        return self.fk_pin(qm, 'left')

    def jacobian_4dof(self, qm: np.ndarray, side: str = 'left') -> np.ndarray:
        self.topik.qm = qm.copy()
        self.topik.get_J()
        j_full = self.topik.J

        r0, r1 = _SIDE_JAC_CFG[side]['jac_rows']
        j_pin = np.vstack([j_full[r0, :], j_full[r1, :]])
        cols = _SIDE_JAC_CFG[side]['jac_cols']
        return j_pin[:, cols]

    def weighted_pseudoinverse(self, J: np.ndarray, W_diag: np.ndarray) -> np.ndarray:
        w_inv = np.diag(1.0 / W_diag)
        j_wij_t = J @ w_inv @ J.T
        j_wij_t += np.eye(J.shape[0]) * 1e-10
        return w_inv @ J.T @ np.linalg.inv(j_wij_t)

    def geometry_plan_arm(self, target_xy: np.ndarray, side: str,
                          measured_q: np.ndarray, hw) -> Tuple[float, float, bool, str, list]:
        """Analytical 2-link arm IK with stage offsets held fixed."""
        d2 = self.topik.d2
        d3_1 = self.topik.d3[1]
        wi = _SIDE_JAC_CFG[side]['wing_joint']
        saved_qm = self.topik.qm.copy()

        tx = float(target_xy[0] - measured_q[0] - self.topik.x0)
        ty = float(target_xy[1] + measured_q[1] - self.topik.y0)

        if side == 'left':
            link1_vec = np.array([-d2[1], d2[0]], dtype=float)
            link2_heading_offset = np.pi
        else:
            link1_vec = np.array([d2[1], d2[0]], dtype=float)
            link2_heading_offset = 0.0

        L1 = float(np.linalg.norm(link1_vec))
        L2 = float(d3_1)
        phi1 = float(np.arctan2(link1_vec[1], link1_vec[0]))

        clamped = False
        r = float(np.hypot(tx, ty))
        r_max = L1 + L2
        r_min = abs(L1 - L2)
        if r > r_max:
            clamped = True
            scale = (r_max - 1e-3) / max(r, 1e-9)
            tx *= scale
            ty *= scale
        elif r < r_min:
            clamped = True
            scale = (r_min + 1e-3) / max(r, 1e-9)
            tx *= scale
            ty *= scale

        c2 = (tx * tx + ty * ty - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        c2 = float(np.clip(c2, -1.0, 1.0))
        s2_abs = float(np.sqrt(max(0.0, 1.0 - c2 * c2)))

        current_q2 = float(measured_q[2])
        current_wing = float(measured_q[wi])
        candidates = []

        for branch_idx, s2 in enumerate((s2_abs, -s2_abs)):
            theta2 = float(np.arctan2(s2, c2))
            theta1 = float(np.arctan2(ty, tx) - np.arctan2(L2 * s2, L1 + L2 * c2))

            rtz = theta1 - phi1
            wing = theta2 - link2_heading_offset + phi1

            rtz = (rtz + np.pi) % (2 * np.pi) - np.pi
            wing = (wing + np.pi) % (2 * np.pi) - np.pi

            q_try = measured_q.copy()
            q_try[2] = rtz
            q_try[wi] = wing
            q_limited = hw.apply_safety_limits(q_try)

            clipped = (abs(q_limited[2] - rtz) > 1e-9 or abs(q_limited[wi] - wing) > 1e-9)
            pin_try = self.fk_pin(q_limited, side)
            residual = float(np.linalg.norm(np.asarray(target_xy[:2], dtype=float) - pin_try))
            q2_delta = abs(q_limited[2] - current_q2)
            wing_delta = abs(q_limited[wi] - current_wing)
            delta = q2_delta + wing_delta

            candidates.append({
                'branch': branch_idx,
                'clipped': clipped,
                'residual': residual,
                'q2_delta': q2_delta,
                'wing_delta': wing_delta,
                'delta': delta,
                'rtZ': float(q_limited[2]),
                'wing': float(q_limited[wi]),
            })

        unclipped = [item for item in candidates if not item['clipped']]
        residual_pool = unclipped if unclipped else candidates
        min_residual = min(item['residual'] for item in residual_pool)
        ambiguity_tol = 5e-4
        ambiguous = [
            item for item in residual_pool
            if item['residual'] <= min_residual + ambiguity_tol
        ]
        best = min(
            ambiguous,
            key=lambda item: (
                item['q2_delta'],
                item['wing_delta'],
                item['delta'],
            ),
        )
        self.topik.qm = saved_qm.copy()
        self.topik.fk()
        return best['rtZ'], best['wing'], clamped, f"branch_{best['branch']}", candidates

    def try_dual_ik_branch(self, hw, cfg, tl: np.ndarray, tr: np.ndarray,
                           cartype: int, flip_wings: bool):
        """Try one simultaneous dual-pin IK branch."""
        d2, d3_1 = self.topik.d2, self.topik.d3[1]
        gap = float(np.linalg.norm(tl - tr))
        q3 = self.topik.cal_wing_angle(gap)

        if cartype == 3:
            qm3, qm4 = q3, -q3
        else:
            qm3, qm4 = -q3, q3
        if flip_wings:
            qm3, qm4 = -qm3, -qm4

        denom = 2.0 * d2[1] + 2.0 * d3_1 * np.cos(qm3)
        if abs(denom) < 1e-9:
            return None

        sq2 = (tr[1] - tl[1]) / denom
        cq2 = (tr[0] - tl[0]) / denom
        q2 = float(np.arctan2(sq2, cq2))

        q0 = tl[0] + cq2 * (d2[1] + d3_1 * np.cos(qm3)) + sq2 * (d2[0] - d3_1 * np.sin(qm3)) - self.topik.x0
        q1 = -tl[1] - sq2 * (d2[1] + d3_1 * np.cos(qm3)) + cq2 * (d2[0] - d3_1 * np.sin(qm3)) + self.topik.y0

        q_math = np.array([q0, q1, q2, qm3, qm4, 0.0, 0.0])
        q_safe = hw.apply_safety_limits(q_math)

        stage_ok = abs(q_safe[0]) <= cfg.stage_limit and abs(q_safe[1]) <= cfg.stage_limit
        joints_ok = np.allclose(q_math[:5], q_safe[:5], atol=1e-4)
        if stage_ok and joints_ok:
            return q_math
        return None

    def check_dual_reachability(self, hw, cfg,
                                target_l: np.ndarray, target_r: np.ndarray,
                                cartype: int = 0):
        """Classify reachability for dual-pin PBVS."""
        info = {}
        tl = np.asarray(target_l[:2], dtype=float)
        tr = np.asarray(target_r[:2], dtype=float)

        gap = float(np.linalg.norm(tl - tr))
        info['gap'] = gap

        if self.topik.dismin <= gap <= self.topik.dismax:
            for flip in (False, True):
                q_sol = self.try_dual_ik_branch(hw, cfg, tl, tr, cartype, flip)
                if q_sol is not None:
                    info['q_math'] = q_sol.copy()
                    info['branch'] = 'flipped' if flip else 'default'
                    return ReachMode.BOTH, info
            info['ik_reason'] = 'both IK branches exceed joint/stage limits'
        else:
            info['gap_reason'] = (
                f"gap {gap*1000:.1f}mm not in "
                f"[{self.topik.dismin*1000:.1f}, {self.topik.dismax*1000:.1f}]mm"
            )

        l1 = np.hypot(self.topik.d2[0], self.topik.d2[1])
        l2 = self.topik.d3[1]
        r_max = l1 + l2 + cfg.stage_limit

        l_ok = np.hypot(tl[0] - self.topik.x0, tl[1] - self.topik.y0) <= r_max
        r_ok = np.hypot(tr[0] - self.topik.x0, tr[1] - self.topik.y0) <= r_max
        info['l_reachable'] = l_ok
        info['r_reachable'] = r_ok

        if l_ok and r_ok:
            info['first_side'] = 'left'
            return ReachMode.SEQUENTIAL, info
        if l_ok or r_ok:
            info['reachable_side'] = 'left' if l_ok else 'right'
            return ReachMode.ONE_SIDE, info

        return ReachMode.UNREACHABLE, info

    def solve_dual_pbvs_step(self, measured_q: np.ndarray,
                             mean_l: np.ndarray, mean_r: np.ndarray,
                             W_diag: np.ndarray, pbvs_lambda: float,
                             max_stage_step: float, max_arm_step: float) -> np.ndarray:
        """Solve one weighted 4x5 dual-pin PBVS update."""
        e = np.array([mean_l[0], mean_l[1], mean_r[0], mean_r[1]], dtype=float)
        self.topik.qm = measured_q.copy()
        self.topik.get_J()
        J = self.topik.J[:, :5]

        j_winv = self.weighted_pseudoinverse(J, W_diag)
        dq = j_winv @ (pbvs_lambda * e)

        s_norm = np.linalg.norm(dq[:2])
        if s_norm > max_stage_step:
            dq[:2] *= max_stage_step / s_norm
        for i in range(2, 5):
            dq[i] = np.clip(dq[i], -max_arm_step, max_arm_step)
        return dq
