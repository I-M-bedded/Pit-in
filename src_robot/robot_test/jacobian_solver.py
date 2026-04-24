from enum import Enum, auto
from typing import Optional, Tuple

import numpy as np

from controller import ik


SIDE_CFG = {
    'left': {
        'fk_idx': 0,                # topik.x[0] = Left pin
        'wing_joint': 3,            # qm[3] = Lr
        'z_motor': 5,               # qm[5] = Lz
        'jac_rows': (0, 1),         # J rows: d(Lx)/dq, d(Ly)/dq
        'jac_cols': [0, 1, 2, 3],   # stage X, Y + rtZ + Lr
    },
    'right': {
        'fk_idx': 1,                # topik.x[1] = Right pin
        'wing_joint': 4,            # qm[4] = Rr
        'z_motor': 6,               # qm[6] = Rz
        'jac_rows': (2, 3),         # J rows: d(Rx)/dq, d(Ry)/dq
        'jac_cols': [0, 1, 2, 4],   # stage X, Y + rtZ + Rr
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
        idx = SIDE_CFG[side]['fk_idx']
        return np.array([self.topik.x[idx][0], self.topik.x[idx][1]])

    def fk_dual(self, qm: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        self._sync_topik(qm)
        pin_l = np.array(self.topik.x[0][:2], dtype=float)
        pin_r = np.array(self.topik.x[1][:2], dtype=float)
        return pin_l, pin_r

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    def mode_decompose(self, pin_l: np.ndarray, pin_r: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Convert dual-pin coordinates into centroid/differential coordinates."""
        pin_l = np.asarray(pin_l[:2], dtype=float)
        pin_r = np.asarray(pin_r[:2], dtype=float)
        centroid = 0.5 * (pin_l + pin_r)
        differential = pin_r - pin_l
        return centroid, differential

    def mode_compose(self, centroid: np.ndarray, differential: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Convert centroid/differential coordinates back to dual-pin targets."""
        centroid = np.asarray(centroid[:2], dtype=float)
        differential = np.asarray(differential[:2], dtype=float)
        pin_l = centroid - 0.5 * differential
        pin_r = centroid + 0.5 * differential
        return pin_l, pin_r

    def modal_fk(self, qm: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Return centroid/differential coordinates for the given pose."""
        pin_l, pin_r = self.fk_dual(qm)
        return self.mode_decompose(pin_l, pin_r)

    def fk_lpin(self, qm: np.ndarray) -> np.ndarray:
        return self.fk_pin(qm, 'left')

    def jacobian_4dof(self, qm: np.ndarray, side: str = 'left') -> np.ndarray:
        self.topik.qm = qm.copy()
        self.topik.get_J()
        j_full = self.topik.J

        r0, r1 = SIDE_CFG[side]['jac_rows']
        j_pin = np.vstack([j_full[r0, :], j_full[r1, :]])
        cols = SIDE_CFG[side]['jac_cols']
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
        wi = SIDE_CFG[side]['wing_joint']
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
        """Deprecated: legacy symmetric dual-pin IK branch used for loading flows."""
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

    def solve_arm_for_d(self, d_target: np.ndarray, q2: float):
        """Closed-form wing solve for a fixed base angle q2."""
        d_target = np.asarray(d_target[:2], dtype=float)
        shoulder = 2.0 * self.d2[1] * np.array([np.cos(q2), np.sin(q2)], dtype=float)
        wing_fill = d_target - shoulder
        wing_norm = float(np.linalg.norm(wing_fill))
        max_wing_span = 2.0 * self.d3[1]
        if wing_norm > max_wing_span + 1e-9:
            return []

        half = 0.5 * wing_norm
        perp_sq = max(0.0, self.d3[1] ** 2 - half ** 2)
        perp_len = float(np.sqrt(perp_sq))

        if wing_norm > 1e-9:
            wing_hat = wing_fill / wing_norm
            n_hat = np.array([-wing_hat[1], wing_hat[0]], dtype=float)
        else:
            arm_hat = np.array([np.cos(q2), np.sin(q2)], dtype=float)
            n_hat = np.array([-arm_hat[1], arm_hat[0]], dtype=float)

        candidates = []
        for sign, branch_name in ((+1.0, 'elbow_up'), (-1.0, 'elbow_down')):
            mu = sign * perp_len * n_hat
            w_l = mu - 0.5 * wing_fill
            w_r = mu + 0.5 * wing_fill
            qm3 = self._wrap_angle(np.arctan2(-w_l[1], -w_l[0]) - q2)
            qm4 = self._wrap_angle(np.arctan2(w_r[1], w_r[0]) - q2)
            candidates.append({
                'branch': branch_name,
                'qm3': float(qm3),
                'qm4': float(qm4),
                'wing_fill': wing_fill.copy(),
                'wing_norm': wing_norm,
            })
        return candidates

    def solve_macro_differential(self, d_target: np.ndarray, q_current: np.ndarray, hw, cfg,
                                 c_target: Optional[np.ndarray] = None):
        """Solve the asymmetric macro arm pose for a target differential vector."""
        d_target = np.asarray(d_target[:2], dtype=float)
        q_current = np.asarray(q_current, dtype=float).copy()
        c_target = None if c_target is None else np.asarray(c_target[:2], dtype=float)

        if float(np.linalg.norm(d_target)) < 1e-9:
            q2_seed = float(q_current[2])
        else:
            q2_seed = float(np.arctan2(d_target[1], d_target[0]))

        q2_range = float(cfg.q2_search_range)
        q2_steps = max(3, int(cfg.q2_search_steps))
        q2_grid = np.linspace(q2_seed - q2_range, q2_seed + q2_range, q2_steps)
        q2_grid = np.unique(np.concatenate([q2_grid, [q_current[2]]]))

        best = None
        for q2 in q2_grid:
            for arm_sol in self.solve_arm_for_d(d_target, float(q2)):
                q_try = q_current.copy()
                q_try[2] = float(q2)
                q_try[3] = arm_sol['qm3']
                q_try[4] = arm_sol['qm4']

                q_limited = hw.apply_safety_limits(q_try)
                if not np.allclose(q_try[:5], q_limited[:5], atol=1e-9):
                    continue

                _, d_fk = self.modal_fk(q_try)
                residual = float(np.linalg.norm(d_target - d_fk))
                c_fk, _ = self.modal_fk(q_try)
                stage_delta = None
                stage_ok = True
                stage_norm = 0.0
                if c_target is not None:
                    stage_delta = c_target - c_fk
                    q_stage = q_try.copy()
                    q_stage[0] += stage_delta[0]
                    q_stage[1] -= stage_delta[1]
                    q_stage_safe = hw.apply_safety_limits(q_stage)
                    stage_ok = bool(np.allclose(q_stage[:2], q_stage_safe[:2], atol=1e-9))
                    stage_norm = float(np.linalg.norm(stage_delta))
                cost = (
                    abs(q_try[3]) + abs(q_try[4])
                    + 0.3 * abs(self._wrap_angle(q_try[2] - q_current[2]))
                )
                candidate = {
                    'q2': float(q_try[2]),
                    'qm3': float(q_try[3]),
                    'qm4': float(q_try[4]),
                    'q_cmd': q_try.copy(),
                    'branch': arm_sol['branch'],
                    'd_target': d_target.copy(),
                    'd_achieved': d_fk.copy(),
                    'c_achieved': c_fk.copy(),
                    'stage_delta': None if stage_delta is None else stage_delta.copy(),
                    'stage_ok': stage_ok,
                    'stage_norm': stage_norm,
                    'residual': residual,
                    'cost': float(cost),
                }
                candidate_key = (
                    0 if candidate['stage_ok'] else 1,
                    candidate['stage_norm'],
                    candidate['residual'],
                    candidate['cost'],
                )
                best_key = None if best is None else (
                    0 if best['stage_ok'] else 1,
                    best['stage_norm'],
                    best['residual'],
                    best['cost'],
                )
                if best is None or candidate_key < best_key:
                    best = candidate
        return best

    def check_modal_reachability(self, hw, cfg,
                                 c_target: np.ndarray, d_target: np.ndarray,
                                 q_current: Optional[np.ndarray] = None):
        """Classify reachability in centroid/differential coordinates."""
        info = {}
        c_target = np.asarray(c_target[:2], dtype=float)
        d_target = np.asarray(d_target[:2], dtype=float)
        target_l, target_r = self.mode_compose(c_target, d_target)

        info['c_target'] = c_target.copy()
        info['d_target'] = d_target.copy()
        info['target_l'] = target_l.copy()
        info['target_r'] = target_r.copy()

        diff_norm = float(np.linalg.norm(d_target))
        diff_limit = float(2.0 * self.d2[1] + 2.0 * self.d3[1])
        info['diff_norm'] = diff_norm
        info['diff_limit'] = diff_limit

        q_seed = self.topik.qm.copy() if q_current is None else np.asarray(q_current, dtype=float).copy()
        if diff_norm <= diff_limit + 1e-9:
            macro = self.solve_macro_differential(d_target, q_seed, hw, cfg, c_target=c_target)
            if macro is not None:
                macro_q = macro['q_cmd'].copy()
                c_macro = macro['c_achieved'].copy()
                d_macro = macro['d_achieved'].copy()
                stage_delta = macro['stage_delta'].copy()
                q_stage = macro_q.copy()
                q_stage[0] += stage_delta[0]
                q_stage[1] -= stage_delta[1]
                stage_ok = macro['stage_ok']

                info['macro_q'] = macro_q.copy()
                info['macro_branch'] = macro['branch']
                info['macro_residual'] = macro['residual']
                info['macro_d'] = d_macro.copy()
                info['macro_c'] = c_macro.copy()
                info['stage_delta'] = stage_delta.copy()
                info['stage_q_target'] = q_stage[:2].copy()
                info['stage_ok'] = stage_ok

                if stage_ok:
                    info['q_math'] = q_stage.copy()
                    return ReachMode.BOTH, info

                info['ik_reason'] = 'centroid target exceeds stage travel after macro solve'
            else:
                info['ik_reason'] = 'no feasible asymmetric macro solution found'
        else:
            info['gap_reason'] = (
                f"|d| {diff_norm*1000:.1f}mm exceeds modal limit {diff_limit*1000:.1f}mm"
            )

        l1 = np.hypot(self.topik.d2[0], self.topik.d2[1])
        l2 = self.topik.d3[1]
        r_max = l1 + l2 + cfg.stage_limit

        l_ok = np.hypot(target_l[0] - self.topik.x0, target_l[1] - self.topik.y0) <= r_max
        r_ok = np.hypot(target_r[0] - self.topik.x0, target_r[1] - self.topik.y0) <= r_max
        info['l_reachable'] = l_ok
        info['r_reachable'] = r_ok

        if l_ok and r_ok:
            info['first_side'] = 'left'
            return ReachMode.SEQUENTIAL, info
        if l_ok or r_ok:
            info['reachable_side'] = 'left' if l_ok else 'right'
            return ReachMode.ONE_SIDE, info
        return ReachMode.UNREACHABLE, info

    def check_dual_reachability(self, hw, cfg,
                                target_l: np.ndarray, target_r: np.ndarray,
                                cartype: int = 0):
        """Backward-compatible wrapper around modal reachability classification."""
        c_target, d_target = self.mode_decompose(target_l, target_r)
        return self.check_modal_reachability(
            hw=hw,
            cfg=cfg,
            c_target=c_target,
            d_target=d_target,
            q_current=self.topik.qm.copy(),
        )

    def solve_dual_pbvs_step(self, measured_q: np.ndarray,
                             mean_l: np.ndarray, mean_r: np.ndarray,
                             W_diag: np.ndarray, pbvs_lambda: float,
                             max_stage_step: float, max_arm_step: float) -> np.ndarray:
        """Solve one weighted 4x5 dual-pin PBVS update."""
        e = np.array([mean_l[0], mean_l[1], mean_r[0], mean_r[1]], dtype=float)
        # Save/restore topik.qm so callers that read topik state after this
        # call see the same pose they put in.
        saved_qm = self.topik.qm.copy()
        try:
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
        finally:
            self.topik.qm = saved_qm
            self.topik.fk()
