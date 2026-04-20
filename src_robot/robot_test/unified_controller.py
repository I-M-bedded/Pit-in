from typing import Dict, List, Optional, Tuple

import numpy as np

from robot_test.jacobian_solver import KinematicsSolver, ReachMode
from robot_test.macro_micro import DualSideController, FinishReason, SingleSideController


class TCPcontroller:
    """Top-level PBVS orchestrator that selects and drives worker controllers."""

    FK_VERIFY_RETRIES = 1

    def __init__(self, topik, cfg):
        self.topik = topik
        self.cfg = cfg
        self.kin = KinematicsSolver(topik)
        self.single_ctrl = SingleSideController(topik, cfg)
        self.dual_ctrl = DualSideController(topik, cfg)

        self.mode = ReachMode.UNREACHABLE
        self.mode_info: Dict[str, object] = {}
        self.finish_reason = FinishReason.SUCCESS
        self._targets: Dict[str, np.ndarray] = {}
        self._seq_sides: List[str] = []
        self._seq_idx = 0
        self._verify_retry = 0
        self._active_side: Optional[str] = None
        self._done = True
        self.q_cmd = np.zeros(7)
        self.q_cmd_cnt = [0] * 7

    def set_targets(self, target_l: np.ndarray, target_r: np.ndarray,
                    cartype: int = 0) -> Tuple[ReachMode, dict]:
        """Classify reachability and initialize the corresponding controller."""
        self._targets = {
            'left': np.asarray(target_l[:2], dtype=float),
            'right': np.asarray(target_r[:2], dtype=float),
        }
        self.mode, self.mode_info = self.kin.check_dual_reachability(
            hw=self.dual_ctrl.hw,
            cfg=self.cfg,
            target_l=self._targets['left'],
            target_r=self._targets['right'],
            cartype=cartype,
        )
        self.finish_reason = FinishReason.SUCCESS
        self._seq_sides = []
        self._seq_idx = 0
        self._verify_retry = 0
        self._active_side = None
        self._done = self.mode == ReachMode.UNREACHABLE

        if self.mode == ReachMode.BOTH:
            self.dual_ctrl.set_targets(self._targets['left'], self._targets['right'], cartype)
        elif self.mode == ReachMode.SEQUENTIAL:
            self._seq_sides = ['left', 'right']
            self._start_single_side(self._seq_sides[0], cartype)
        elif self.mode == ReachMode.ONE_SIDE:
            side = self.mode_info['reachable_side']
            self._seq_sides = [side]
            self._start_single_side(side, cartype)
        else:
            self.finish_reason = FinishReason.FAILED_IK

        return self.mode, self.mode_info

    def step(self, world_raw_l: Optional[np.ndarray] = None,
             world_raw_r: Optional[np.ndarray] = None,
             current_c_pos: Optional[np.ndarray] = None) -> list:
        """Run one cycle of the selected PBVS controller."""
        if self._done:
            return self.q_cmd_cnt

        if self.mode == ReachMode.BOTH:
            q_cnt = self.dual_ctrl.step(world_raw_l, world_raw_r, current_c_pos)
            self.q_cmd = self.dual_ctrl.q_cmd.copy()
            self.q_cmd_cnt = list(q_cnt)
            if self.dual_ctrl.is_done:
                self._handle_dual_completion()
            return self.q_cmd_cnt

        if self.mode in (ReachMode.SEQUENTIAL, ReachMode.ONE_SIDE):
            if self._active_side == 'left':
                world_raw = world_raw_l
            else:
                world_raw = world_raw_r
            q_cnt = self.single_ctrl.step(world_raw=world_raw, current_c_pos=current_c_pos)
            self.q_cmd = self.single_ctrl.q_cmd.copy()
            self.q_cmd_cnt = list(q_cnt)
            if self.single_ctrl.is_done:
                self._handle_single_completion()
            return self.q_cmd_cnt

        self.q_cmd_cnt = [0] * 7
        return self.q_cmd_cnt

    @property
    def is_done(self) -> bool:
        return self._done

    @property
    def succeeded(self) -> bool:
        return self.is_done and self.finish_reason == FinishReason.SUCCESS

    def _start_single_side(self, side: str, cartype: int):
        self._active_side = side
        self._verify_retry = 0
        self._done = False
        self.single_ctrl.set_target(self._targets[side], side=side, cartype=cartype)
        self.q_cmd = self.single_ctrl.q_cmd.copy()
        self.q_cmd_cnt = list(self.single_ctrl.q_cmd_cnt)

    def _verify_side_target(self, side: str) -> float:
        pin = self.kin.fk_pin(self.q_cmd, side)
        return float(np.linalg.norm(pin - self._targets[side]))

    def _verify_dual_targets(self) -> Tuple[float, float]:
        pin_l, pin_r = self.kin.fk_dual(self.q_cmd)
        err_l = float(np.linalg.norm(pin_l - self._targets['left']))
        err_r = float(np.linalg.norm(pin_r - self._targets['right']))
        return err_l, err_r

    def _handle_dual_completion(self):
        self.finish_reason = self.dual_ctrl.finish_reason
        if self.finish_reason != FinishReason.SUCCESS:
            self._done = True
            return

        err_l, err_r = self._verify_dual_targets()
        verify_tol = self.cfg.micro_threshold
        if err_l <= verify_tol and err_r <= verify_tol:
            self._done = True
            return
        if self._verify_retry < self.FK_VERIFY_RETRIES:
            self._verify_retry += 1
            print(f"[TCP VERIFY] Dual FK residual too large "
                  f"(L={err_l*1000:.2f}mm, R={err_r*1000:.2f}mm). Retrying controller.")
            self.dual_ctrl.set_targets(self._targets['left'], self._targets['right'], self.topik.cartype)
            self.finish_reason = FinishReason.SUCCESS
            self._done = False
        else:
            self.finish_reason = FinishReason.FAILED_TIMEOUT
            self._done = True

    def _handle_single_completion(self):
        side = self._active_side
        if side is None:
            self.finish_reason = FinishReason.FAILED_TIMEOUT
            self._seq_idx = len(self._seq_sides)
            self._done = True
            return

        err = self._verify_side_target(side)
        verify_tol = self.cfg.micro_threshold
        if err > verify_tol and self._verify_retry < self.FK_VERIFY_RETRIES:
            self._verify_retry += 1
            print(f"[TCP VERIFY] {side} FK residual {err*1000:.2f}mm. Retrying controller.")
            self.single_ctrl.set_target(self._targets[side], side=side, cartype=self.topik.cartype)
            return
        if err > verify_tol:
            self.finish_reason = FinishReason.FAILED_TIMEOUT
            self._seq_idx = len(self._seq_sides)
            self._done = True
            return

        self._seq_idx += 1
        if self._seq_idx >= len(self._seq_sides):
            self.finish_reason = FinishReason.SUCCESS
            self._done = True
            return

        next_side = self._seq_sides[self._seq_idx]
        self._start_single_side(next_side, self.topik.cartype)


UnifiedHybridController = TCPcontroller
