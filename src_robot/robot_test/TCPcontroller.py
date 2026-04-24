from typing import Dict, List, Optional, Tuple

import numpy as np

from robot_test.hardware import HardwareInterface
from robot_test.jacobian_solver import KinematicsSolver, ReachMode
from robot_test.macro_micro import (
    ControlConfig,
    DualSideController,
    FinishReason,
    SingleSideController,
)


class TCPcontroller:
    """Top-level PBVS orchestrator that selects and drives worker controllers.

    robot_main only needs this class: feed dual-pin targets via
    :meth:`set_targets` (auto-classified via reachability) or force a
    single-side run via :meth:`set_single_target`.
    """

    FK_VERIFY_RETRIES = 1

    def __init__(self, topik, cfg: ControlConfig):
        self.topik = topik
        self.cfg = cfg
        self.hw = HardwareInterface(topik, cfg)
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
        self._cartype: int = 0
        self._done = True
        self.q_cmd = np.zeros(7)
        self.q_cmd_cnt = [0] * 7

    # ---- Target setup ----

    def set_targets(self, target_l: np.ndarray, target_r: np.ndarray,
                    cartype: int = 0) -> Tuple[ReachMode, dict]:
        """Classify reachability and initialize the corresponding controller."""
        self._targets = {
            'left': np.asarray(target_l[:2], dtype=float),
            'right': np.asarray(target_r[:2], dtype=float),
        }
        c_target, d_target = self.kin.mode_decompose(self._targets['left'], self._targets['right'])
        self._cartype = int(cartype)
        self.mode, self.mode_info = self.kin.check_modal_reachability(
            hw=self.hw,
            cfg=self.cfg,
            c_target=c_target,
            d_target=d_target,
            q_current=self.topik.qm.copy(),
        )
        self.finish_reason = FinishReason.SUCCESS
        self._seq_sides = []
        self._seq_idx = 0
        self._verify_retry = 0
        self._active_side = None
        self._done = self.mode == ReachMode.UNREACHABLE

        if self.mode == ReachMode.BOTH:
            self.dual_ctrl.set_modal_targets(
                c_target=c_target,
                d_target=d_target,
                target_l=self._targets['left'],
                target_r=self._targets['right'],
                cartype=self._cartype,
            )
        elif self.mode == ReachMode.SEQUENTIAL:
            self._seq_sides = ['left', 'right']
            self._start_single_side(self._seq_sides[0])
        elif self.mode == ReachMode.ONE_SIDE:
            side = self.mode_info['reachable_side']
            self._seq_sides = [side]
            self._start_single_side(side)
        else:
            self.finish_reason = FinishReason.FAILED_IK

        return self.mode, self.mode_info

    def set_single_target(self, side: str, target_xy: np.ndarray,
                          cartype: int = 0) -> Tuple[ReachMode, dict]:
        """Force a single-side run (used by PbvsLTask / PbvsRTask).

        Bypasses dual-pin reachability classification and routes directly
        through the single-side worker as ``ReachMode.ONE_SIDE``.
        """
        if side not in ('left', 'right'):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")

        target_xy = np.asarray(target_xy[:2], dtype=float)
        self._targets = {side: target_xy}
        self._cartype = int(cartype)
        self.mode = ReachMode.ONE_SIDE
        self.mode_info = {'reachable_side': side, 'forced': True}
        self.finish_reason = FinishReason.SUCCESS
        self._seq_sides = [side]
        self._seq_idx = 0
        self._verify_retry = 0
        self._done = False
        self._start_single_side(side)
        return self.mode, self.mode_info

    # ---- Step ----

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
            world_raw = world_raw_l if self._active_side == 'left' else world_raw_r
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

    # ---- Internal sequencing ----

    def _start_single_side(self, side: str):
        self._active_side = side
        self._verify_retry = 0
        self._done = False
        self.single_ctrl.set_target(self._targets[side], side=side, cartype=self._cartype)
        self.q_cmd = self.single_ctrl.q_cmd.copy()
        self.q_cmd_cnt = list(self.single_ctrl.q_cmd_cnt)

    def _verify_side_target(self, side: str) -> float:
        pin = self.kin.fk_pin(self.q_cmd, side)
        return float(np.linalg.norm(pin - self._targets[side]))

    def _verify_dual_targets(self) -> Tuple[float, float]:
        pin_l, pin_r = self.kin.fk_dual(self.q_cmd)
        c_now, d_now = self.kin.mode_decompose(pin_l, pin_r)
        c_target = self.dual_ctrl.current_c_target
        d_target = self.dual_ctrl.current_d_target
        err_c = float(np.linalg.norm(c_now - c_target))
        err_d = float(np.linalg.norm(d_now - d_target))
        return err_c, err_d

    def _handle_dual_completion(self):
        self.finish_reason = self.dual_ctrl.finish_reason
        if self.finish_reason != FinishReason.SUCCESS:
            self._done = True
            return

        err_c, err_d = self._verify_dual_targets()
        if err_c <= self.cfg.micro_threshold and err_d <= self.cfg.diff_deadband:
            self._done = True
            return
        if self._verify_retry < self.FK_VERIFY_RETRIES:
            self._verify_retry += 1
            c_target = self.dual_ctrl.current_c_target
            d_target = self.dual_ctrl.current_d_target
            print(f"[TCP VERIFY] Dual modal residual too large "
                  f"(|e_c|={err_c*1000:.2f}mm, |e_d|={err_d*1000:.2f}mm). Retrying controller.")
            self.dual_ctrl.set_modal_targets(
                c_target=c_target,
                d_target=d_target,
                cartype=self._cartype,
            )
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
            self.single_ctrl.set_target(self._targets[side], side=side, cartype=self._cartype)
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
        self._start_single_side(next_side)
