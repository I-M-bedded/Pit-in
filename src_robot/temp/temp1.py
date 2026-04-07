"""
Hybrid PBVS Control System  (q2-Locked Variant)
=============================================================
4-DOF system: 2-DOF Manipulator (Macro) + 2-DOF XY Stage (Micro)
Right-handed coordinate: X=forward, Y=left, Z=up

q2 (rtZ, base rotation) Strategy
----------------------------------
  q2 starts LOCKED at 0.  Only Lr (wing angle) is solved analytically.
  The XY stage absorbs any remaining 2-D residual.

  q2 is activated ONLY when the arm-only residual exceeds stage_limit,
  and is always clamped to ±Q2_MAX (default ±15 °).

  Activation flow
  ──────────────
  Step 1  q2=0  →  solve optimal Lr  →  compute residual
  Step 2  if residual ≤ stage_limit   →  stay locked, stage compensates
  Step 3  else                        →  full 2-link IK for optimal q2
                                         clamp q2 to ±Q2_MAX
                                         re-solve Lr for clamped q2
  Step 4  report final residual

State Machine:
    MACRO_ARM → MICRO_STAGE → VS_LIFT → DONE

Motor mapping (7-servo system):
    [0] trY  : Y-translation (Micro-StageX_fwd)  → qm[0]
    [1] trX  : X-translation (Micro-StageY_left) → qm[1]
    [2] rtZ  : Z-rotation    (Macro-Arm base)     → qm[2]   ← q2
    [3] Lr   : L-wing rot    (Macro-Arm link)     → qm[3]
    [4] Rr   : R-wing rot    (Macro-Arm link)     → qm[4]
    [5] Lz   : L-pin height                       → qm[5]
    [6] Rz   : R-pin height                       → qm[6]
"""

import numpy as np
import threading        
import time
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from controller import ik


# =============================================================================
# Constants
# =============================================================================
class State(Enum):
    MACRO_ARM        = auto()
    MICRO_STAGE      = auto()
    VS_LIFT          = auto()
    DONE             = auto()


@dataclass
class ControlConfig:
    """Control parameters for Macro-Micro PBVS system (q2-Locked variant)."""
    version: int = 4

    enable_macro: bool = True
    enable_micro: bool = True

    pbvs_lambda: float = 0.5
    dt: float = 0.1

    macro_threshold: float = 0.005
    micro_threshold: float = 0.001
    pbvs_threshold:  float = 0.0003

    micro_settling_time: float = 0.5
    stage_limit: float = 0.07
    q2_max_deg: float = 15.0

    w_arm:   float = 1000.0
    w_stage: float = 1.0

    ema_alpha: float = 0.3

    max_arm_vel:   float = 0.5
    max_stage_vel: float = 0.05

    t_vel: List[int] = field(default_factory=lambda: [10000, 10000, 2000, 600, 600, 50000, 50000])

    @property
    def q2_max_rad(self) -> float:
        return np.deg2rad(self.q2_max_deg)


# =============================================================================
# Module A: VisionFeedback
# =============================================================================
class VisionFeedback:
    """ArUco marker / target 3-D pose estimation with EMA low-pass filter.

    Thread-safety
    -------------
    update_error() / reset() 비전 스레드
    error_norm / _filtered_err 제어 스레드
    두 접근을 _lock 으로 보호
    """

    def __init__(self, topik: ik.Topik, alpha: float = 0.3):
        self.topik = topik
        self.alpha = alpha
        self._filtered_err = np.zeros(2)
        self._initialized = False
        self._lock = threading.Lock()   

        self._R_lcam_mount = np.array([[ 0.0,  1.0, 0.0],
                                       [-1.0,  0.0, 0.0],
                                       [ 0.0,  0.0, 1.0]])
        self._offset_lcam = np.array([-0.057, 0.029, 0.0])

    def get_pin_error(self, cam_err: np.ndarray, side: str = 'left') -> np.ndarray:
        hole_local = self._offset_lcam + self._R_lcam_mount @ np.asarray(cam_err)
        R_world_pin = self.topik.so3_lcam
        hole_world_offset = np.asarray(R_world_pin @ hole_local).flatten()
        return hole_world_offset[:2]

    def update_error(self, raw_pin_err: np.ndarray) -> np.ndarray:
        """비전 스레드에서 호출. EMA 갱신 후 현재 필터값을 반환"""
        with self._lock:                
            if not self._initialized:
                self._filtered_err = raw_pin_err.copy()
                self._initialized = True
            else:
                self._filtered_err = (self.alpha * raw_pin_err
                                      + (1.0 - self.alpha) * self._filtered_err)
            return self._filtered_err.copy()

    def reset(self):
        """제어 스레드(set_target) 또는 비전 스레드에서 호출 가능"""
        with self._lock:             
            self._filtered_err = np.zeros(2)
            self._initialized = False

    @property
    def error_norm(self) -> float:
        with self._lock:             
            return float(np.linalg.norm(self._filtered_err))

    def get_filtered_err(self) -> np.ndarray:
        """현재 필터된 에러를 스레드-안전하게 복사 반환"""
        with self._lock:            
            return self._filtered_err.copy()


# =============================================================================
# Module B: KinematicsSolver
# =============================================================================
class KinematicsSolver:
    """Forward Kinematics and Jacobian for the 4-DOF hybrid system.

    Thread-safety
    -------------
    fk_lpin / jacobian_4dof 는 Topik 인스턴스의 내부 상태(qm, J)를
    직접 덮어쓰기. 여러 스레드에서 동시에 호출하면 결과가 오염.
    호출 측(HybridController)이 _ctrl_lock 으로 보호.
    """

    def __init__(self, topik: ik.Topik):
        self.topik = topik
        self.d2 = topik.d2
        self.d3 = topik.d3

    def fk_lpin(self, qm: np.ndarray) -> np.ndarray:
        self.topik.qm = qm.copy()
        self.topik.fk()
        return np.array([self.topik.x[0][0], self.topik.x[0][1]])

    def jacobian_4dof(self, qm: np.ndarray) -> np.ndarray:
        self.topik.qm = qm.copy()
        self.topik.get_J()
        J_full = self.topik.J
        J_lpin = np.vstack([J_full[0, :], J_full[1, :]])
        return J_lpin[:, [0, 1, 2, 3]]

    def wing_center_lpin(self, q2: float) -> Tuple[float, float]:
        d2 = self.d2
        wx = -np.cos(q2) * d2[1] + np.sin(q2) * d2[0]
        wy = -np.sin(q2) * d2[1] - np.cos(q2) * d2[0]
        return wx, wy

    def lpin_from_q2_Lr(self, q2: float, Lr: float) -> Tuple[float, float]:
        d3 = self.d3[1]
        wx, wy = self.wing_center_lpin(q2)
        total = q2 + Lr
        px = wx - np.cos(total) * d3
        py = wy - np.sin(total) * d3
        return px, py

    def solve_Lr_for_q2(self, q2: float, tx: float, ty: float) -> float:
        wx, wy = self.wing_center_lpin(q2)
        dx = tx - wx
        dy = ty - wy
        total = np.arctan2(-dy, -dx)
        return total - q2

    def residual_arm_only(self, q2: float, Lr: float,
                          tx: float, ty: float) -> float:
        px, py = self.lpin_from_q2_Lr(q2, Lr)
        return float(np.hypot(tx - px, ty - py))

    def weighted_pseudoinverse(self, J: np.ndarray,
                                W_diag: np.ndarray) -> np.ndarray:
        W_inv = np.diag(1.0 / W_diag)
        JWiJt = J @ W_inv @ J.T + np.eye(2) * 1e-10
        return W_inv @ J.T @ np.linalg.inv(JWiJt)


# =============================================================================
# Module C: HybridController
# =============================================================================
class HybridController:
    """Macro-Micro hybrid PBVS state-machine controller (q2-Locked variant).

    Thread-safety
    -------------
    공유 상태(q_cmd, state, target_xy, _q2_activated, _settle_timer)는
    모두 _ctrl_lock 으로 보호.

    외부 스레드 사용 패턴
    ─────────────────────
      # 비전 스레드
      ctrl.push_vision(raw_pin_err)   # lock-free; VisionFeedback 내부 lock

      # 제어 스레드
      ctrl.step()                     # _ctrl_lock 획득 후 전체 스텝 실행

      # 메인/UI 스레드
      ctrl.set_target(xy)             # _ctrl_lock 획득 후 상태 초기화
      cnt = ctrl.get_q_cnt()          # _ctrl_lock 획득 후 q_cmd 스냅샷 반환
      done = ctrl.is_done             # 원자적 bool 읽기 (GIL 보호)
    """

    def __init__(self, topik: ik.Topik, cfg: ControlConfig):
        self.topik = topik
        self.cfg = cfg
        self.kin = KinematicsSolver(topik)
        self.vision = VisionFeedback(topik=topik, alpha=cfg.ema_alpha)
        self.hw = HardwareInterface(topik, cfg)

        self._ctrl_lock = threading.Lock()  

        self.state = State.MACRO_ARM
        self._settle_timer: float = 0.0
        self._micro_step_count: int = 0     

        self.q_cmd = np.zeros(7)
        self.target_xy = np.zeros(2)

        self.W_diag = np.array([cfg.w_stage, cfg.w_stage,
                                cfg.w_arm,   cfg.w_arm])

        self._q2_activated: bool = False

    # -------------------------------------------------------------------------
    # Public thread-safe API
    # -------------------------------------------------------------------------

    def set_target(self, target_xy: np.ndarray, cartype: int = 0):
        """메인/UI 스레드에서 안전하게 타깃을 교체"""
        with self._ctrl_lock:              
            self.target_xy = np.asarray(target_xy[:2], dtype=float)
            self.topik.cartype = cartype
            self.state = State.MACRO_ARM
            self._settle_timer = 0.0
            self._micro_step_count = 0
            self._q2_activated = False
        self.vision.reset()                 # vision lock이 따로 있으므로 ctrl_lock 밖에서 호출

    def push_vision(self, raw_pin_err: np.ndarray):
        """비전 스레드에서 호출. 제어 락 없이 EMA만 갱신"""
        self.vision.update_error(raw_pin_err)

    def step(self) -> list:
        """제어 스레드에서 호출. 1 스텝 실행 후 엔코더 카운트 반환"""
        with self._ctrl_lock:             
            if self.state == State.MACRO_ARM:
                if self.cfg.enable_macro:
                    self._step_macro()
                else:
                    print("[SKIP] Phase 1 MACRO_ARM disabled by config.")
                    self.state = State.MICRO_STAGE

            elif self.state == State.MICRO_STAGE:
                if self.cfg.enable_micro:
                    # VisionFeedback에서 스냅샷을 가져와 사용
                    current_err = self.vision.get_filtered_err()
                    self._step_micro(current_err)
                else:
                    print("[SKIP] Phase 2 MICRO_STAGE disabled by config.")
                    self.state = State.VS_LIFT

            elif self.state == State.VS_LIFT:
                current_err = self.vision.get_filtered_err()
                self._step_vs_lift(current_err)

            elif self.state == State.DONE:
                pass

            return self._q_cmd_to_cnt()

    def get_q_cnt(self) -> list:
        """현재 q_cmd를 엔코더 카운트로 변환해 스레드-안전하게 반환."""
        with self._ctrl_lock:               
            return self._q_cmd_to_cnt()

    # -------------------------------------------------------------------------
    # q2-Locked Geometry Planner  (이하 내부 메서드 — _ctrl_lock 보유 중 호출)
    # -------------------------------------------------------------------------

    def _geometry_plan_arm_q2_locked(
            self, target_xy: np.ndarray
    ) -> Tuple[float, float, bool, float]:
        tx, ty = float(target_xy[0]), float(target_xy[1])
        stage_lim = self.cfg.stage_limit
        q2_max = self.cfg.q2_max_rad

        q2 = 0.0
        Lr = self.kin.solve_Lr_for_q2(q2, tx, ty)
        residual = self.kin.residual_arm_only(q2, Lr, tx, ty)

        if residual <= stage_lim:
            return q2, Lr, False, residual

        q2_opt, Lr_opt, _ = self._geometry_plan_arm_full(target_xy)
        q2_clamped = float(np.clip(q2_opt, -q2_max, q2_max))
        Lr_clamped = self.kin.solve_Lr_for_q2(q2_clamped, tx, ty)
        residual_clamped = self.kin.residual_arm_only(q2_clamped, Lr_clamped, tx, ty)

        activated = abs(q2_clamped) > 1e-9  # ← 실제로 0이면 False
        return q2_clamped, Lr_clamped, activated, residual_clamped

    def _geometry_plan_arm_full(
            self, target_xy: np.ndarray
    ) -> Tuple[float, float, bool]:
        d2 = self.topik.d2
        d3_1 = self.topik.d3[1]

        L1 = np.sqrt(d2[0]**2 + d2[1]**2)
        L2 = d3_1
        phi_offset = np.arctan2(-d2[0], -d2[1])

        tx, ty = target_xy[0], target_xy[1]
        r = np.sqrt(tx**2 + ty**2)
        clamped = False

        r_max = L1 + L2
        r_min = abs(L1 - L2)
        if r > r_max:
            clamped = True
            r = r_max - 0.001
        elif r < r_min:
            clamped = True
            r = r_min + 0.001

        cos_alpha = np.clip((L1**2 + L2**2 - r**2) / (2 * L1 * L2), -1.0, 1.0)
        alpha = np.arccos(cos_alpha)

        theta_target = np.arctan2(ty, tx)
        cos_beta = np.clip((L1**2 + r**2 - L2**2) / (2 * L1 * r), -1.0, 1.0)
        beta = np.arccos(cos_beta)

        rtZ = theta_target - beta - phi_offset
        Lr  = -(np.pi - alpha)

        return rtZ, Lr, clamped

    # -------------------------------------------------------------------------
    # State handlers  (모두 _ctrl_lock 보유 중 호출됨)
    # -------------------------------------------------------------------------

    def _step_macro(self):
        target = self.target_xy.copy()
        tx, ty = target

        print("\n" + "=" * 60)
        print("[PHASE 1] MACRO_ARM — q2-Locked Geometry Planner")
        print("=" * 60)

        try:
            rtZ, Lr, q2_activated, residual = \
                self._geometry_plan_arm_q2_locked(target)

            required_q = np.zeros(7)
            required_q[2] = rtZ
            required_q[3] = Lr
            required_q = self.hw.apply_safety_limits(required_q)

            lpin_reach = self.kin.fk_lpin(required_q)
            fk_err = np.linalg.norm(target - lpin_reach)

            q2_deg = np.rad2deg(required_q[2])
            Lr_deg = np.rad2deg(required_q[3])

            print(f"  Target        : ({tx:.4f}, {ty:.4f}) m")
            print(f"  q2 (rtZ)      : {q2_deg:+.2f} deg  "
                  f"{'[ACTIVATED]' if q2_activated else '[LOCKED q2=0]'}")
            print(f"  Lr  (wing)    : {Lr_deg:+.2f} deg")
            print(f"  Stage         : LOCKED (0.0, 0.0) m")
            print(f"  Arm residual  : {residual*1000:.2f} mm "
                  f"(stage_limit={self.cfg.stage_limit*1000:.0f} mm)")
            print(f"  FK Verify     : ({lpin_reach[0]:.4f}, {lpin_reach[1]:.4f}) m")
            print(f"  FK Error      : {fk_err*1000:.2f} mm")

            if q2_activated:
                q2_max_deg = self.cfg.q2_max_deg
                if abs(np.rad2deg(rtZ)) >= q2_max_deg - 0.01:
                    print(f"  [q2 CLAMPED]  Hit ±{q2_max_deg:.0f}° limit.")
                else:
                    print(f"  [q2 ACTIVATED] q2={q2_deg:+.2f}° "
                          f"(within ±{q2_max_deg:.0f}°)")

            if residual > self.cfg.stage_limit:
                print(f"  [WARNING] Arm residual {residual*1000:.1f} mm exceeds "
                      f"stage limit {self.cfg.stage_limit*1000:.0f} mm.")
                if residual > self.cfg.stage_limit * 1.5:
                    print(f"  [ABORT] Residual too large. Moving to DONE.")
                    self.state = State.DONE
                    return

            self.q_cmd = required_q.copy()
            self._q2_activated = q2_activated

            print(f"\n  [TRANSITION] MACRO_ARM → MICRO_STAGE")
            self.state = State.MICRO_STAGE
            self._settle_timer = 0.0
            self.vision.reset()

        except Exception as e:
            print(f"  [ERROR] Geometry planner failed: {e}")
            import traceback; traceback.print_exc()
            self.state = State.DONE

    def _step_micro(self, current_err: np.ndarray):
        """Phase 2: MICRO_STAGE — Stage 2DOF PBVS.

        current_err: VisionFeedback에서 이미 EMA된 스냅샷 (2D array)
        """
        err_norm = float(np.linalg.norm(current_err))

        if err_norm < self.cfg.pbvs_threshold:
            self._settle_timer += self.cfg.dt
            if self._settle_timer > self.cfg.micro_settling_time:
                print(f"\n{'='*60}")
                print(f"[PHASE 2] MICRO_STAGE — CONVERGED!")
                print(f"  Final Error  : {err_norm*1000:.3f} mm")
                print(f"  Stage X      : {self.q_cmd[0]*1000:.2f} mm")
                print(f"  Stage Y      : {self.q_cmd[1]*1000:.2f} mm")
                print(f"  Settle Time  : {self._settle_timer:.2f} sec")
                print(f"{'='*60}")
                print(f"  [TRANSITION] MICRO_STAGE → VS_LIFT")
                self.state = State.VS_LIFT
                self._settle_timer = 0.0
            return
        else:
            self._settle_timer = 0.0

        q_dot_stage_target = self.cfg.pbvs_lambda * current_err
        J = self.kin.jacobian_4dof(self.q_cmd)
        J_stage = J[:, 0:2]

        try:
            J_stage_inv = np.linalg.inv(J_stage)
            dq_stage = J_stage_inv @ q_dot_stage_target

            speed = np.linalg.norm(dq_stage)
            if speed > self.cfg.max_stage_vel:
                dq_stage *= self.cfg.max_stage_vel / speed

            self.q_cmd[0] += dq_stage[0] * self.cfg.dt
            self.q_cmd[1] += dq_stage[1] * self.cfg.dt
            self.q_cmd = self.hw.apply_safety_limits(self.q_cmd)

            self._micro_step_count += 1
            if self._micro_step_count % 10 == 1:
                print(f"  [MICRO #{self._micro_step_count:03d}] err={err_norm*1000:.2f}mm  "
                      f"stgX={self.q_cmd[0]*1000:.1f}mm  stgY={self.q_cmd[1]*1000:.1f}mm")

        except np.linalg.LinAlgError:
            print("[ERROR] Stage Jacobian inversion failed in Micro!")

    def _step_vs_lift(self, current_vision_xy: np.ndarray):
        print("\n[PHASE 3] VS_LIFT: Pin-Hole aligned. Shutting down PBVS.")
        self.state = State.DONE

    # -------------------------------------------------------------------------
    # Utility
    # -------------------------------------------------------------------------

    def _q_cmd_to_cnt(self) -> list:
        """_ctrl_lock 보유 중에만 호출할 것."""
        return [int(np.round(self.q_cmd[i] * self.topik.m2cnt[i]))
                for i in range(7)]

    @property
    def is_done(self) -> bool:
        # bool 읽기는 GIL이 보호 — 별도 lock 불필요
        return self.state == State.DONE

    @property
    def q2_activated(self) -> bool:
        return self._q2_activated


# =============================================================================
# Module D: HardwareInterface
# =============================================================================
class HardwareInterface:
    """Converts q_cmd (rad, m) ↔ encoder counts and enforces safety limits.

    Thread-safety
    -------------
    순수 연산만 수행하며 내부 가변 상태가 없으므로 별도 lock 불필요.
    """

    def __init__(self, topik: ik.Topik, cfg: ControlConfig):
        self.topik = topik
        self.cfg = cfg
        self.m2cnt = topik.m2cnt.copy()
        self.cnt2m = topik.cnt2m.copy()

    def q_to_cnt(self, q_cmd: np.ndarray) -> List[int]:
        return [int(np.round(q_cmd[i] * self.m2cnt[i])) for i in range(7)]

    def cnt_to_q(self, cnt: np.ndarray) -> np.ndarray:
        return np.array([cnt[i] * self.cnt2m[i] for i in range(7)])

    def get_velocity_profile(self, q_dot: np.ndarray) -> List[int]:
        vel = np.array([abs(q_dot[i] * self.m2cnt[i]) for i in range(7)])
        vel = np.clip(vel, 100, None)
        for i in range(7):
            vel[i] = min(vel[i], self.cfg.t_vel[i])
        return [int(v) for v in vel]

    def apply_safety_limits(self, q_cmd: np.ndarray) -> np.ndarray:
        q_safe = q_cmd.copy()
        sl = self.cfg.stage_limit
        q_safe[0] = np.clip(q_safe[0], -sl,    sl)
        q_safe[1] = np.clip(q_safe[1], -sl,    sl)
        q2_lim = self.cfg.q2_max_rad
        q_safe[2] = np.clip(q_safe[2], -q2_lim, q2_lim)
        q_safe[3] = np.clip(q_safe[3], -1.7,  1.025)
        q_safe[4] = np.clip(q_safe[4], -1.7,  1.318)
        q_safe[5] = max(q_safe[5], 0.0)
        q_safe[6] = max(q_safe[6], 0.0)
        return q_safe


# =============================================================================
# Simulation & Test
# =============================================================================
def simulate_pbvs():
    """멀티스레드 시뮬레이션: 비전 스레드 / 제어 스레드 분리."""
    print("=" * 60)
    print("Macro-Micro Hybrid PBVS Simulation  (q2-Locked, Thread-Safe)")
    print("=" * 60)

    cfg = ControlConfig(version=4)
    topik = ik.Topik(cfg.version)
    hw = HardwareInterface(topik, cfg)
    ctrl = HybridController(topik, cfg)

    target_xy = np.array([0.02, 0.70])
    ctrl.set_target(target_xy, cartype=0)

    vision_noise_std = 0.0002
    stop_event = threading.Event()

    # ── 비전 스레드: 20 Hz로 에러 갱신 ─────────────────────────────────────
    def vision_thread():
        while not stop_event.is_set():
            with ctrl._ctrl_lock:
                true_lpin = ctrl.kin.fk_lpin(ctrl.q_cmd.copy())
            noise = np.random.randn(2) * vision_noise_std
            ctrl.push_vision(true_lpin + noise - target_xy)  # 에러 = 현위치 - 목표
            time.sleep(0.05)

    vt = threading.Thread(target=vision_thread, daemon=True)
    vt.start()

    # ── 제어 스레드: 10 Hz로 step 실행 ──────────────────────────────────────
    for i in range(200):
        ctrl.step()
        if ctrl.is_done:
            print(f"\n[DONE] Converged in {i+1} iterations")
            break
        if (i + 1) % 10 == 0:
            cnt = ctrl.get_q_cnt()
            with ctrl._ctrl_lock:
                lpin = ctrl.kin.fk_lpin(ctrl.q_cmd)
            err = np.linalg.norm(lpin - target_xy)
            print(f"  iter {i+1:3d}: state={ctrl.state.name:<12s} "
                  f"L-Pin=({lpin[0]:.4f},{lpin[1]:.4f})  err={err*1000:.3f}mm")
        time.sleep(cfg.dt)

    stop_event.set()
    vt.join(timeout=1.0)

    print("\n" + "=" * 60)
    final_lpin = ctrl.kin.fk_lpin(ctrl.q_cmd)
    final_err  = np.linalg.norm(final_lpin - target_xy)
    print(f"  State  : {ctrl.state.name}")
    print(f"  Error  : {final_err*1000:.3f} mm")
    print(f"  q_cmd  : {np.round(ctrl.q_cmd, 6)}")
    print("=" * 60)


def test_q2_locked_planner():
    """Unit test — 스레드 변경 없음, 단일 스레드 검증."""
    print("=" * 60)
    print("q2-Locked Planner Unit Test")
    print("=" * 60)

    cfg = ControlConfig(version=4)
    topik = ik.Topik(cfg.version)
    hw = HardwareInterface(topik, cfg)
    ctrl = HybridController(topik, cfg)
    kin = ctrl.kin

    d2 = topik.d2
    d3 = topik.d3[1]
    stage_lim = cfg.stage_limit

    wx0, wy0 = kin.wing_center_lpin(0.0)

    target1 = np.array([wx0 - d3 * np.cos(np.pi/4),
                         wy0 - d3 * np.sin(np.pi/4)])
    rtZ1, Lr1, act1, res1 = ctrl._geometry_plan_arm_q2_locked(target1)
    print(f"\nTest 1: Target on arm circle")
    assert not act1 and res1 < 1e-6
    print("  ✓ q2 locked, residual ≈ 0")

    target2 = np.array([wx0 - (d3 + stage_lim * 0.5) * np.cos(np.pi/4),
                         wy0 - (d3 + stage_lim * 0.5) * np.sin(np.pi/4)])
    _, _, act2, res2 = ctrl._geometry_plan_arm_q2_locked(target2)
    print(f"\nTest 2: Slightly outside circle")
    assert not act2 and res2 <= stage_lim + 1e-9
    print("  ✓ q2 locked, stage covers residual")

    target3 = np.array([0.02, 0.70])
    rtZ3, _, act3, _ = ctrl._geometry_plan_arm_q2_locked(target3)
    print(f"\nTest 3: Far target")
    assert act3 and abs(rtZ3) <= cfg.q2_max_rad + 1e-9
    print(f"  ✓ q2 activated within ±{cfg.q2_max_deg:.0f}°")

    q_test = np.zeros(7)
    q_test[2] = np.deg2rad(30.0)
    q_safe = hw.apply_safety_limits(q_test)
    assert abs(q_safe[2]) <= cfg.q2_max_rad + 1e-9
    print(f"\nTest 4: Safety limit  {np.rad2deg(q_test[2]):.1f}° → {np.rad2deg(q_safe[2]):.1f}°")
    print("  ✓ Safety limit enforced")

    print("\n" + "=" * 60)
    print("All tests PASSED ✓")
    print("=" * 60)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--sim',  action='store_true')
    parser.add_argument('--all',  action='store_true')
    args = parser.parse_args()

    if args.test or args.all:
        test_q2_locked_planner()
        print()
    if args.sim or args.all:
        simulate_pbvs()
    if not any([args.test, args.sim, args.all]):
        test_q2_locked_planner()
        print()
        simulate_pbvs()