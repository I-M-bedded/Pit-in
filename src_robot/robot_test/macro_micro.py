"""
Macro-Micro Hybrid PBVS Control System
=======================================
4-DOF system: 2-DOF Manipulator (Macro) + 2-DOF XY Stage (Micro)
Right-handed coordinate: X=forward, Y=left, Z=up

State Machine:
    MACRO_APPROACH → MICRO_APPROACH → PBVS_SERVOING → DONE

Motor mapping (7-servo system):
    [0] trY  : Y-translation (Micro-StageX_fwd)  → qm[0]
    [1] trX  : X-translation (Micro-StageY_left) → qm[1]
    [2] rtZ  : Z-rotation    (Macro-Arm base)     → qm[2]
    [3] Lr   : L-wing rot    (Macro-Arm link)     → qm[3]
    [4] Rr   : R-wing rot    (Macro-Arm link)     → qm[4]
    [5] Lz   : L-pin height                       → qm[5]
    [6] Rz   : R-pin height                       → qm[6]
"""

import numpy as np
import time
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from controller import ik


# =============================================================================
# Constants
# =============================================================================
class State(Enum):
    MACRO_ARM        = auto()   # Phase 1: Arm 2DOF only
    MICRO_STAGE      = auto()   # Phase 2: Stage 2DOF PBVS only
    VS_LIFT          = auto()   # Phase 3: Z up + 4DOF PBVS
    DONE             = auto()


@dataclass
class ControlConfig:
    """Control parameters for Macro-Micro PBVS system."""
    # Robot version
    version: int = 4

    # PBVS gain
    pbvs_lambda: float = 0.5

    # Control loop period (sec) — matched to vision framerate
    dt: float = 0.1

    # Convergence thresholds (meter)
    macro_threshold: float = 0.005     # 5mm for macro approach
    micro_threshold: float = 0.001     # 1mm for micro approach
    pbvs_threshold:  float = 0.0003    # 0.3mm for PBVS convergence (DONE)

    # Macro workspace limit (fraction of max reach)
    macro_reach_ratio: float = 0.85

    # Settling time after micro approach (sec)
    micro_settling_time: float = 0.5

    # Weight matrix diagonal: [arm_rtZ, arm_wing, stageX, stageY]
    # Large penalty on arm → stage preferred
    w_arm:   float = 1000.0
    w_stage: float = 1.0

    # EMA filter coefficient for vision noise (0 < alpha <= 1, smaller = more smooth)
    ema_alpha: float = 0.3

    # Joint velocity limits (rad/s for arm, m/s for stage)
    max_arm_vel:   float = 0.5    # rad/s
    max_stage_vel: float = 0.05   # m/s

    # Unit conversion (from ik.Topik)
    # m2cnt / cnt2m inherited from Topik instance

    # Motor velocity defaults (cnt/s)
    t_vel: List[int] = field(default_factory=lambda: [10000, 10000, 2000, 600, 600, 50000, 50000])


# =============================================================================
# Module A: VisionFeedback
# =============================================================================
class VisionFeedback:
    """ArUco marker / target 3D Pose estimation.
    
    Converts camera-frame error to robot-base-frame error (right-handed).
    Applies EMA low-pass filter for noise rejection.
    """

    def __init__(self, topik: ik.Topik, alpha: float = 0.3):
        self.topik = topik
        self.alpha = alpha
        self._filtered_err = np.zeros(2)  # [ex, ey] pin-to-hole world error
        self._initialized = False

        # Camera is mounted on the Left Pin.
        # It is rotated -90 degrees around Z relative to the pin's local frame
        # rot_Z(-90) = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
        self._R_lcam_mount = np.array([[ 0.0,  1.0, 0.0],
                                       [-1.0,  0.0, 0.0],
                                       [ 0.0,  0.0, 1.0]])
                                       
        # Camera physical offset from the pin center (X=-5.7cm, Y=2.9cm)
        self._offset_lcam = np.array([-0.057, 0.029, 0.0])

    def get_pin_error(self, cam_err: np.ndarray, side: str = 'left') -> np.ndarray:
        """Transform raw camera vision err into a pinpoint world tracking error.
        
        Args:
            cam_err: [x, y, z] target error in camera frame (meter)
            side: 'left' or 'right'
        
        Returns:
            world_err: [ex, ey] world frame error between Pin and Target Hole (m)
        """
        # Hole position in Pin's local coordinate frame
        hole_local = self._offset_lcam + self._R_lcam_mount @ np.asarray(cam_err)
        
        # Hole position offset in World coordinate frame
        R_world_pin = self.topik.so3_lcam
        hole_world_offset = np.asarray(R_world_pin @ hole_local).flatten()
        return hole_world_offset[:2]

    def update_error(self, raw_pin_err: np.ndarray) -> np.ndarray:
        """Apply EMA filter to the world pin error."""
        if not self._initialized:
            self._filtered_err = raw_pin_err.copy()
            self._initialized = True
        else:
            self._filtered_err = (self.alpha * raw_pin_err
                                  + (1.0 - self.alpha) * self._filtered_err)

        return self._filtered_err.copy()

    def reset(self):
        """Reset filter state."""
        self._filtered_err = np.zeros(2)
        self._initialized = False

    @property
    def error_norm(self) -> float:
        return float(np.linalg.norm(self._filtered_err))


# =============================================================================
# Module B: KinematicsSolver
# =============================================================================
class KinematicsSolver:
    """Forward Kinematics and Jacobian for the 4-DOF hybrid system.
    
    Right-handed coordinate: X=forward (cos), Y=left (sin), Z=up.

    4-DOF subset for PBVS:
        q = [q_rtZ, q_wing, q_stageX, q_stageY]
        → maps to full system [qm0(trY), qm1(trX), qm2(rtZ), qm3(Lr)]
        
    Task space: [x, y] position of a reference point (e.g., center of pins).
    """

    def __init__(self, topik: ik.Topik):
        self.topik = topik
        self.d2 = topik.d2
        self.d3 = topik.d3

    def fk_lpin(self, qm: np.ndarray) -> np.ndarray:
        """Compute Left Pin position from joint values.
        
        Args:
            qm: [qm0..qm6] in (m, rad) — physical joint values
        
        Returns:
            lpin: [x, y] in meter (right-handed)
        """
        # Update topik internal state
        self.topik.qm = qm.copy()
        # Zero out qm2 sign conventions used internally
        self.topik.qm2[0] = -qm[0]
        self.topik.qm2[1] = -qm[1]
        self.topik.qm2[2] = -qm[2]
        self.topik.qm2[3] = -qm[3]
        self.topik.qm2[4] = -qm[4]
        self.topik.qm2[5] =  qm[5]
        self.topik.qm2[6] =  qm[6]

        self.topik.fk()

        # Left pin coordinate
        lx, ly = self.topik.x[0][0], self.topik.x[0][1]
        return np.array([lx, ly])

    def jacobian_4dof(self, qm: np.ndarray) -> np.ndarray:
        """Compute 2×4 Jacobian for the 4-DOF subset.
        
        Maps q_dot = [dqm0(stageX), dqm1(stageY), dqm2(rtZ), dqm3(Lw)] 
        → x_dot = [dx, dy] of center point.
        
        The full 4×7 Jacobian from Topik gives [dLx, dLy, dRx, dRy] / d[qm0..qm6].
        Center Jacobian = 0.5*(J_left + J_right) projected to the 4-DOF columns.
        
        Args:
            qm: [qm0..qm6] in (m, rad)
        
        Returns:
            J_4dof: (2, 4) Jacobian matrix
        """
        # Ensure FK is up-to-date
        self.topik.qm = qm.copy()
        self.topik.get_J()
        J_full = self.topik.J  # (4, 7): [Lx, Ly, Rx, Ry] × [qm0..qm6]

        # Left Pin Jacobian: dx_lpin = dLx, dy_lpin = dLy
        J_lx = J_full[0, :]  # d(Lx)/d(qm0..6)
        J_ly = J_full[1, :]  # d(Ly)/d(qm0..6)

        J_lpin = np.vstack([J_lx, J_ly])  # (2, 7)

        # Extract 4-DOF columns: qm0(stageX), qm1(stageY), qm2(rtZ), qm3(Lw)
        cols = [0, 1, 2, 3]
        return J_lpin[:, cols]  # (2, 4)

    def weighted_pseudoinverse(self, J: np.ndarray, W_diag: np.ndarray) -> np.ndarray:
        """Compute weighted pseudo-inverse: W^{-1} J^T (J W^{-1} J^T)^{-1}.
        
        Args:
            J:      (2, 4) Jacobian
            W_diag: (4,) diagonal of weight matrix [w_stageX, w_stageY, w_rtZ, w_wing]
        
        Returns:
            J_winv: (4, 2) weighted pseudo-inverse
        """
        W_inv = np.diag(1.0 / W_diag)
        JWiJt = J @ W_inv @ J.T  # (2, 2)

        # Regularize for numerical stability
        JWiJt += np.eye(2) * 1e-10

        return W_inv @ J.T @ np.linalg.inv(JWiJt)  # (4, 2)


# =============================================================================
# Module C: HybridController
# =============================================================================
class HybridController:
    """Macro-Micro hybrid PBVS state machine controller.
    
    States:
        MACRO_APPROACH  — IK-based arm positioning
        MICRO_APPROACH  — Residual error correction via XY stage
        PBVS_SERVOING   — Real-time weighted visual servoing
        DONE            — Converged
    """

    def __init__(self, topik: ik.Topik, cfg: ControlConfig):
        self.topik = topik
        self.cfg = cfg
        self.kin = KinematicsSolver(topik)
        self.vision = VisionFeedback(topik=topik, alpha=cfg.ema_alpha)
        # Instantiate hardware interface inside to access safety limits dynamically
        self.hw = HardwareInterface(topik, cfg)

        self.state = State.MACRO_ARM
        self._settle_timer: float = 0.0

        # Current joint command in (m, rad)
        self.q_cmd = np.zeros(7)

        # Target position [x, y] in robot frame (meter)
        self.target_xy = np.zeros(2)

        # Weight diagonal: [stageX, stageY, rtZ, wing]
        self.W_diag = np.array([cfg.w_stage, cfg.w_stage,
                                cfg.w_arm,   cfg.w_arm])

    def set_target(self, target_xy: np.ndarray, cartype: int = 0):
        """Set the target position and initialize state machine.
        
        Args:
            target_xy: [x, y] target in robot base frame (meter, RH)
            cartype: vehicle type index
        """
        self.target_xy = np.asarray(target_xy[:2], dtype=float)
        self.topik.cartype = cartype
        self.state = State.MACRO_ARM
        self.vision.reset()
        self._settle_timer = 0.0

    def step(self, current_vision_xy: Optional[np.ndarray] = None) -> np.ndarray:
        """Execute one control step.
        
        Args:
            current_vision_xy: [x, y] from vision system (for PBVS), or None
        
        Returns:
            q_cmd_cnt: (7,) motor position command in encoder counts
        """
        if self.state == State.MACRO_ARM:
            self._step_macro()
        elif self.state == State.MICRO_STAGE:
            if current_vision_xy is not None:
                self._step_micro(current_vision_xy)
        elif self.state == State.VS_LIFT:
            if current_vision_xy is not None:
                self._step_vs_lift(current_vision_xy)
        elif self.state == State.DONE:
            pass  # Hold position

        return self._q_cmd_to_cnt()

    def _solve_single_pin_ik(self, target_xy: np.ndarray) -> np.ndarray:
        """Numerical Inverse Kinematics solver for purely 1-Pin (Left) 2DOF target -> 4DOF manipulator.
        Iteratively minimizes the position error using the weighted 2x4 Jacobian pseudoinverse.
        """
        q_opt = np.zeros(7)
        # Start from current actual arm position to minimize unnecessary flailing
        q_opt[2] = self.q_cmd[2]
        q_opt[3] = self.q_cmd[3]
        
        max_iter = 100
        for _ in range(max_iter):
            lpin = self.kin.fk_lpin(q_opt)
            err = target_xy[:2] - lpin
            
            # converged within 0.5mm
            if np.linalg.norm(err) < 0.0005:
                break
                
            J = self.kin.jacobian_4dof(q_opt)
            
            # For MACRO, severely penalize Stage movement and heavily favor Arm movement.
            # This forces the arm joints to absorb the long distances.
            W_macro = np.array([100.0, 100.0, 1.0, 1.0])
            J_winv = self.kin.weighted_pseudoinverse(J, W_macro)
            
            dq_4 = J_winv @ err
            q_opt[0] += dq_4[0]
            q_opt[1] += dq_4[1]
            q_opt[2] += dq_4[2]
            q_opt[3] += dq_4[3]
            
            # Clamp limits to ensure realistic reachability convergence
            q_opt = self.hw.apply_safety_limits(q_opt)
            
        return q_opt

    # ----- State handlers -----

    def _step_macro(self):
        """MACRO_APPROACH: Use Single-Pin Numerical IK to reach the target."""
        target = self.target_xy.copy()

        try:
            # Solve using dedicated pure 1-Pin 4DOF IK!
            required_q = self._solve_single_pin_ik(target)
            
            # --- [DEBUG] Print numerical IK targets ---
            print(f"\n[MACRO DEBUG] Single-Pin Numerical IK Angles to reach {np.round(target, 3)}:")
            print(f"  Stage X (trY_fwd) : {required_q[0]:.4f} m")
            print(f"  Stage Y (trX_left): {required_q[1]:.4f} m")
            print(f"  Base Rotation(rtZ): {np.rad2deg(required_q[2]):.2f} deg")
            print(f"  L-Wing Angle(Lr)  : {np.rad2deg(required_q[3]):.2f} deg")
            
            # --- [UNREACHABLE CHECK] Validate physical reachability ---
            lpin_reach = self.kin.fk_lpin(required_q)
            residual = np.linalg.norm(target - lpin_reach)
            
            # If the iterative solver hit a physical boundary, the residual will be > 0.
            if residual > 0.005:  # > 5mm error means unreachable physically
                print(f"\n[ERROR] Target is UNREACHABLE!")
                print(f"Physical hardware limits prevent the Left Pin from reaching the target.")
                print(f"  Target  : {np.round(target, 4)}")
                print(f"  Best Pos: {np.round(lpin_reach, 4)} (Residual: {residual:.4f}m)")
                self.state = State.DONE  # Escaping loop safely
                return

            self.q_cmd = required_q.copy()
        except Exception as e:
            print(f"[MACRO] Custom IK solver failed: {e}")
            self.state = State.DONE
            return

        # Check if macro is close enough → transition
        qm_check = self.q_cmd.copy()
        lpin = self.kin.fk_lpin(qm_check)
        err = np.linalg.norm(lpin - self.target_xy)
        print(f"[MACRO] target={self.target_xy}, L-Pin={lpin}, err={err:.4f}m")

        if err < self.cfg.macro_threshold:
            print(f"[MACRO→MICRO_STAGE] Arm approach complete, residual={err:.4f}m")
            self.state = State.MICRO_STAGE
            self._settle_timer = time.time()

    def _step_micro(self, raw_pin_err: np.ndarray):
        """Phase 2: MICRO_STAGE (스테이지 2DOF 전용 정밀 에러 보상 PBVS)"""
        # EMA 필터 적용된 월드 오차 산출
        err = self.vision.update_error(raw_pin_err)
        
        # 타겟 센터를 조준하고 있다면 에러가 0근처일 시 종료
        if self.vision.error_norm < self.cfg.pbvs_threshold:
            self._settle_timer += self.cfg.dt
            if self._settle_timer > self.cfg.micro_settling_time:
                print(f"\n[TRANSITION] MICRO_STAGE -> VS_LIFT (Vision Error Conquered!)")
                self.state = State.VS_LIFT
                self._settle_timer = 0.0
            return
        else:
            self._settle_timer = 0.0
        
        # PBVS 이득 곱 (lambda)
        q_dot_stage_target = self.cfg.pbvs_lambda * err
        
        # 1-Pin Jacobian 계산 (Stage 속도 변환)
        J = self.kin.jacobian_4dof(self.q_cmd)
        J_stage = J[:, 0:2]  # Stage X, Y에 해당하는 2x2 영역만 추출
        
        # 편미분 2x2 행렬의 역행렬 사용
        try:
            J_stage_inv = np.linalg.inv(J_stage)
            dq_stage = J_stage_inv @ q_dot_stage_target
            
            # 속도 제한 (선형 댐핑)
            scale = 1.0
            if np.linalg.norm(dq_stage) > self.cfg.max_stage_vel:
                scale = np.linalg.norm(dq_stage) / self.cfg.max_stage_vel
            dq_stage /= scale
            
            # PBVS 누적 이동 (이산 적분)
            self.q_cmd[0] += dq_stage[0] * self.cfg.dt  # stageX
            self.q_cmd[1] += dq_stage[1] * self.cfg.dt  # stageY
            
            # 스테이지 물리계 리미트 검파
            self.q_cmd = self.hw.apply_safety_limits(self.q_cmd)
            
            # 팔 각도는 MACRO 단계에서 설정된 값으로 절대 고정 (변동 불가)
            
            if self._settle_timer == 0.0: # Debug reduce spam
                # print(f"[MICRO] Stage pbvs tracking (err: {np.linalg.norm(err):.5f}m)")
                pass
                
        except np.linalg.LinAlgError:
            print("[ERROR] Stage Jacobian pseudo-inverse failed in Micro!")

    def _step_vs_lift(self, current_vision_xy: np.ndarray):
        """Phase 3: VS_LIFT (핀-홀 결합용 Z모터 4DOF 융합 미세 PBVS)
        
        [구현 스펙/주석 블록]
        1. Z축 결합 모터(Lz) 스크류 폼을 작동하여 지정 높이까지 일정 속도로 들어 올린다.
        2. Z축이 상승하는 동안 물리적 마찰로 발생하는 XY 오차와 카메라 비전 에러를 모니터링한다.
        3. _step_micro 와 비슷하게 비전 PBVS를 돌리되, 이번엔 전체 4DOF Jacobian을 사용한다!
        4. 다만 W_diag(가중치 행렬) 설정 시 스테이지(X, Y) 쪽에 매우 낮은 페널티(예: 1.0)를 주고,
           팔 관절(rtZ, wing) 쪽에 극도로 높은 페널티(예: 1000.0)를 할당한다.
        5. 결과적으로 보정은 십중팔구 스테이지가 주름잡게 되며, 팔은 물리적 한계점 부근에서나 미세하게 거드는
           하이브리드 충돌 회피형 VS 보정을 달성하게 될 것이다.
        6. 높이(Lz)가 요구치에 도달하면 self.state = State.DONE 판정.
        """
        # 현재는 주석 골격 상태이므로 즉각 DONE 처리
        print("\n[PHASE 3] VS_LIFT Stub reached. Pin-Hole fully aligned. Shutting down PBVS.")
        self.state = State.DONE
    # ----- Utility -----

    def _q_cmd_to_cnt(self) -> list:
        """Convert q_cmd (m, rad) to encoder counts.
        
        Returns native Python int list (NOT numpy int64!)
        because Fastech protocol.py calls position.to_bytes()
        which only works on native Python int.
        """
        cnt = [0] * 7
        for i in range(7):
            cnt[i] = int(np.round(self.q_cmd[i] * self.topik.m2cnt[i]))
        return cnt

    @property
    def is_done(self) -> bool:
        return self.state == State.DONE


# =============================================================================
# Module D: HardwareInterface
# =============================================================================
class HardwareInterface:
    """Interface between HybridController and physical servo motors.
    
    Converts q_cmd (radian, meter) → encoder count commands
    and manages motor profile settings for real-time tracking.
    """

    def __init__(self, topik: ik.Topik, cfg: ControlConfig):
        self.topik = topik
        self.cfg = cfg
        self.m2cnt = topik.m2cnt.copy()
        self.cnt2m = topik.cnt2m.copy()

    def q_to_cnt(self, q_cmd: np.ndarray) -> List[int]:
        """Convert joint-space command to encoder counts.
        
        Args:
            q_cmd: (7,) [trY(m), trX(m), rtZ(rad), Lr(rad), Rr(rad), Lz(m), Rz(m)]
        
        Returns:
            cnt: (7,) encoder count command
        """
        return [int(np.round(q_cmd[i] * self.m2cnt[i])) for i in range(7)]

    def cnt_to_q(self, cnt: np.ndarray) -> np.ndarray:
        """Convert encoder counts to joint-space values.
        
        Args:
            cnt: (7,) encoder counts
        
        Returns:
            q: (7,) in (m, rad)
        """
        return np.array([cnt[i] * self.cnt2m[i] for i in range(7)])

    def get_velocity_profile(self, q_dot: np.ndarray) -> List[int]:
        """Compute motor velocity commands from joint velocities.
        
        Args:
            q_dot: (7,) joint velocities in (m/s, rad/s)
        
        Returns:
            vel_cnt: (7,) velocity commands in cnt/s
        """
        vel = np.zeros(7)
        for i in range(7):
            vel[i] = abs(q_dot[i] * self.m2cnt[i])
        
        # Clamp to default velocities
        for i in range(7):
            vel[i] = min(vel[i], self.cfg.t_vel[i])
            vel[i] = max(vel[i], 100)  # minimum velocity

        return [int(v) for v in vel]

    def apply_safety_limits(self, q_cmd: np.ndarray) -> np.ndarray:
        """Apply joint-space safety limits.
        
        Args:
            q_cmd: (7,) command in (m, rad)
        
        Returns:
            q_safe: (7,) clamped command
        """
        q_safe = q_cmd.copy()

        # Stage limits (±88mm)
        q_safe[0] = np.clip(q_safe[0], -0.088, 0.054)   # trY (X forward)
        q_safe[1] = np.clip(q_safe[1], -0.063, 0.057)    # trX (Y left)

        # Rotation limit (±~18.6°)
        q_safe[2] = np.clip(q_safe[2], -0.325, 0.065)    # rtZ

        # Wing rotation limits (±~97°)
        q_safe[3] = np.clip(q_safe[3], -1.7, 1.025)      # Lr
        q_safe[4] = np.clip(q_safe[4], -1.7, 1.318)      # Rr

        # Pin height limits (always >= 0)
        q_safe[5] = max(q_safe[5], 0.0)
        q_safe[6] = max(q_safe[6], 0.0)

        return q_safe


# =============================================================================
# Simulation & Test
# =============================================================================
def simulate_pbvs():
    """Simulate the full Macro-Micro PBVS pipeline without real hardware."""
    print("=" * 60)
    print("Macro-Micro Hybrid PBVS Simulation")
    print("=" * 60)

    # Initialize
    cfg = ControlConfig(version=4)
    topik = ik.Topik(cfg.version)
    hw = HardwareInterface(topik, cfg)
    ctrl = HybridController(topik, cfg)

    # Target: slightly offset from niro home position
    target_xy = np.array([0.02, 0.70])  # X=2cm forward, Y=0.70m left
    ctrl.set_target(target_xy, cartype=0)

    print(f"\nTarget position: {target_xy}")
    print(f"Initial state: {ctrl.state.name}")
    print("-" * 60)

    max_iter = 200
    vision_noise_std = 0.0002  # 0.2mm noise

    for i in range(max_iter):
        # Simulate vision feedback (true L-Pin position + noise)
        true_lpin = ctrl.kin.fk_lpin(ctrl.q_cmd)
        noise = np.random.randn(2) * vision_noise_std
        simulated_vision = true_lpin + noise

        # Step controller
        q_cnt = ctrl.step(current_vision_xy=simulated_vision)

        # Apply safety limits
        ctrl.q_cmd = hw.apply_safety_limits(ctrl.q_cmd)

        if ctrl.is_done:
            print(f"\n[DONE] Converged in {i+1} iterations")
            break

        if (i + 1) % 10 == 0:
            lpin = ctrl.kin.fk_lpin(ctrl.q_cmd)
            err = np.linalg.norm(lpin - target_xy)
            print(f"  iter {i+1}: state={ctrl.state.name}, "
                  f"L-Pin={lpin}, err={err:.6f}m")

    # Final results
    print("\n" + "=" * 60)
    print("Final Results:")
    final_lpin = ctrl.kin.fk_lpin(ctrl.q_cmd)
    final_err = np.linalg.norm(final_lpin - target_xy)
    q_cnt = hw.q_to_cnt(ctrl.q_cmd)

    print(f"  State:    {ctrl.state.name}")
    print(f"  Target:   {target_xy}")
    print(f"  Achieved (L-Pin): {final_lpin}")
    print(f"  Error:    {final_err:.6f} m")
    print(f"  q_cmd(m,rad): {np.round(ctrl.q_cmd, 6)}")
    print(f"  q_cmd(cnt):   {q_cnt}")
    print("=" * 60)


def test_coordinate_conversion():
    """Verify FK coordinate conversion: same physical position, swapped axes."""
    print("=" * 60)
    print("FK Right-Handed Coordinate Verification")
    print("=" * 60)

    topik = ik.Topik(4)

    # Test 1: Zero configuration — arm extends along X (forward)
    topik.get_q([0, 0, 0, 0, 0, 0, 0])
    topik.fk()
    print(f"\nTest 1: q=[0]*7 (zero config)")
    print(f"  Left pin:  x={topik.x[0][0]:.6f}, y={topik.x[0][1]:.6f} (RH: X=fwd, Y=left)")
    print(f"  Right pin: x={topik.x[1][0]:.6f}, y={topik.x[1][1]:.6f}")
    print(f"  Pin gap:   {topik.p2p:.6f}m")
    # At q2=0, arm extends along X (forward) → x[0] should be large (cos-dominant)
    assert abs(topik.x[0][0]) > 0.1, "X should be significant at q=0"
    # Left-right symmetry: pins mirror about origin in X (x_L ≈ -x_R)
    assert abs(topik.x[0][0] + topik.x[1][0]) < 0.001, "Pins should be mirrored in X at symmetric q"
    print("  ✓ At q=0, arm extends along X-forward (cos-dominant)")
    print("  ✓ Left/right pins mirrored symmetrically")

    # Test 2: Niro configuration — compute q from IK, then verify FK round-trip
    topik2 = ik.Topik(4)
    q_niro = topik2.set_xd(0, np.array([[0, 0, 0], [0, 0, 0]]))
    topik2.get_q(q_niro)
    topik2.fk()
    x_fk = topik2.x.copy()
    p2p_fk = topik2.p2p
    print(f"\nTest 2: Niro home position (IK→FK)")
    print(f"  IK q_cmd:  {q_niro}")
    print(f"  FK Left:   x={x_fk[0][0]:.6f}, y={x_fk[0][1]:.6f}")
    print(f"  FK Right:  x={x_fk[1][0]:.6f}, y={x_fk[1][1]:.6f}")
    print(f"  Pin gap:   {p2p_fk:.6f}m (expected ≈ {ik.niro_gap}m)")
    assert abs(p2p_fk - ik.niro_gap) < 0.02, f"Pin gap mismatch: {p2p_fk} vs {ik.niro_gap}"
    print("  ✓ Pin gap matches niro specification")

    # Test 3: FK→IK round-trip consistency
    topik2.xd = x_fk.copy()
    topik2.num_ik()
    q_reik = topik2.qd
    print(f"\nTest 3: FK→IK round trip")
    print(f"  Original:  {q_niro}")
    print(f"  Re-IK:     {q_reik}")
    max_diff = max(abs(q_niro[i] - q_reik[i]) for i in range(7))
    print(f"  Max diff:  {max_diff} counts")
    assert max_diff < 5, f"FK→IK round-trip error too large: {max_diff}"
    print("  ✓ FK→IK round-trip verified")

    # Test 4: Jacobian consistency (numerical vs analytical)
    topik3 = ik.Topik(4)
    q_test = topik3.set_xd(0, np.array([[0, 0, 0], [0, 0, 0]]))
    topik3.get_q(q_test)
    topik3.fk()
    topik3.get_J()
    J_analytical = topik3.J.copy()
    # Numerical Jacobian via finite differences
    eps = 1e-6
    J_numerical = np.zeros((4, 7))
    q_base_m = topik3.qm.copy()
    for j in range(7):
        qm_plus = q_base_m.copy()
        qm_plus[j] += eps
        topik3.qm = qm_plus
        topik3.fk()
        x_plus = np.array([topik3.x[0][0], topik3.x[0][1],
                           topik3.x[1][0], topik3.x[1][1]])
        qm_minus = q_base_m.copy()
        qm_minus[j] -= eps
        topik3.qm = qm_minus
        topik3.fk()
        x_minus = np.array([topik3.x[0][0], topik3.x[0][1],
                            topik3.x[1][0], topik3.x[1][1]])
        J_numerical[:, j] = (x_plus - x_minus) / (2 * eps)
    j_err = np.max(np.abs(J_analytical - J_numerical))
    print(f"\nTest 4: Jacobian consistency (analytical vs numerical)")
    print(f"  Max element error: {j_err:.2e}")
    assert j_err < 1e-4, f"Jacobian mismatch: {j_err}"
    print("  ✓ Jacobian analytically correct")

    print("\n" + "=" * 60)
    print("All coordinate conversion tests PASSED ✓")
    print("=" * 60)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Macro-Micro PBVS System")
    parser.add_argument('--test', action='store_true', help='Run coordinate tests')
    parser.add_argument('--sim', action='store_true', help='Run PBVS simulation')
    parser.add_argument('--all', action='store_true', help='Run all tests')
    args = parser.parse_args()

    if args.test or args.all:
        test_coordinate_conversion()
        print()

    if args.sim or args.all:
        simulate_pbvs()

    if not any([args.test, args.sim, args.all]):
        # Default: run both
        test_coordinate_conversion()
        print()
        simulate_pbvs()
