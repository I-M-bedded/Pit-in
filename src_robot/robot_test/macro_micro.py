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
    MACRO_APPROACH   = auto()
    MICRO_APPROACH   = auto()
    PBVS_SERVOING    = auto()
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

    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha
        self._filtered_err = np.zeros(2)  # [ex, ey] in robot frame
        self._initialized = False

        # Camera-to-robot rotation (right-handed, Z-up)
        # Right camera: same orientation as robot base
        self._R_rcam = np.eye(3)
        # Left camera: 180° rotated about Z
        self._R_lcam = np.array([[-1.0, 0.0, 0.0],
                                  [0.0, -1.0, 0.0],
                                  [0.0,  0.0, 1.0]])

    def set_camera_rotation(self, R_rcam: np.ndarray, R_lcam: np.ndarray):
        """Override camera-to-robot rotation matrices if needed."""
        self._R_rcam = R_rcam.copy()
        self._R_lcam = R_lcam.copy()

    def cam_to_robot(self, cam_err: np.ndarray, side: str = 'right') -> np.ndarray:
        """Transform error from camera frame to robot base frame.
        
        Args:
            cam_err: [x, y, z] error in camera frame (meter)
            side: 'left' or 'right'
        
        Returns:
            [x, y, z] error in robot base frame (right-handed)
        """
        R = self._R_rcam if side == 'right' else self._R_lcam
        return R @ np.asarray(cam_err)

    def update(self, target_pos: np.ndarray, current_pos: np.ndarray) -> np.ndarray:
        """Compute filtered 2D position error in robot frame.
        
        Args:
            target_pos:  [x, y] target position in robot frame (m)
            current_pos: [x, y] current position from vision (m)
        
        Returns:
            filtered_err: [ex, ey] EMA-filtered error (m)
        """
        raw_err = np.asarray(target_pos[:2]) - np.asarray(current_pos[:2])

        if not self._initialized:
            self._filtered_err = raw_err.copy()
            self._initialized = True
        else:
            self._filtered_err = (self.alpha * raw_err
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
        self.vision = VisionFeedback(alpha=cfg.ema_alpha)

        self.state = State.MACRO_APPROACH
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
        self.state = State.MACRO_APPROACH
        self.vision.reset()
        self._settle_timer = 0.0

    def step(self, current_vision_xy: Optional[np.ndarray] = None) -> np.ndarray:
        """Execute one control step.
        
        Args:
            current_vision_xy: [x, y] from vision system (for PBVS), or None
        
        Returns:
            q_cmd_cnt: (7,) motor position command in encoder counts
        """
        if self.state == State.MACRO_APPROACH:
            self._step_macro()
        elif self.state == State.MICRO_APPROACH:
            self._step_micro()
        elif self.state == State.PBVS_SERVOING:
            if current_vision_xy is not None:
                self._step_pbvs(current_vision_xy)
        elif self.state == State.DONE:
            pass  # Hold position

        return self._q_cmd_to_cnt()

    # ----- State handlers -----

    def _step_macro(self):
        """MACRO_APPROACH: Use arm IK to reach the target vicinity."""
        # Clamp target to macro workspace (85% of max reach)
        max_reach = (self.topik.d2[1] + self.topik.d3[1]) * self.cfg.macro_reach_ratio
        target = self.target_xy.copy()
        dist = np.linalg.norm(target)
        if dist > max_reach:
            target = target * (max_reach / dist)

        # Set desired pin position for IK
        # Use symmetric pin gap from current cartype
        xo = self.topik.xo_arr[self.topik.cartype]
        pin_gap_half = (xo[0][1] - xo[1][1]) / 2.0  # Y-axis half gap

        # Set desired left/right pin positions
        # Target IS the Left Pin destination.
        self.topik.xd[0][0] = target[0]
        self.topik.xd[0][1] = target[1]
        self.topik.xd[0][2] = 0.0
        
        # Right Pin is offset from the Left Pin by twice the gap width along Y 
        # (Y is Left, so Right Pin is -Y relative to Left Pin when q=0)
        self.topik.xd[1][0] = target[0]
        self.topik.xd[1][1] = target[1] - abs(pin_gap_half * 2.0)
        self.topik.xd[1][2] = 0.0

        # Force stage to origin (0,0) for macro — arm does all the work
        self.topik.qm[0] = 0.0  # stageX = 0
        self.topik.qm[1] = 0.0  # stageY = 0

        try:
            self.topik.num_ik()
            self.q_cmd = self.topik.qdm.copy()
            # Override stage commands to zero
            self.q_cmd[0] = 0.0
            self.q_cmd[1] = 0.0
        except Exception as e:
            print(f"[MACRO] IK failed: {e}")
            return

        # Check if macro is close enough → transition
        qm_check = self.q_cmd.copy()
        lpin = self.kin.fk_lpin(qm_check)
        err = np.linalg.norm(lpin - self.target_xy)
        print(f"[MACRO] target={self.target_xy}, L-Pin={lpin}, err={err:.4f}m")

        if err < self.cfg.macro_threshold:
            print(f"[MACRO→MICRO] Macro approach complete, residual={err:.4f}m")
            self.state = State.MICRO_APPROACH
            self._settle_timer = time.time()

    def _step_micro(self):
        """MICRO_APPROACH: Compensate residual error with XY stage."""
        # Compute residual error from current q_cmd (Left Pin vs Target)
        lpin = self.kin.fk_lpin(self.q_cmd)
        residual = self.target_xy - lpin

        # Apply correction to stage only (lock arm joints)
        # Stage moves in opposite direction of residual (due to -qm mapping)
        self.q_cmd[0] += -residual[0]   # stageX correction (trY motor)
        self.q_cmd[1] += -residual[1]   # stageY correction (trX motor)

        # Clamp stage travel (typical max ±0.09m)
        stage_max = 0.088
        self.q_cmd[0] = np.clip(self.q_cmd[0], -stage_max, stage_max)
        self.q_cmd[1] = np.clip(self.q_cmd[1], -stage_max, stage_max)

        # Re-check after correction
        lpin = self.kin.fk_lpin(self.q_cmd)
        err = np.linalg.norm(lpin - self.target_xy)
        print(f"[MICRO] L-Pin={lpin}, err={err:.4f}m")

        # Wait for settling time
        elapsed = time.time() - self._settle_timer
        if elapsed > self.cfg.micro_settling_time and err < self.cfg.micro_threshold:
            print(f"[MICRO→PBVS] Micro approach complete, residual={err:.6f}m")
            self.state = State.PBVS_SERVOING
            self.vision.reset()

    def _step_pbvs(self, vision_xy: np.ndarray):
        """PBVS_SERVOING: Weighted visual servoing with discrete integration."""
        # Compute filtered error
        err = self.vision.update(self.target_xy, vision_xy)

        # Check convergence
        if self.vision.error_norm < self.cfg.pbvs_threshold:
            print(f"[PBVS→DONE] Converged! err={self.vision.error_norm:.6f}m")
            self.state = State.DONE
            return

        # Desired Cartesian velocity: x_dot = lambda * e
        x_dot = self.cfg.pbvs_lambda * err  # (2,)

        # Compute Jacobian and weighted pseudo-inverse
        J = self.kin.jacobian_4dof(self.q_cmd)
        J_winv = self.kin.weighted_pseudoinverse(J, self.W_diag)

        # Joint velocity: q_dot = J_winv * x_dot
        q_dot_4 = J_winv @ x_dot  # (4,): [dqm0, dqm1, dqm2, dqm3]

        # Velocity saturation
        max_vels = np.array([self.cfg.max_stage_vel, self.cfg.max_stage_vel,
                             self.cfg.max_arm_vel,   self.cfg.max_arm_vel])
        scale = np.max(np.abs(q_dot_4) / max_vels)
        if scale > 1.0:
            q_dot_4 /= scale

        # Discrete integration: q_cmd[k+1] = q_cmd[k] + q_dot * dt
        self.q_cmd[0] += q_dot_4[0] * self.cfg.dt  # stageX (trY)
        self.q_cmd[1] += q_dot_4[1] * self.cfg.dt  # stageY (trX)
        self.q_cmd[2] += q_dot_4[2] * self.cfg.dt  # rtZ
        self.q_cmd[3] += q_dot_4[3] * self.cfg.dt  # Lr (left wing)

        # Keep right wing symmetric (optional: could also be controlled)
        # self.q_cmd[4] = -self.q_cmd[3]

        print(f"[PBVS] err={err}, |err|={self.vision.error_norm:.6f}m, "
              f"q_dot={q_dot_4}")

    # ----- Utility -----

    def _q_cmd_to_cnt(self) -> np.ndarray:
        """Convert q_cmd (m, rad) to encoder counts."""
        cnt = np.zeros(7, dtype=int)
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
