"""
Macro-Micro Hybrid PBVS Control System
=======================================
4-DOF system: 2-DOF Manipulator (Macro) + 2-DOF XY Stage (Micro)
Right-handed coordinate: X=forward, Y=left, Z=up

State Machine:
    MACRO_APPROACH -> MICRO_APPROACH -> PBVS_SERVOING -> DONE

Motor mapping (7-servo system):
    [0] trY  : Y-translation (Micro-StageX_fwd)  -> qm[0]
    [1] trX  : X-translation (Micro-StageY_left) -> qm[1]
    [2] rtZ  : Z-rotation    (Macro-Arm base)     -> qm[2]
    [3] Lr   : L-wing rot    (Macro-Arm link)     -> qm[3]
    [4] Rr   : R-wing rot    (Macro-Arm link)     -> qm[4]
    [5] Lz   : L-pin height                       -> qm[5]
    [6] Rz   : R-pin height                       -> qm[6]
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
from robot_test.jacobian_solver import KinematicsSolver


# =============================================================================
# Constants
# =============================================================================
class State(Enum):
    MACRO_ARM        = auto()   # Phase 1: Arm 2DOF only
    WAIT_MACRO       = auto()   # Wait for MACRO_ARM to physically reach target
    MICRO_STAGE      = auto()   # Phase 2: Observation (Look)
    WAIT_MICRO       = auto()   # Phase 2: Stage action (Move)
    VS_LIFT          = auto()   # Phase 3: Z up + 4DOF PBVS
    DONE             = auto()


class FinishReason(Enum):
    """Termination reason for controller tasks."""

    SUCCESS = auto()
    FAILED_IK = auto()
    FAILED_TIMEOUT = auto()
    FAILED_VARIANCE = auto()
    FAILED_VISION = auto()


# Side-dependent configuration for Left / Right pin
SIDE_CFG = {
    'left': {
        'fk_idx': 0,                # topik.x[0] = Left pin
        'wing_joint': 3,            # qm[3] = Lr
        'z_motor': 5,               # qm[5] = Lz
        'jac_rows': (0, 1),         # J rows: d(Lx)/dq, d(Ly)/dq
        'jac_cols': [0, 1, 2, 3],   # stage X, Y + rtZ + Lr
        'so3_attr': 'so3_lcam',
        'cam_mount_R': np.array([[ 0.0,  1.0, 0.0],    # rot_Z(-90)
                                  [-1.0,  0.0, 0.0],
                                  [ 0.0,  0.0, 1.0]]),
        'cam_offset': np.array([-0.057, 0.029, 0.0]),
        'ik_sign': -1,              # d2, d3 extend in -cos direction
    },
    'right': {
        'fk_idx': 1,                # topik.x[1] = Right pin
        'wing_joint': 4,            # qm[4] = Rr
        'z_motor': 6,               # qm[6] = Rz
        'jac_rows': (2, 3),         # J rows: d(Rx)/dq, d(Ry)/dq
        'jac_cols': [0, 1, 2, 4],   # stage X, Y + rtZ + Rr
        'so3_attr': 'so3_rcam',
        'cam_mount_R': np.array([[ 0.0, -1.0, 0.0],    # rot_Z(+90)
                                  [ 1.0,  0.0, 0.0],
                                  [ 0.0,  0.0, 1.0]]),
        'cam_offset': np.array([0.057, -0.029, 0.0]),
        'ik_sign': +1,              # d2, d3 extend in +cos direction
    },
}


@dataclass
class ControlConfig:
    """Control parameters for Macro-Micro PBVS system."""
    # Robot version
    version: int = 4

    # --- Phase Enable Flags (for debugging) ---
    enable_macro: bool = True    # False -> skip Phase 1, jump to Phase 2
    enable_micro: bool = True    # False -> skip Phase 2, jump to DONE

    # PBVS gain
    pbvs_lambda: float = 0.5

    # Control loop period (sec) -matched to vision framerate
    dt: float = 0.1

    # Convergence thresholds (meter)
    macro_threshold: float = 0.005    # 5mm for macro approach
    micro_threshold: float = 0.003     # 3mm for micro final success
    pbvs_threshold:  float = 0.0003    # 0.3mm for PBVS convergence (DONE)

    # Settling / Wait Setting tolerances
    joint_settle_rad: float = 0.01     # 0.57 deg for arm joints
    stage_settle_m: float = 0.001      # 1mm for stage X, Y
    settle_timeout: float = 40        # seconds before aborting wait

    # Micro Observation Settings
    micro_obs_frames: int = 10         # Number of frames for mean/variance check
    micro_std_limit: float = 0.005     # Target variance std-dev limit (5mm)

    # Settling time after micro approach (sec)
    micro_settling_time: float = 300

    # Stage travel limit from home (meter)
    stage_limit: float = 0.075          # 7.5 cm

    # Weight matrix diagonal: [stageX, stageY, arm_rtZ, arm_wing]
    # Large penalty on arm -> stage preferred
    w_arm:   float = 1000.0
    w_stage: float = 1.0

    # EMA filter coefficient for vision noise (0 < alpha <= 1, smaller = more smooth)
    ema_alpha: float = 0.3

    # Joint velocity limits (rad/s for arm, m/s for stage)
    max_arm_vel:   float = 0.5    # rad/s
    max_stage_vel: float = 0.05   # m/s

    # Motor velocity defaults (cnt/s)
    t_vel: List[int] = field(default_factory=lambda: [10000, 10000, 2000, 600, 600, 50000, 50000])


# =============================================================================
# Module A: VisionFeedback
# =============================================================================
class VisionFeedback:
    """World-frame hole position -> pin-to-hole error converter.

    Vision node (`vision_node.py`) publishes the filtered hole position in
    the robot world frame directly on `/cam0`.  This class reads the
    current pin world-position via FK and returns the 2D displacement.
    Applies an EMA low-pass filter for extra smoothing on top of
    WorldTracker's EMA.
    """

    def __init__(self, topik: ik.Topik, alpha: float = 0.3, side: str = 'left'):
        self.topik = topik
        self.alpha = alpha
        self._filtered_err = np.zeros(2)
        self._initialized = False
        self.set_side(side)

    def set_side(self, side: str):
        """Switch FK index for the given side."""
        self.side = side
        self._fk_idx = SIDE_CFG[side]['fk_idx']

    def get_pin_error(self, world_hole_xyz: np.ndarray) -> np.ndarray:
        """Compute world-frame pin-to-hole error.

        Args:
            world_hole_xyz: [x, y, z] hole position in robot world frame (m),
                            published by the vision node on /cam0.

        Returns:
            world_err: [ex, ey] world-frame pin-to-hole error (m).
        """
        pin_world = np.asarray(self.topik.x[self._fk_idx], dtype=float).flatten()
        hole_world = np.asarray(world_hole_xyz, dtype=float).flatten()
        return hole_world[:2] - pin_world[:2]

    def get_target_error(self, target_xy: np.ndarray) -> np.ndarray:
        """Compute world-frame pin-to-target error for a frozen world target."""
        pin_world = np.asarray(self.topik.x[self._fk_idx], dtype=float).flatten()
        target_world = np.asarray(target_xy[:2], dtype=float).flatten()
        return target_world - pin_world[:2]

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
# Module B: SingleSideController
# =============================================================================
class SingleSideController:
    """Macro-Micro hybrid PBVS state machine controller.
    
    States:
        MACRO_APPROACH  -IK-based arm positioning
        MICRO_APPROACH  -Residual error correction via XY stage
        PBVS_SERVOING   -Real-time weighted visual servoing
        DONE            -Converged
    """

    def __init__(self, topik: ik.Topik, cfg: ControlConfig):
        self.topik = topik
        self.cfg = cfg
        self.kin = KinematicsSolver(topik)
        self.vision = VisionFeedback(topik=topik, alpha=cfg.ema_alpha, side='left')
        self.hw = HardwareInterface(topik, cfg)

        self.side = 'left'
        self.state = State.MACRO_ARM
        self._settle_timer: float = 0.0

        # Current joint command in (m, rad)
        self.q_cmd = np.zeros(7)
        self.q_cmd_cnt = [0] * 7
        self._measured_q = np.zeros(7)
        self._selected_arm_branch = ""

        # Phase targets [x, y] in robot/world frame (meter).
        # Macro uses the task-start snapshot. Micro latches a fresh snapshot
        # once, right after the arm settles, and keeps following only that.
        self.macro_target_xy = np.zeros(2)
        self.micro_target_xy: Optional[np.ndarray] = None

        # Weight diagonal: [stageX, stageY, rtZ, wing]
        self.W_diag = np.array([cfg.w_stage, cfg.w_stage,
                                cfg.w_arm,   cfg.w_arm])

    @property
    def _wing_idx(self) -> int:
        return SIDE_CFG[self.side]['wing_joint']

    @property
    def _z_idx(self) -> int:
        return SIDE_CFG[self.side]['z_motor']

    def set_target(self, target_xy: np.ndarray, side: str = 'left', cartype: int = 0):
        """Set the target position and initialize state machine.

        Args:
            target_xy: [x, y] target in robot base frame (meter, RH)
            side: 'left' or 'right'
            cartype: vehicle type index
        """
        self.macro_target_xy = np.asarray(target_xy[:2], dtype=float)
        self.micro_target_xy = None
        self.side = side
        self.topik.cartype = cartype
        self.vision.set_side(side)
        self.vision.reset()
        self.state = State.MACRO_ARM
        self._settle_timer = 0.0
        self._obs_buffer = []

    def step(self, world_raw: Optional[np.ndarray] = None,
                   current_c_pos: Optional[np.ndarray] = None) -> list:
        """Execute one control step.

        Args:
            world_raw: [x, y, z] hole position in world frame (from /cam0, m).
                       Macro uses the task-start snapshot. Micro latches a new
                       world target once at phase entry and then keeps following
                       that frozen target, instead of the live hole stream.
            current_c_pos: [7] actual motor positions in encoder counts, or None

        Returns:
            q_cmd_cnt: [7] motor position command in encoder counts (native int)
        """
        # Keep the state read path identical to the runtime FK chain:
        # JointState counts -> Topik.get_q() -> Topik.fk().
        # This avoids subtle drift between manual cnt2m/sign handling here
        # and the FK basis used to publish /cam0 world coordinates.

        actual_cnt = None
        if current_c_pos is not None:
            q_cnt = np.asarray(current_c_pos, dtype=int)
            actual_cnt = q_cnt.copy()
            self.topik.get_q(q_cnt)
            self.topik.fk()
            actual_q = self.topik.qm.copy()
            self._measured_q = actual_q.copy()

        # --- World-frame hole -> pin-to-hole error ---
        world_pin_err = None
        if self.state == State.MICRO_STAGE and world_raw is not None:
            if self.micro_target_xy is None:
                self.micro_target_xy = np.asarray(world_raw[:2], dtype=float)
                print(f"  [LATCH] MICRO target fixed at ({self.micro_target_xy[0]:.4f}, "
                      f"{self.micro_target_xy[1]:.4f}) m")
            world_pin_err = self.vision.get_target_error(self.micro_target_xy)
        elif self.state == State.VS_LIFT:
            if self.micro_target_xy is not None:
                world_pin_err = self.vision.get_target_error(self.micro_target_xy)
            elif world_raw is not None:
                world_pin_err = self.vision.get_pin_error(np.asarray(world_raw))
        
        # --- State Machine ---
        if self.state == State.MACRO_ARM:
            if self.cfg.enable_macro:
                self._step_macro()
            else:
                print("[SKIP] Phase 1 MACRO_ARM disabled by config.")
                self.state = State.MICRO_STAGE
        elif self.state == State.WAIT_MACRO:
            if current_c_pos is not None:
                self._step_wait_macro(actual_q, actual_cnt)
        elif self.state == State.MICRO_STAGE:
            if self.cfg.enable_micro:
                if world_pin_err is not None:
                    self._step_micro(world_pin_err)
            else:
                print("[SKIP] Phase 2 MICRO_STAGE disabled by config.")
                self.state = State.VS_LIFT
        elif self.state == State.WAIT_MICRO:
            if current_c_pos is not None:
                self._step_wait_micro(actual_q)
        elif self.state == State.VS_LIFT:
            if world_pin_err is not None:
                self._step_vs_lift(world_pin_err)
        elif self.state == State.DONE:
            pass  # Hold position

        self.q_cmd_cnt = self._q_cmd_to_cnt()
        return self.q_cmd_cnt

    # ----- Geometry Planner (Analytical 2-Link IK) -----

    def _geometry_plan_arm(self, target_xy: np.ndarray, side: str = 'left') -> Tuple[float, float, bool, str, list]:
        return self.kin.geometry_plan_arm(
            target_xy=np.asarray(target_xy[:2], dtype=float),
            side=side,
            measured_q=self._measured_q.copy(),
            hw=self.hw,
        )

    # ----- State handlers -----

    def _step_macro(self):
        """Phase 1: MACRO_ARM -Analytical Geometry Planner (2-Link IK)"""
        target = self.macro_target_xy.copy()
        wi = self._wing_idx  # 3 (Lr) or 4 (Rr)

        print("\n" + "="*60)
        print(f"[PHASE 1] MACRO_ARM -Geometry Planner ({self.side})")
        print("="*60)

        try:
            rtZ, wing, clamped, branch_name, candidates = self._geometry_plan_arm(target, self.side)
            self._selected_arm_branch = branch_name

            # Build q_cmd from the latest measured FK state.
            # FK Verify must use the same current stage offsets seen in
            # check_fk / topik.x, otherwise the "verify" pose is evaluated
            # at a different origin than the real robot.
            required_q = self._measured_q.copy()
            required_q[2] = rtZ
            required_q[wi] = wing

            required_q = self.hw.apply_safety_limits(required_q)
            target_cnt = self._q_math_to_cnt(required_q)

            # FK verification
            pin_reach = self.kin.fk_pin(required_q, self.side)
            residual = np.linalg.norm(target - pin_reach)

            print(f"  Side         : {self.side}")
            print(f"  Target       : ({target[0]:.4f}, {target[1]:.4f}) m")
            for cand in candidates:
                print(f"  IK Candidate  : branch_{cand['branch']}  "
                      f"rtZ={np.rad2deg(cand['rtZ']):.2f} deg, "
                      f"wing={np.rad2deg(cand['wing']):.2f} deg, "
                      f"res={cand['residual']*1000:.2f} mm, "
                      f"d_rtZ={np.rad2deg(cand['q2_delta']):.2f} deg, "
                      f"d_wing={np.rad2deg(cand['wing_delta']):.2f} deg")
            print(f"  Branch       : {branch_name}")
            print(f"  rtZ (base)   : {np.rad2deg(required_q[2]):.2f} deg")
            print(f"  Wing [q{wi}]  : {np.rad2deg(required_q[wi]):.2f} deg")
            print(f"  Target Cnt   : rtZ={target_cnt[2]}, wing={target_cnt[wi]}")
            print(f"  FK Verify    : ({pin_reach[0]:.4f}, {pin_reach[1]:.4f}) m")
            print(f"  Residual     : {residual*1000:.2f} mm")
            print(f"  Clamped      : {'YES' if clamped else 'NO'}")

            if clamped and residual > self.cfg.stage_limit:
                print(f"  [UNREACHABLE] Residual {residual*1000:.1f}mm > stage limit. Aborting.")
                self.state = State.DONE
                return
            elif clamped:
                print(f"  [CLAMPED->OK] Stage will compensate {residual*1000:.1f}mm.")

            self.q_cmd = required_q.copy()

            print(f"  [TRANSITION] MACRO_ARM -> WAIT_MACRO")
            self.state = State.WAIT_MACRO
            self._settle_timer = 0.0
            self._obs_buffer = []

        except Exception as e:
            print(f"  [ERROR] Geometry planner failed: {e}")
            import traceback; traceback.print_exc()
            self.state = State.DONE

    def _step_wait_macro(self, actual_q: np.ndarray, actual_cnt: np.ndarray):
        """Phase 1.5: WAIT_MACRO -Wait for physical joints to reach target."""
        if actual_q is None or actual_cnt is None:
            return

        wi = self._wing_idx
        rtz_tol_cnt = max(1, int(np.ceil(self.cfg.joint_settle_rad * self.topik.m2cnt[2])))
        wing_tol_cnt = max(1, int(np.ceil(self.cfg.joint_settle_rad * self.topik.m2cnt[wi])))

        err_rtz_cnt = abs(int(self.q_cmd_cnt[2]) - int(actual_cnt[2]))
        err_wing_cnt = abs(int(self.q_cmd_cnt[wi]) - int(actual_cnt[wi]))

        self._settle_timer += self.cfg.dt

        if err_rtz_cnt <= rtz_tol_cnt and err_wing_cnt <= wing_tol_cnt:
            print(f"  [MACRO SETTLED] Arm reached target in {self._settle_timer:.2f}s.")
            print(f"  [TRANSITION] WAIT_MACRO -> MICRO_STAGE")
            self.state = State.MICRO_STAGE
            self.micro_target_xy = None
            self._obs_buffer = []
            self.vision.reset()
            self._settle_timer = 0.0
        elif self._settle_timer > self.cfg.settle_timeout:
            err_rtz = abs(self.q_cmd[2] - actual_q[2])
            err_wing = abs(self.q_cmd[wi] - actual_q[wi])
            print(f"  [TIMEOUT] Arm settle fail. "
                  f"rtZ={err_rtz:.3f}rad ({err_rtz_cnt} cnt), "
                  f"wing={err_wing:.3f}rad ({err_wing_cnt} cnt)")
            print(f"           target_cnt rtZ/wing=({self.q_cmd_cnt[2]}, {self.q_cmd_cnt[wi]}), "
                  f"actual_cnt=({int(actual_cnt[2])}, {int(actual_cnt[wi])})")
            self.state = State.DONE

    def _step_micro(self, raw_pin_err: np.ndarray):
        """Phase 2: MICRO_STAGE -frozen-target observation and stage correction."""
        
        # 1. Gather pin-to-frozen-target errors
        self._obs_buffer.append(raw_pin_err)
        
        # Keep observing until we reach the required number of frames
        if len(self._obs_buffer) < self.cfg.micro_obs_frames:
            return
            
        # 2. We collected enough frames. Analyze.
        obs_array = np.array(self._obs_buffer)
        mean_err = np.mean(obs_array, axis=0)      # [dx, dy] mean
        std_err = np.std(obs_array, axis=0)        # [dx, dy] std dev
        max_std = np.max(std_err)
        
        # Clear buffer for the next observation cycle (if needed)
        self._obs_buffer = []
        
        err_norm = np.linalg.norm(mean_err)
        print(f"\n  [MICRO OBS] Err={err_norm*1000:.2f}mm, StdDev={max_std*1000:.2f}mm")
        
        # 3. Variance / Reliability Check
        if max_std > self.cfg.micro_std_limit:
            print(f"\n{'='*60}")
            print(f"[ERROR] Target variance is too high (StdDev: {max_std*1000:.1f}mm)!")
            print(f"  Suspicion: Camera Calibration Error or Kinematics (FK) Error.")
            print(f"             Or physical collision shifting the stage/target.")
            print(f"  Aborting for safety.")
            print(f"{'='*60}")
            self.state = State.DONE
            return

        # 4. Convergence Check
        if err_norm < self.cfg.micro_threshold:
            print(f"\n{'='*60}")
            print(f"[PHASE 2] MICRO_STAGE -CONVERGED!")
            print(f"  Final Error  : {err_norm*1000:.3f} mm")
            print(f"  Stage X      : {self.q_cmd[0]*1000:.2f} mm")
            print(f"  Stage Y      : {self.q_cmd[1]*1000:.2f} mm")
            print(f"  Wait Time    : {self._settle_timer:.2f} sec")
            print(f"{'='*60}")
            print(f"  [TRANSITION] MICRO_STAGE -> VS_LIFT")
            self.state = State.VS_LIFT
            return
            
        # Phase 2 uses stage-only translation.
        # We map the world-frame residual directly into stage motion here.
        
        # mean_err = Target World - Current World (m)
        err_x = mean_err[0]
        err_y = mean_err[1]
        
        # Build a proportional move in world coordinates.
        step_x = err_x * self.cfg.pbvs_lambda
        step_y = err_y * self.cfg.pbvs_lambda

        # Small residuals still need a decisive move, but preserve the sign.
        if abs(step_x) < 0.005:
            step_x = err_x * 0.95
        if abs(step_y) < 0.005:
            step_y = err_y * 0.95

        disp = np.hypot(step_x, step_y)
        max_disp = self.cfg.max_stage_vel * self.cfg.dt * self.cfg.micro_obs_frames
        if disp > max_disp:
            scale = max_disp / disp
            step_x *= scale
            step_y *= scale
        
        print(f"  [MICRO OBS] Target Error: dX={err_x*1000:.1f}mm, dY={err_y*1000:.1f}mm")
        
        # Sign mapping:
        # q_cmd[0] follows the world X convention directly.
        # q_cmd[1] is inverted relative to math-space world Y.
        # Subtracting step_y keeps the physical stage motion consistent.

        # Axis 0 is stage X and axis 1 is stage Y in math-space notation.
        # If motor wiring changes, update the signs here only.
        # The FK and Jacobian conventions should stay unchanged.
        self.q_cmd[0] += step_x  # Axis 0 Translation
        self.q_cmd[1] -= step_y  # Axis 1 Translation
        
        # Apply Safety Limits
        self.q_cmd = self.hw.apply_safety_limits(self.q_cmd)
        
        print(f"  [MICRO MOVE] Applied Offset (Math): axis0={step_x*1000:.1f}mm, axis1={step_y*1000:.1f}mm")
        
        self.state = State.WAIT_MICRO
        self._settle_timer = 0.0


    def _step_wait_micro(self, actual_q: np.ndarray):
        """Phase 2.5: WAIT_MICRO -Wait for physical stage to reach new command."""
        if actual_q is None: return
        err_x = abs(self.q_cmd[0] - actual_q[0])
        err_y = abs(self.q_cmd[1] - actual_q[1])
        
        self._settle_timer += self.cfg.dt
        
        if err_x < self.cfg.stage_settle_m and err_y < self.cfg.stage_settle_m:
            # Let it mechanically bounce-settle for an extra 0.2 sec
            if self._settle_timer >= 0.2:
                # settled. Go back to Look phase.
                self.state = State.MICRO_STAGE
                self._obs_buffer = [] # Start fresh look
                self.vision.reset()
                self._settle_timer = 0.0
        elif self._settle_timer > self.cfg.settle_timeout:
            print(f"  [TIMEOUT] Stage failed to settle. Error: dX={err_x*1000:.1f}mm, dY={err_y*1000:.1f}mm")
            self.state = State.DONE

    def _step_vs_lift(self, current_vision_xy: np.ndarray):
        """Phase 3 placeholder for coupled Z-lift and PBVS.

        Intended behavior:
        1. Raise the pin pair with the Z actuator.
        2. Correct XY residual caused by contact while lifting.
        3. Escalate from stage-only correction to coupled 4-DOF PBVS.
        4. Prefer stage motion first and use arm motion when needed.
        5. Finish once the insertion height target is reached safely.

        This is still a stub in the current implementation.
        """
        # TODO: implement Z lift plus coupled XY PBVS during insertion.

        print(f"\n[PHASE 3] VS_LIFT Stub ({self.side}). Pin-Hole aligned. DONE.")
        self.state = State.DONE
    # ----- Utility -----

    def _q_cmd_to_cnt(self) -> list:
        """Convert q_cmd (m, rad) to encoder counts.
        
        q_cmd is in MATH space (CCW = +).
        HW motors use CW = + for rotation axes [2,3,4].
        So we negate rotation axes here, mirroring get_q()'s read-path negation.
        
        Returns native Python int list (NOT numpy int64!)
        because Fastech protocol.py calls position.to_bytes()
        which only works on native Python int.
        """
        q_hw = self.q_cmd.copy()
        # Math->HW: invert rotation axes (mirrors get_q's HW->Math negation)
        q_hw[2] = -q_hw[2]
        q_hw[3] = -q_hw[3]
        q_hw[4] = -q_hw[4]
        cnt = [0] * 7
        for i in range(7):
            cnt[i] = int(np.round(q_hw[i] * self.topik.m2cnt[i]))
        return cnt

    def _q_math_to_cnt(self, q_math: np.ndarray) -> list:
        """Convert an arbitrary math-space q vector to hardware counts."""
        q_hw = q_math.copy()
        q_hw[2] = -q_hw[2]
        q_hw[3] = -q_hw[3]
        q_hw[4] = -q_hw[4]
        return [int(np.round(q_hw[i] * self.topik.m2cnt[i])) for i in range(7)]

    @property
    def is_done(self) -> bool:
        return self.state == State.DONE


# =============================================================================
# Module D: HardwareInterface
# =============================================================================
class HardwareInterface:
    """Interface between low-level PBVS workers and physical servo motors.
    
    Converts q_cmd (radian, meter) -> encoder count commands
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

        # Stage limits from home.
        sl = self.cfg.stage_limit  # 0.07m = 7cm
        q_safe[0] = np.clip(q_safe[0], -sl, sl)   # trY (X forward)
        q_safe[1] = np.clip(q_safe[1], -sl, sl)    # trX (Y left)

        # Base rotation limit.
        q_safe[2] = np.clip(q_safe[2], -0.325, 0.065)    # rtZ

        # Wing rotation limits.
        q_safe[3] = np.clip(q_safe[3], -1.7, 1.025)      # Lr
        q_safe[4] = np.clip(q_safe[4], -1.7, 1.318)      # Rr

        # Pin height limits (always >= 0)
        q_safe[5] = max(q_safe[5], 0.0)
        q_safe[6] = max(q_safe[6], 0.0)

        return q_safe


class DualState(Enum):
    MACRO_BOTH = auto()
    WAIT_MACRO = auto()
    MICRO_BOTH = auto()
    WAIT_MICRO = auto()
    DONE = auto()


class DualSideController:
    """Dual-pin controller that generates motor commands from solver outputs."""

    VISION_TIMEOUT_CYCLES = 100

    def __init__(self, topik: ik.Topik, cfg: ControlConfig):
        self.topik = topik
        self.cfg = cfg
        self.kin = KinematicsSolver(topik)
        self.hw = HardwareInterface(topik, cfg)

        self.state = DualState.DONE
        self.finish_reason = FinishReason.SUCCESS
        self.q_cmd = np.zeros(7)
        self.q_cmd_cnt = [0] * 7
        self._measured_q = np.zeros(7)
        self._settle_timer = 0.0
        self._vision_miss = 0
        self._obs_buf_l: List[np.ndarray] = []
        self._obs_buf_r: List[np.ndarray] = []

        self.target_l = np.zeros(2)
        self.target_r = np.zeros(2)
        self.micro_target_l: Optional[np.ndarray] = None
        self.micro_target_r: Optional[np.ndarray] = None

        self.W_diag = np.array([cfg.w_stage, cfg.w_stage, cfg.w_arm, cfg.w_arm, cfg.w_arm])

    def set_targets(self, target_l: np.ndarray, target_r: np.ndarray, cartype: int = 0):
        self.target_l = np.asarray(target_l[:2], dtype=float)
        self.target_r = np.asarray(target_r[:2], dtype=float)
        self.micro_target_l = None
        self.micro_target_r = None
        self.topik.cartype = cartype
        self.state = DualState.MACRO_BOTH
        self.finish_reason = FinishReason.SUCCESS
        self._settle_timer = 0.0
        self._vision_miss = 0
        self._obs_buf_l.clear()
        self._obs_buf_r.clear()

    def step(self, world_raw_l: Optional[np.ndarray] = None,
             world_raw_r: Optional[np.ndarray] = None,
             current_c_pos: Optional[np.ndarray] = None) -> list:
        actual_cnt = None
        if current_c_pos is not None:
            q_cnt = np.asarray(current_c_pos, dtype=int)
            actual_cnt = q_cnt.copy()
            self.topik.get_q(q_cnt)
            self.topik.fk()
            self._measured_q = self.topik.qm.copy()

        err_l = err_r = None
        if self.state == DualState.MICRO_BOTH:
            if world_raw_l is not None and world_raw_r is not None:
                self._vision_miss = 0
                if self.micro_target_l is None:
                    self.micro_target_l = np.asarray(world_raw_l[:2], dtype=float)
                    self.micro_target_r = np.asarray(world_raw_r[:2], dtype=float)
                    print(f"  [LATCH] L=({self.micro_target_l[0]:.4f}, {self.micro_target_l[1]:.4f}), "
                          f"R=({self.micro_target_r[0]:.4f}, {self.micro_target_r[1]:.4f})")
                err_l = self._pin_error(0, self.micro_target_l)
                err_r = self._pin_error(1, self.micro_target_r)
            else:
                self._vision_miss += 1
                if self._vision_miss > self.VISION_TIMEOUT_CYCLES:
                    missing = []
                    if world_raw_l is None:
                        missing.append('L')
                    if world_raw_r is None:
                        missing.append('R')
                    print(f"  [VISION TIMEOUT] No dual-camera data for "
                          f"{self._vision_miss} cycles. Missing: {missing}")
                    self._finish(FinishReason.FAILED_VISION)

        if self.state == DualState.MACRO_BOTH:
            self._step_macro()
        elif self.state == DualState.WAIT_MACRO:
            if actual_cnt is not None:
                self._step_wait_macro(actual_cnt)
        elif self.state == DualState.MICRO_BOTH:
            if err_l is not None and err_r is not None:
                self._step_micro(err_l, err_r)
        elif self.state == DualState.WAIT_MICRO:
            if current_c_pos is not None:
                self._step_wait_micro()

        self.q_cmd_cnt = self._q_cmd_to_cnt()
        return self.q_cmd_cnt

    @property
    def is_done(self) -> bool:
        return self.state == DualState.DONE

    @property
    def succeeded(self) -> bool:
        return self.is_done and self.finish_reason == FinishReason.SUCCESS

    def _finish(self, reason: FinishReason):
        self.finish_reason = reason
        self.state = DualState.DONE

    def _pin_error(self, fk_idx: int, target_xy: np.ndarray) -> np.ndarray:
        pin = np.array(self.topik.x[fk_idx][:2], dtype=float)
        return np.asarray(target_xy, dtype=float) - pin

    def _step_macro(self):
        print("\n" + "=" * 60)
        print("[DUAL PHASE 1] MACRO_BOTH -Simultaneous IK")
        print("=" * 60)

        q_sol = None
        branch_name = ""
        for flip in (False, True):
            q_sol = self.kin.try_dual_ik_branch(
                self.hw, self.cfg,
                self.target_l, self.target_r,
                self.topik.cartype, flip,
            )
            if q_sol is not None:
                branch_name = 'flipped' if flip else 'default'
                break

        if q_sol is None:
            print("  [ERROR] No feasible IK branch found.")
            self._finish(FinishReason.FAILED_IK)
            return

        pin_l, pin_r = self.kin.fk_dual(q_sol)
        res_l = np.linalg.norm(self.target_l - pin_l)
        res_r = np.linalg.norm(self.target_r - pin_r)
        print(f"  Branch       : {branch_name}")
        print(f"  L residual   : {res_l*1000:.2f} mm")
        print(f"  R residual   : {res_r*1000:.2f} mm")

        self.q_cmd = q_sol.copy()
        self.state = DualState.WAIT_MACRO
        self._settle_timer = 0.0

    def _step_wait_macro(self, actual_cnt: np.ndarray):
        target_cnt = self.q_cmd_cnt
        tols = [
            max(1, int(np.ceil(self.cfg.stage_settle_m * self.topik.m2cnt[i]))) if i < 2
            else max(1, int(np.ceil(self.cfg.joint_settle_rad * self.topik.m2cnt[i])))
            for i in range(5)
        ]
        settled = all(abs(int(target_cnt[i]) - int(actual_cnt[i])) <= tols[i] for i in range(5))
        self._settle_timer += self.cfg.dt

        if settled:
            print(f"  [MACRO SETTLED] {self._settle_timer:.2f}s -> MICRO_BOTH")
            self.state = DualState.MICRO_BOTH
            self.micro_target_l = None
            self.micro_target_r = None
            self._obs_buf_l.clear()
            self._obs_buf_r.clear()
            self._settle_timer = 0.0
            self._vision_miss = 0
        elif self._settle_timer > self.cfg.settle_timeout:
            errs = [abs(int(target_cnt[i]) - int(actual_cnt[i])) for i in range(5)]
            print(f"  [TIMEOUT] cnt errors: {errs}")
            self._finish(FinishReason.FAILED_TIMEOUT)

    def _step_micro(self, err_l: np.ndarray, err_r: np.ndarray):
        self._obs_buf_l.append(err_l.copy())
        self._obs_buf_r.append(err_r.copy())
        if len(self._obs_buf_l) < self.cfg.micro_obs_frames:
            return

        mean_l = np.mean(self._obs_buf_l, axis=0)
        mean_r = np.mean(self._obs_buf_r, axis=0)
        max_std = max(
            np.max(np.std(self._obs_buf_l, axis=0)),
            np.max(np.std(self._obs_buf_r, axis=0)),
        )
        self._obs_buf_l.clear()
        self._obs_buf_r.clear()

        norm_l = float(np.linalg.norm(mean_l))
        norm_r = float(np.linalg.norm(mean_r))
        print(f"\n  [MICRO OBS] L={norm_l*1000:.2f}mm  R={norm_r*1000:.2f}mm  std={max_std*1000:.2f}mm")

        if max_std > self.cfg.micro_std_limit:
            print(f"  [ERROR] Variance {max_std*1000:.1f}mm > limit. Aborting.")
            self._finish(FinishReason.FAILED_VARIANCE)
            return

        if norm_l < self.cfg.micro_threshold and norm_r < self.cfg.micro_threshold:
            print(f"  [CONVERGED] L={norm_l*1000:.3f}mm  R={norm_r*1000:.3f}mm")
            self._finish(FinishReason.SUCCESS)
            return

        dt_obs = self.cfg.dt * self.cfg.micro_obs_frames
        dq = self.kin.solve_dual_pbvs_step(
            measured_q=self._measured_q.copy(),
            mean_l=mean_l,
            mean_r=mean_r,
            W_diag=self.W_diag,
            pbvs_lambda=self.cfg.pbvs_lambda,
            max_stage_step=self.cfg.max_stage_vel * dt_obs,
            max_arm_step=self.cfg.max_arm_vel * dt_obs,
        )

        self.q_cmd[:5] += dq
        self.q_cmd = self.hw.apply_safety_limits(self.q_cmd)

        print(f"  [MICRO MOVE] stage=({dq[0]*1000:.1f}, {dq[1]*1000:.1f})mm  "
              f"rtZ={np.rad2deg(dq[2]):.2f}deg  Lr={np.rad2deg(dq[3]):.2f}deg  "
              f"Rr={np.rad2deg(dq[4]):.2f}deg")

        self.state = DualState.WAIT_MICRO
        self._settle_timer = 0.0

    def _step_wait_micro(self):
        aq = self._measured_q
        err_s = max(abs(self.q_cmd[0] - aq[0]), abs(self.q_cmd[1] - aq[1]))
        err_a = max(abs(self.q_cmd[i] - aq[i]) for i in range(2, 5))
        self._settle_timer += self.cfg.dt

        if err_s < self.cfg.stage_settle_m and err_a < self.cfg.joint_settle_rad:
            if self._settle_timer >= 0.2:
                self.state = DualState.MICRO_BOTH
                self._obs_buf_l.clear()
                self._obs_buf_r.clear()
                self._settle_timer = 0.0
                self._vision_miss = 0
        elif self._settle_timer > self.cfg.settle_timeout:
            print(f"  [TIMEOUT] stage={err_s*1000:.1f}mm  arm={np.rad2deg(err_a):.2f}deg")
            self._finish(FinishReason.FAILED_TIMEOUT)

    def _q_cmd_to_cnt(self) -> list:
        q_hw = self.q_cmd.copy()
        q_hw[2] = -q_hw[2]
        q_hw[3] = -q_hw[3]
        q_hw[4] = -q_hw[4]
        return [int(np.round(q_hw[i] * self.topik.m2cnt[i])) for i in range(7)]

