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
from robot_test.hardware import HardwareInterface
from robot_test.jacobian_solver import SIDE_CFG, KinematicsSolver


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

    # EMA filter coefficient for vision noise (0 < alpha <= 1, smaller = more smooth)
    ema_alpha: float = 0.3
    diff_ema_alpha: float = 0.1

    # Dual-pin modal control thresholds
    diff_deadband: float = 0.0015
    q2_search_range: float = np.pi / 6.0
    q2_search_steps: int = 25

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


class DualState(Enum):
    MACRO_DIFF = auto()
    WAIT_MACRO = auto()
    MICRO_COMMON = auto()
    WAIT_MICRO = auto()
    DONE = auto()


class DualSideController:
    """Dual-pin controller with modal macro/micro decomposition."""

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
        self._have_measured_q = False
        self._settle_timer = 0.0
        self._vision_miss = 0

        self.target_l = np.zeros(2)
        self.target_r = np.zeros(2)
        self._c_target = np.zeros(2)
        self._d_target = np.zeros(2)
        self._c_latest_vision: Optional[np.ndarray] = None
        self._d_latest_vision: Optional[np.ndarray] = None
        self._vision_initialized = False

    def set_targets(self, target_l: np.ndarray, target_r: np.ndarray, cartype: int = 0):
        c_target, d_target = self.kin.mode_decompose(target_l, target_r)
        self.set_modal_targets(c_target, d_target, target_l=target_l, target_r=target_r, cartype=cartype)

    def set_modal_targets(self, c_target: np.ndarray, d_target: np.ndarray,
                          target_l: Optional[np.ndarray] = None,
                          target_r: Optional[np.ndarray] = None,
                          cartype: int = 0):
        self._c_target = np.asarray(c_target[:2], dtype=float)
        self._d_target = np.asarray(d_target[:2], dtype=float)
        if target_l is None or target_r is None:
            self.target_l, self.target_r = self.kin.mode_compose(self._c_target, self._d_target)
        else:
            self.target_l = np.asarray(target_l[:2], dtype=float)
            self.target_r = np.asarray(target_r[:2], dtype=float)
        self.topik.cartype = cartype
        self.state = DualState.MACRO_DIFF
        self.finish_reason = FinishReason.SUCCESS
        self._settle_timer = 0.0
        self._vision_miss = 0
        self._c_latest_vision = self._c_target.copy()
        self._d_latest_vision = self._d_target.copy()
        self._vision_initialized = True
        self._measured_q = self.topik.qm.copy()
        self._have_measured_q = True
        self.q_cmd = self.topik.qm.copy()
        self.q_cmd_cnt = self._q_cmd_to_cnt()

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
            self._have_measured_q = True

        if world_raw_l is not None and world_raw_r is not None:
            self._update_modal_vision(world_raw_l, world_raw_r)
            self._vision_miss = 0
        elif self.state in (DualState.MICRO_COMMON, DualState.WAIT_MICRO):
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

        if self.state == DualState.MACRO_DIFF:
            self._step_macro()
        elif self.state == DualState.WAIT_MACRO:
            if actual_cnt is not None:
                self._step_wait_macro(actual_cnt)
        elif self.state == DualState.MICRO_COMMON:
            self._step_micro()
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

    @property
    def current_c_target(self) -> np.ndarray:
        if self._c_latest_vision is not None:
            return self._c_latest_vision.copy()
        return self._c_target.copy()

    @property
    def current_d_target(self) -> np.ndarray:
        if self._d_latest_vision is not None:
            return self._d_latest_vision.copy()
        return self._d_target.copy()

    def _finish(self, reason: FinishReason):
        self.finish_reason = reason
        self.state = DualState.DONE

    def _current_q(self) -> np.ndarray:
        return self._measured_q.copy() if self._have_measured_q else self.q_cmd.copy()

    def _compute_modal(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        return self.kin.modal_fk(np.asarray(q, dtype=float))

    def _compute_c(self, q: np.ndarray) -> np.ndarray:
        c_now, _ = self._compute_modal(q)
        return c_now

    def _compute_d(self, q: np.ndarray) -> np.ndarray:
        _, d_now = self._compute_modal(q)
        return d_now

    def _update_modal_vision(self, world_raw_l: np.ndarray, world_raw_r: np.ndarray):
        raw_c, raw_d = self.kin.mode_decompose(world_raw_l[:2], world_raw_r[:2])
        if not self._vision_initialized:
            self._c_latest_vision = raw_c.copy()
            self._d_latest_vision = raw_d.copy()
            self._vision_initialized = True
        else:
            self._c_latest_vision = (
                self.cfg.ema_alpha * raw_c
                + (1.0 - self.cfg.ema_alpha) * self._c_latest_vision
            )
            self._d_latest_vision = (
                self.cfg.diff_ema_alpha * raw_d
                + (1.0 - self.cfg.diff_ema_alpha) * self._d_latest_vision
            )
        self._c_target = self._c_latest_vision.copy()

    def _step_macro(self):
        print("\n" + "=" * 60)
        print("[DUAL PHASE 1] MACRO_DIFF -Differential IK")
        print("=" * 60)

        if self._d_latest_vision is not None:
            self._d_target = self._d_latest_vision.copy()

        macro = self.kin.solve_macro_differential(
            d_target=self._d_target,
            q_current=self._current_q(),
            hw=self.hw,
            cfg=self.cfg,
            c_target=self._c_latest_vision if self._c_latest_vision is not None else self._c_target,
        )
        if macro is None:
            print("  [ERROR] No feasible differential IK solution found.")
            self._finish(FinishReason.FAILED_IK)
            return

        q_sol = macro['q_cmd'].copy()
        c_sol, d_sol = self._compute_modal(q_sol)
        print(f"  d_target     : ({self._d_target[0]:.4f}, {self._d_target[1]:.4f}) m")
        print(f"  d_macro      : ({d_sol[0]:.4f}, {d_sol[1]:.4f}) m")
        print(f"  Branch       : {macro['branch']}")
        print(f"  rtZ          : {np.rad2deg(q_sol[2]):.2f} deg")
        print(f"  Lr / Rr      : {np.rad2deg(q_sol[3]):.2f} / {np.rad2deg(q_sol[4]):.2f} deg")
        print(f"  c_macro      : ({c_sol[0]:.4f}, {c_sol[1]:.4f}) m")
        print(f"  |e_d|        : {macro['residual']*1000:.2f} mm")

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
            print(f"  [MACRO SETTLED] {self._settle_timer:.2f}s -> MICRO_COMMON")
            self.state = DualState.MICRO_COMMON
            self._settle_timer = 0.0
            self._vision_miss = 0
        elif self._settle_timer > self.cfg.settle_timeout:
            errs = [abs(int(target_cnt[i]) - int(actual_cnt[i])) for i in range(5)]
            print(f"  [TIMEOUT] cnt errors: {errs}")
            self._finish(FinishReason.FAILED_TIMEOUT)

    def _step_micro(self):
        if self._c_latest_vision is None or self._d_latest_vision is None:
            return

        d_err = self._d_latest_vision - self._compute_d(self.q_cmd)
        d_err_norm = float(np.linalg.norm(d_err))
        if d_err_norm > self.cfg.diff_deadband:
            print(f"  [DIFF GUARD] |e_d|={d_err_norm*1000:.2f}mm -> re-enter MACRO_DIFF")
            self._d_target = self._d_latest_vision.copy()
            self.state = DualState.MACRO_DIFF
            return

        c_now = self._compute_c(self._current_q())
        c_err = self._c_latest_vision - c_now
        c_err_norm = float(np.linalg.norm(c_err))
        print(f"\n  [MICRO COMMON] |e_c|={c_err_norm*1000:.2f}mm  |e_d|={d_err_norm*1000:.2f}mm")

        if c_err_norm < self.cfg.micro_threshold:
            self._finish(FinishReason.SUCCESS)
            return

        stage_step = self.cfg.pbvs_lambda * c_err
        max_step = self.cfg.max_stage_vel * self.cfg.dt
        step_norm = float(np.linalg.norm(stage_step))
        if step_norm > max_step > 0.0:
            stage_step *= max_step / step_norm

        self.q_cmd[0] += stage_step[0]
        self.q_cmd[1] -= stage_step[1]
        self.q_cmd = self.hw.apply_safety_limits(self.q_cmd)

        print(f"  [MICRO MOVE] dq_stage=({stage_step[0]*1000:.1f}, {stage_step[1]*1000:.1f})mm")

        self.state = DualState.WAIT_MICRO
        self._settle_timer = 0.0

    def _step_wait_micro(self):
        aq = self._measured_q
        err_s = max(abs(self.q_cmd[0] - aq[0]), abs(self.q_cmd[1] - aq[1]))
        self._settle_timer += self.cfg.dt

        if err_s < self.cfg.stage_settle_m:
            if self._settle_timer >= 0.2:
                self.state = DualState.MICRO_COMMON
                self._settle_timer = 0.0
                self._vision_miss = 0
        elif self._settle_timer > self.cfg.settle_timeout:
            print(f"  [TIMEOUT] stage={err_s*1000:.1f}mm")
            self._finish(FinishReason.FAILED_TIMEOUT)

    def _q_cmd_to_cnt(self) -> list:
        q_hw = self.q_cmd.copy()
        q_hw[2] = -q_hw[2]
        q_hw[3] = -q_hw[3]
        q_hw[4] = -q_hw[4]
        return [int(np.round(q_hw[i] * self.topik.m2cnt[i])) for i in range(7)]

