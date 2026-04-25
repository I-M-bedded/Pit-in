import time
import sys
import os
import numpy as np
import csv
import cv2

# Project root for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")

try:
    from controller import ik
except ImportError:
    pass

from robot_test.jacobian_solver import ReachMode
from robot_test.macro_micro import FinishReason

try:
    import pyrealsense2 as rs
    from vision.dataset_gen.auto_annotator import AutoAnnotator, write_yolo_pose_label, MARKER_CENTERS, MIN_MARKERS_REQUIRED
    HAS_VISION_GEN = True
except ImportError:
    HAS_VISION_GEN = False
    print("[WARNING] Vision dataset generation modules not found.")

class BaseTask:
    def __init__(self, robot):
        self.robot = robot
        self.config = robot.config
        self.is_active = False
        self.mode = 0
        self.step = 0
        self.timer = 0.0
        self.flag = False
        self.factor = self.config.factor_distance

    def reset(self):
        self.is_active = False
        self.mode = 0
        self.step = 0
        self.flag = False
    
class SystemMonitor:
    def __init__(self, robot):
        self.robot = robot
        self.config = robot.config
        self.start_time = time.time()
        self.was_home_checked = False

    def update(self):
        self._check_threads()
        self._check_home_status()
        self._check_task_clear()

    def _check_threads(self):
        thread_total = sum(self.robot.thereadcheck[i+1] for i in range(7))
        if thread_total == 7:
            self.robot.agv.lift_state_ros |= 0x0008
            self.robot.agv.thread_check_total = True
        else:
            self.robot.agv.lift_state_ros &= 0xfff7
            self.robot.agv.thread_check_total = False

    def _check_home_status(self):
        if not self.was_home_checked:
            if time.time() - self.start_time > 2.0:
                self.was_home_checked = True
                is_all_homed = all(s.is_originfinding_ok for s in self.robot.servos)
                self.robot.was_home = is_all_homed
                if is_all_homed:
                    self.robot.agv.lift_state_ros |= 0x0004
                else:
                    self.robot.agv.lift_state_ros &= 0xfffb

    def _check_task_clear(self):
        if self.robot.agv.task_clear_top:
            self.robot.agv.task_clear_top = False
            self.robot.agv.presetnum = 0

class HomingTask(BaseTask):
    """Top-module homing sequence with a stepwise recovery flow."""
    def start(self):
        if not self.is_active:
            self.is_active = True; self.step = 0; self.flag = False
            self.robot.agv.master_homming = True
            print("Homing Task Started")

    def run(self):
        if not self.is_active: return
        cfg = self.config
        servos = self.robot.servos
        
        if self.step == 0:
            if not self.flag: self.timer = time.time(); self.flag = True
            # Keep axis 5 servo-off during the first homing step.
            # Original sequence was [2, 2, 1, 3, 3, 2, 2].
            self.robot.t_action = [2, 2, 1, 3, 3, 3, 2] 
            
            condition = servos[2].is_originfinding_ok if cfg.version != 1 else True
            if (time.time() - self.timer > cfg.homing_timeout_step_1) and condition:
                self.step = 1; self.flag = False; self.robot.t_pos[2] = 0

        elif self.step == 1:
            if not self.flag: self.timer = time.time(); self.flag = True
            # Keep axis 5 disabled during the second homing step as well.
            # Original sequence was [1, 1, 0, 3, 3, 1, 1].
            self.robot.t_action = [1, 1, 0, 3, 3, 3, 1]

            # Do not wait for servo 5 homing completion in this branch.
            # This path intentionally ignores the broken axis-5 origin signal.
            if (time.time() - self.timer > cfg.homing_timeout_step_1) and \
               servos[0].is_originfinding_ok and servos[1].is_originfinding_ok and \
               servos[6].is_originfinding_ok:
                self.step = 2; self.flag = False

        elif self.step == 2:
            if not self.flag: self.timer = time.time(); self.flag = True
            # Keep axis 5 disabled while the wing pair continues homing.
            # Original sequence was [0, 0, 0, 2, 2, 0, 0].
            self.robot.t_action = [0, 0, 0, 2, 2, 3, 0] 
            
            if time.time() - self.timer > getattr(cfg, 'homing_timeout_step_3', 3.0):
                self.step = 3; self.flag = False

        elif self.step == 3:
            if not self.flag: self.timer = time.time(); self.flag = True
            # Keep the same axis-5 protection during the next step.
            self.robot.t_action = [0, 0, 0, 1, 1, 3, 0]
            
            if (time.time() - self.timer > getattr(cfg, 'homing_timeout_step_4', 3.0)) and \
               servos[3].is_originfinding_ok and servos[4].is_originfinding_ok:
                self.step = 4; self.flag = False

        elif self.step == 4:
            if not self.flag: self.timer = time.time(); self.flag = True
            
            # Finalize while forcing axis 5 to stay servo-off.
            self.robot.t_action = [0]*7
            self.robot.t_action[5] = 3
            
            self.robot.t_pos = [0]*7
            # Preserve the current axis-5 position so it does not get pulled to zero.
            # This prevents an unintended move when t_action returns to position mode.
            self.robot.t_pos[5] = self.robot.c_pos[5] 

            if time.time() - self.timer > getattr(cfg, 'homing_timeout_step_3', 3.0):
                self.step = 5; self.flag = False

        else:
            # Final completion branch.
            self.robot.agv.op_state[0] = 255; self.robot.agv.presetnum = 0
            self.robot.was_home = True
            self.robot.agv.lift_state_ros |= 0x0004
            print(f"Homing Complete. Pos: {self.robot.c_pos[:5]}")
            self.reset()


class LoadingTask(BaseTask):
    """Top-module loading task."""
    def start_approach(self):
        if not self.is_active:
            self.is_active = True; self.mode = 1; self.flag = False
            self.robot.laser = True
            print("Approach Task Started")

    def start_load(self):
        if not self.is_active:
            self.is_active = True; self.mode = 2; self.flag = False
            print("Load Task Started")

    def run(self):
        if not self.is_active: return
        cfg = self.config
        
        if not self.flag: self.flag = True; self.timer = time.time()
        
        if self.mode == 1: # Approach
            self.robot.t_action = [0]*7
            toff_st = getattr(cfg, 'toff_st', [[0,0,0]]*6)
            qtemp = self.robot.topik.set_xd(self.robot.agv.cartype, toff_st[self.robot.agv.cartype])
            self.robot.t_pos[0:5] = qtemp[0:5]
            self.robot.t_pos[5] = cfg.pin_h_st_attach
            self.robot.t_pos[6] = cfg.pin_h_st_attach

        elif self.mode == 2: # Load
            self.robot.t_action = [0]*7
            if time.time() - self.timer < 3.0:
                for i in range(5): self.robot.t_pos[i] = self.robot.servos[i].com_apos
                self.robot.t_action[4] = 3
                pin_load_h = getattr(cfg, 'pin_h_st_load', 270000)
                self.robot.t_pos[5] = pin_load_h
                self.robot.t_pos[6] = pin_load_h

        if time.time() - self.timer > getattr(cfg, 'inching_timeout', 5.0):
            self.robot.agv.op_state[0] = 255; self.robot.agv.presetnum = 0
            self.reset()
            print("Loading Task Finished")

class ManualpinTask(BaseTask):
    """Manual top-module control task."""
    # Lift helper: n = axis index, dir = {1, -1, 0}, rot toggles rotation mode.
    def lift(self, n, dir, rot=0):
        self.robot.t_action[n] = 0
        if dir == 0:
            self.robot.t_pos[n] = self.robot.c_pos[n]
            return
        
        if rot:
            # Axis 2 Rotation (Original logic hardcoded 500)
            delta = self.config.factor_lift_rotate * self.factor 
            self.robot.t_pos[n] = self.robot.c_pos[n] + (delta * dir)
        else:
            # Axis 0, 1 Lift
            delta = self.config.factor_lift * self.factor
            self.robot.t_pos[n] = self.robot.c_pos[n] + (delta * dir)

    # Pin helper: n = axis index, dir = {1, -1, 0}, rot toggles rotation mode.
    def pin_control(self, n, dir, rot=0):
        self.robot.t_action[n] = 0
        if dir == 0:
            self.robot.t_pos[n] = self.robot.c_pos[n]
            return

        if rot:
            # Pin Rotation (CW/CCW)
            delta = self.config.factor_rotation * self.factor
            self.robot.t_pos[n] = self.robot.c_pos[n] + (delta * dir)
        else:
            # Pin Height (Up/Down)
            delta = self.config.factor_pin_adjust
            target = self.robot.servos[n].com_apos + (delta * dir)
            
            # Clamp the commanded pin height.
            if dir > 0:
                self.robot.t_pos[n] = min(target, self.config.max_pin_height_big)
            else:
                self.robot.t_pos[n] = max(target, 0)
    
    # Wing gap helper.
    def wing_gap(self, dir):
        # dir: 1 (Expand), -1 (Shorten)
        delta = 0.001 * dir
        new_gap = self.robot.pingap + delta
        new_gap = max(min(new_gap, self.robot.topik.dismax), self.robot.topik.dismin)
        self.robot.pingap = new_gap
        
        qd = self.robot.topik.get_wing_q(self.robot.pingap)
        self.robot.t_action[3]=0; self.robot.t_action[4]=0
        self.robot.t_pos[3] = int(qd[3]); self.robot.t_pos[4] = int(qd[4])
        print(f"Wing Gap: {self.robot.pingap:.3f}")

    # System helper: reset, buzzer stop, and servo on/off.
    def system_control(self, mode, val=0):
        if mode == 'reset':
             self.robot.reset = True
             self.robot.estop_joy = False
             self.robot.agv.estop_server2 = False
        elif mode == 'buzzer_stop':
             self.robot.buzzer_stop = True
        elif mode == 'servo':
             if val == 1: # On
                 self.robot.t_action = [2]*7
                 self.robot.drv_srv = True
                 self.robot.t_pos = [s.pos for s in self.robot.servos]
                 self.robot.manual_servo_flag = True
                 self.robot.is_fastech_on = True
             else: # Off
                 self.robot.t_action = [3]*7
                 self.robot.drv_srv = False
                 self.robot.manual_servo_flag = True
                 self.robot.is_fastech_on = False

# Vision and dataset capture utilities.
class VisionTask(BaseTask):
    def __init__(self, robot):
        super().__init__(robot)
        self.Lx = []; self.Ly = []; self.Rx = []; self.Ry = []
        self.abs_error = [1e10]*2
        self.repeat_time = 0
        self.target_mid_state = []
        self.pause_ratio = self.config.pause_ratio
        self.annotator = None
        self.save_dir = "./dataset_auto"
        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs(os.path.join(self.save_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(self.save_dir, "labels"), exist_ok=True)

    def run(self):
        inputs = self.robot.input_manager.get_state()
        # L + R trigger is handled in supervisor now, calling save_annotation()

        if inputs['L'] and not inputs['R']:
            self.robot.topik.get_q(self.robot.c_pos)
            self.robot.topik.fk()
            print("FK Check:", self.robot.topik.x[0][0], self.robot.topik.x[0][1], self.robot.agv.lcam_hole_pos)
    
    def save_annotation(self):
        """L + R trigger: Capture current frame and save YOLO Pose annotation."""
        if not HAS_VISION_GEN:
            print("[ERROR] Vision dataset generation modules not available.")
            return

        print("[INFO] Attempting to save annotation...")
        
        # 1. RealSense Capture
        pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        try:
            profile = pipeline.start(rs_config)
            
            # Get Intrinsics
            stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = stream.get_intrinsics()
            K = np.array([[intr.fx, 0, intr.ppx],
                          [0, intr.fy, intr.ppy],
                          [0, 0, 1]], dtype=np.float64)
            D = np.array(intr.coeffs, dtype=np.float64)
            
            # Capture Frame
            for _ in range(10): # Warm up
                pipeline.wait_for_frames()
            
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                print("[ERROR] Failed to capture color frame.")
                return
            
            frame = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # 2. ArUco Marker Detection
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) # Default type
            params = cv2.aruco.DetectorParameters()
            
            if hasattr(cv2.aruco, "ArucoDetector"):
                detector = cv2.aruco.ArucoDetector(aruco_dict, params)
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
                
            if ids is None or len(ids) < MIN_MARKERS_REQUIRED:
                print(f"[ERROR] Not enough markers detected ({len(ids) if ids is not None else 0}/{MIN_MARKERS_REQUIRED})")
                return
            
            image_points = []
            object_points = []
            for i, marker_id in enumerate(ids.flatten()):
                mid = int(marker_id)
                if mid in MARKER_CENTERS:
                    c = corners[i][0]
                    cx_m, cy_m = np.mean(c[:, 0]), np.mean(c[:, 1])
                    image_points.append([cx_m, cy_m])
                    object_points.append(MARKER_CENTERS[mid])
            
            if len(image_points) < MIN_MARKERS_REQUIRED:
                print(f"[ERROR] Required markers not in view.")
                return
                
            success, rvec, tvec = cv2.solvePnP(
                np.array(object_points, dtype=np.float32),
                np.array(image_points, dtype=np.float32),
                K, D, flags=cv2.SOLVEPNP_SQPNP
            )
            
            if not success:
                print("[ERROR] PnP failed.")
                return
                
            # 3. Compute and Save
            if self.annotator is None:
                self.annotator = AutoAnnotator(K, D, 640, 480)
            else:
                self.annotator.K = K
                self.annotator.D = D
                
            annotations, imgpts_dict = self.annotator.compute(rvec, tvec)
            
            if not annotations:
                print("[ERROR] No valid annotations generated.")
                return
                
            stem = f"robot_{int(time.time())}"
            img_path = os.path.join(self.save_dir, "images", f"{stem}.jpg")
            label_path = os.path.join(self.save_dir, "labels", f"{stem}.txt")
            
            cv2.imwrite(img_path, frame)
            write_yolo_pose_label(label_path, annotations)
            print(f"[SUCCESS] Saved image and YOLO Pose label: {stem}")
            
        except Exception as e:
            print(f"[ERROR] VisionTask.save_annotation: {e}")
        finally:
            pipeline.stop()
    
    def save_data(self):
        # 1. Update robot state and forward kinematics.
        self.robot.topik.get_q(self.robot.c_pos)
        self.robot.topik.fk()
        robot_yaw = self.robot.c_pos[3]*self.robot.topik.cnt2m[3]
        
        # 2. Assemble CSV header and robot pose.
        # Store the robot center pose followed by up to two marker records.
        header = [
            "robot_x", "robot_y", "robot_th",
            "m1_id", "m1_x", "m1_y", "m1_z", "m1_qx", "m1_qy", "m1_qz", "m1_qw",
            "m2_id", "m2_x", "m2_y", "m2_z", "m2_qx", "m2_qy", "m2_qz", "m2_qw",
        ]
        
        center_x = (self.robot.topik.x[0][0] + self.robot.topik.x[1][0]) / 2.0
        center_y = (self.robot.topik.x[0][1] + self.robot.topik.x[1][1]) / 2.0

        detected = getattr(self.robot.agv, 'detected_markers', {})
        if not isinstance(detected, dict):
            detected = {}

        # New ROS marker cache is nested by camera: {'cam0': {id: pose}, 'cam1': {id: pose}}.
        # Keep a tiny fallback for older flat caches so existing manual runs do not crash.
        cam_marker_sets = {
            'cam0': detected.get('cam0', {}) if isinstance(detected.get('cam0', {}), dict) else {},
            'cam1': detected.get('cam1', {}) if isinstance(detected.get('cam1', {}), dict) else {},
        }
        flat_markers = {
            k: v for k, v in detected.items()
            if isinstance(k, int) and isinstance(v, dict)
        }
        if flat_markers and not cam_marker_sets['cam0']:
            cam_marker_sets['cam0'] = flat_markers

        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        calibration_dir = os.path.join(repo_root, "vision", "calibration")
        os.makedirs(calibration_dir, exist_ok=True)

        saved = []
        for cam_name, markers in cam_marker_sets.items():
            sorted_ids = sorted(markers.keys())
            if not sorted_ids:
                continue

            marker_data = []
            for i in range(2):
                if i < len(sorted_ids):
                    m_id = sorted_ids[i]
                    m_info = markers[m_id]
                    marker_data += [m_id, m_info['x'], m_info['y'], m_info['z'],
                                    m_info['qx'], m_info['qy'], m_info['qz'], m_info['qw']]
                else:
                    marker_data += ["N/A", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

            row_data = [center_x, center_y, robot_yaw] + marker_data
            filename = os.path.join(calibration_dir, f"Calibration_data_{cam_name}.csv")
            file_exists = os.path.isfile(filename)

            with open(filename, 'a', newline='') as file:
                writer = csv.writer(file)
                if not file_exists or os.path.getsize(filename) == 0:
                    writer.writerow(header)
                writer.writerow(row_data)
            saved.append(f"{cam_name}:{sorted_ids}")

        if saved:
            print(f"[SUCCESS] Calibration data saved. {', '.join(saved)}")
        else:
            print("[WARN] Calibration data not saved: no cam0/cam1 markers detected.")

class _PbvsBase(BaseTask):
    """Single-side Macro-Micro PBVS task base.

    Vision node publishes hole position in the robot world frame on /cam0
    (left) / /cam1 (right). The orchestrator (TCPcontroller) is the single
    entry point: we force ONE_SIDE mode via ``set_single_target`` so that
    robot_main never touches the worker controllers directly.
    """
    SIDE: str = ''

    def __init__(self, robot):
        super().__init__(robot)
        self.tcp_ctrl = robot.pbvs_tcp_ctrl

    def start(self):
        if self.is_active:
            return
        world_hole = self._get_world_hole(self.SIDE)
        target_xy = world_hole[:2]

        self.tcp_ctrl.set_single_target(self.SIDE, target_xy, cartype=self.robot.agv.cartype)
        self.is_active = True
        self.robot.t_action = [0] * 7
        print(f"PBVS Task Started - {self.SIDE} pin (world target: {target_xy})")

    def _get_world_hole(self, side):
        if side == 'left':
            return np.array(self.robot.agv.lcam_hole_pos, dtype=float)
        else:
            return np.array(self.robot.agv.rcam_hole_pos, dtype=float)

    def run(self):
        if not self.is_active:
            return

        world_raw = self._get_world_hole(self.SIDE)
        c_pos = np.array(self.robot.c_pos, dtype=float)

        if self.SIDE == 'left':
            q_cnt = self.tcp_ctrl.step(world_raw_l=world_raw, current_c_pos=c_pos)
        else:
            q_cnt = self.tcp_ctrl.step(world_raw_r=world_raw, current_c_pos=c_pos)

        for i in range(7):
            self.robot.t_action[i] = 0
        for i in range(5):
            self.robot.t_pos[i] = q_cnt[i]

        if self.tcp_ctrl.is_done:
            reason = self.tcp_ctrl.finish_reason
            if reason == FinishReason.SUCCESS:
                print(f"PBVS {self.SIDE} pin Finished.")
                self.robot.agv.op_state[0] = 255
            else:
                print(f"PBVS {self.SIDE} pin FAILED ({reason.name}).")
                self.robot.agv.op_state[0] = 254
            self.reset()


class PbvsLTask(_PbvsBase):
    """PBVS for the left pin, triggered by LT+Y."""
    SIDE = 'left'


class PbvsRTask(_PbvsBase):
    """PBVS for the right pin, triggered by RT+Y."""
    SIDE = 'right'


class PbvsBothTask(BaseTask):
    """Dual-pin PBVS triggered by LT+RT."""

    def __init__(self, robot):
        super().__init__(robot)
        self.tcp_ctrl = robot.pbvs_tcp_ctrl
        self._mode = None
        self._targets = {}

    def start(self):
        if self.is_active:
            return

        target_l = np.array(self.robot.agv.lcam_hole_pos, dtype=float)
        target_r = np.array(self.robot.agv.rcam_hole_pos, dtype=float)
        self._targets = {'left': target_l[:2], 'right': target_r[:2]}

        cartype = self.robot.agv.cartype
        mode, info = self.tcp_ctrl.set_targets(target_l[:2], target_r[:2], cartype)

        print(f"\n[PBVS BOTH] Reachability: {mode.name}")
        for k, v in info.items():
            print(f"  {k}: {v}")

        self._mode = mode
        if mode in (ReachMode.BOTH, ReachMode.SEQUENTIAL, ReachMode.ONE_SIDE):
            self.is_active = True
            self.robot.t_action = [0] * 7
            print(f"[PBVS BOTH] Controller started in mode: {mode.name}")
        else:
            self.robot.agv.op_state[0] = 254
            print("[PBVS BOTH] ERROR - both pins unreachable!")

    def run(self):
        if not self.is_active:
            return

        wl = np.array(self.robot.agv.lcam_hole_pos, dtype=float)
        wr = np.array(self.robot.agv.rcam_hole_pos, dtype=float)
        cp = np.array(self.robot.c_pos, dtype=float)
        q_cnt = self.tcp_ctrl.step(world_raw_l=wl, world_raw_r=wr, current_c_pos=cp)

        for i in range(7):
            self.robot.t_action[i] = 0
        for i in range(5):
            self.robot.t_pos[i] = q_cnt[i]

        if self.tcp_ctrl.is_done:
            reason = self.tcp_ctrl.finish_reason
            if reason == FinishReason.SUCCESS:
                print(f"[PBVS BOTH] {self._mode.name} completed.")
                self.robot.agv.op_state[0] = 255
            else:
                print(f"[PBVS BOTH] FAILED ({reason.name})")
                self.robot.agv.op_state[0] = 254
            self.reset()

    def reset(self):
        super().reset()
        self._mode = None
