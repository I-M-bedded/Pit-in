import time
import sys
import os
import numpy as np
import csv

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
try:
    from sky import pos_target_oper
    from controller import ik
except ImportError:
    pass

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
    """상부모듈 호밍 작업 클래스 (원점 복귀)"""
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
            # [수정 1] 5번 인덱스를 2(Homing) -> 3(Servo Off)으로 변경
            # 원본: [2, 2, 1, 3, 3, 2, 2]
            self.robot.t_action = [2, 2, 1, 3, 3, 3, 2] 
            
            condition = servos[2].is_originfinding_ok if cfg.version != 1 else True
            if (time.time() - self.timer > cfg.homing_timeout_step_1) and condition:
                self.step = 1; self.flag = False; self.robot.t_pos[2] = 0

        elif self.step == 1:
            if not self.flag: self.timer = time.time(); self.flag = True
            # [수정 2] 5번 인덱스를 1(Origin) -> 3(Servo Off)으로 변경
            # 원본: [1, 1, 0, 3, 3, 1, 1]
            self.robot.t_action = [1, 1, 0, 3, 3, 3, 1]

            # [수정 3] servos[5].is_originfinding_ok 조건 삭제
            # 고장난 모터의 완료 신호를 기다리지 않도록 합니다.
            if (time.time() - self.timer > cfg.homing_timeout_step_1) and \
               servos[0].is_originfinding_ok and servos[1].is_originfinding_ok and \
               servos[6].is_originfinding_ok: # servos[5] 제거됨
                self.step = 2; self.flag = False

        elif self.step == 2:
            if not self.flag: self.timer = time.time(); self.flag = True
            # [수정 4] 5번 모터는 계속 3(Off) 혹은 움직이지 않도록 설정
            # 원본: [0, 0, 0, 2, 2, 0, 0] (0은 위치제어이므로 3으로 바꾸는게 안전함)
            self.robot.t_action = [0, 0, 0, 2, 2, 3, 0] 
            
            if time.time() - self.timer > getattr(cfg, 'homing_timeout_step_3', 3.0):
                self.step = 3; self.flag = False

        elif self.step == 3:
            if not self.flag: self.timer = time.time(); self.flag = True
            # [수정 5] 여기도 안전하게 3으로 변경
            self.robot.t_action = [0, 0, 0, 1, 1, 3, 0]
            
            if (time.time() - self.timer > getattr(cfg, 'homing_timeout_step_4', 3.0)) and \
               servos[3].is_originfinding_ok and servos[4].is_originfinding_ok:
                self.step = 4; self.flag = False

        elif self.step == 4:
            if not self.flag: self.timer = time.time(); self.flag = True
            
            # [수정 6] 마지막 원점 이동 시 5번 모터가 0으로 가는 것 방지
            self.robot.t_action = [0]*7
            self.robot.t_action[5] = 3 # 5번만 Servo Off 강제
            
            self.robot.t_pos = [0]*7
            # 만약 t_action이 0(위치제어)인데 t_pos가 0이면 모터가 강제로 0위치로 이동하려 듭니다.
            # 5번 모터는 현재 위치를 유지하거나 명령을 무시해야 합니다.
            self.robot.t_pos[5] = self.robot.c_pos[5] 

            if time.time() - self.timer > getattr(cfg, 'homing_timeout_step_3', 3.0):
                self.step = 5; self.flag = False

        else:
            # ... (기존 코드 동일) ...
            self.robot.agv.op_state[0] = 255; self.robot.agv.presetnum = 0
            self.robot.was_home = True
            self.robot.agv.lift_state_ros |= 0x0004
            print(f"Homing Complete. Pos: {self.robot.c_pos[:5]}")
            self.reset()


class LoadingTask(BaseTask):
    """상부모듈 적재 작업 클래스"""
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
    """상부모듈 수동 제어 클래스"""
    # [Lift 기능] - n: 축 번호, dir: 방향 (1: 증가, -1: 감소, 0: 정지), rot: 회전 모드 플래그
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

    # [Pin 기능] - n: 축 번호, dir: 방향 (1: 증가, -1: 감소, 0: 정지), rot: 회전 모드 플래그
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
            
            # 높이 제한
            if dir > 0:
                self.robot.t_pos[n] = min(target, self.config.max_pin_height_big)
            else:
                self.robot.t_pos[n] = max(target, 0)
    
    # [Wing Gap 기능]
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

    # [System 기능] - 리셋, 버저, 서보
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

#연구할 부분.
class VisionTask(BaseTask):
    def __init__(self, robot):
        super().__init__(robot)
        reacquire_rate = getattr(self.config, 'reacquire_rate', 100)
        self.pto = pos_target_oper.pos_target_oper(self.config.version, reacquire_rate)
        self.Lx = []; self.Ly = []; self.Rx = []; self.Ry = []
        self.abs_error = [1e10]*2
        self.repeat_time = 0
        self.target_mid_state = []
        self.pause_ratio = self.config.pause_ratio

    def run(self):
        inputs = self.robot.input_manager.get_state()
        if (inputs['R'] and inputs['L']) or self.pto.target_mode:
            pass 

        if inputs['L']:
            self.robot.topik.get_q(self.robot.c_pos)
            self.pto.compute_FK(self.robot.topik.q)
            zloc = self.pto.compute_FK_z(self.robot.topik.q)
            print("FK Check:", self.pto.X0, zloc[0], self.robot.topik.cnt2m)
    
    def save_data(self):
    # 1. 로봇 상태 및 순운동학(FK) 업데이트
        self.robot.topik.get_q(self.robot.c_pos)
        self.pto.compute_FK(self.robot.topik.q)
        robot_yaw = self.robot.c_pos[3]*self.robot.topik.cnt2m[3]
        
        # 3. 동적 마커 데이터 가져오기 (self.agv.detected_markers)
        # {id: {'x':.., 'y':.., 'z':.., 'qx':.., 'qy':.., 'qz':.., 'qw':.., 'stamp':..}}
        markers = self.robot.agv.detected_markers
        sorted_ids = sorted(markers.keys())  # ID 순으로 정렬하여 일관성 유지
        
        # 데이터셋 구성 (최대 2개의 마커를 나란히 저장)
        marker_data = []
        for i in range(2):  # Marker 1, Marker 2 공간 확보
            if i < len(sorted_ids):
                m_id = sorted_ids[i]
                m_info = markers[m_id]
                marker_data += [m_id, m_info['x'], m_info['y'], m_info['z'], 
                                m_info['qx'], m_info['qy'], m_info['qz'], m_info['qw']]
            else:
                # 감지된 마커가 2개 미만일 경우 빈칸(또는 0.0) 처리
                marker_data += ["N/A", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        # 4. 헤더 및 행 데이터 구성
        # FK 포즈(X0) + 마커1 정보 + 마커2 정보 + 로봇 추가 데이터
        header = [
            "robot_x", "robot_y", "robot_th",
            "m1_id", "m1_x", "m1_y", "m1_z", "m1_qx", "m1_qy", "m1_qz", "m1_qw",
            "m2_id", "m2_x", "m2_y", "m2_z", "m2_qx", "m2_qy", "m2_qz", "m2_qw",
        ]
        
        row_data = self.pto.X0 + [robot_yaw]+ marker_data

        # 5. CSV 파일 저장
        filename = "Calibration_data.csv"
        file_exists = os.path.isfile(filename)

        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            # 파일이 비어있거나 존재하지 않을 때만 헤더 작성
            if not file_exists or os.path.getsize(filename) == 0:
                writer.writerow(header)
            writer.writerow(row_data)

        print(f"[SUCCESS] Calibration data saved. Detected Markers: {sorted_ids}")


    

