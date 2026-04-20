import time
from tasks import *


class TopPlateSupervisor:
    """
    상부 모듈 제어 총괄 클래스
    버튼 -> 작업(Task) 매핑 및 실행 관리
    버튼별 작업이 보기 편하도록 구조화.
    """
    def __init__(self, robot):
        self.robot = robot
        self.monitor = SystemMonitor(robot)
        self.homing = HomingTask(robot)
        self.loading = LoadingTask(robot)
        self.manual = ManualpinTask(robot)
        self.vision = VisionTask(robot)
        self.pbvs_l = PbvsLTask(robot)
        self.pbvs_r = PbvsRTask(robot)
        self.pbvs_both = PbvsBothTask(robot)

    def run_loop(self):
        print("*** Top plate Supervisor Start ***")
        
        while not self.robot.process_command:
            inputs = self.robot.input_manager.get_state()
            self.monitor.update()
            self.homing.run()
            self.loading.run()
            self.vision.run()
            self.pbvs_l.run()
            self.pbvs_r.run()
            self.pbvs_both.run()

            if self.robot.agv.task_clear_top:
                self.homing.reset(); self.loading.reset()
                self.pbvs_l.reset(); self.pbvs_r.reset(); self.pbvs_both.reset()
            
            # --- Trigger Logic ---
            
            # [Homing]
            if (self.robot.agv.reqid[0][0] == 0x141 and self.robot.agv.op_state[0]!=255) or \
               ((inputs['LT_PRESSED'] and inputs['A'])):
                self.homing.start()


            # [Loading]
            elif self.robot.was_home:
                reqid = self.robot.agv.reqid[0][0]; preset = self.robot.agv.presetnum
                if (reqid == 0x131 and preset == 4) or (inputs['LT_PRESSED'] and inputs['B']):
                    self.loading.start_approach()
                elif (reqid == 0x131 and preset == 5) or (inputs['LT_PRESSED'] and inputs['X']):
                    self.loading.start_load()
            # [PBVS — L pin: LT+Y, R pin: RT+Y, Both: LT+RT]
            if inputs['LT_PRESSED'] and inputs['Y'] and not self.pbvs_r.is_active and not self.pbvs_both.is_active:
                self.pbvs_l.start()
            if inputs['RT_PRESSED'] and inputs['Y'] and not self.pbvs_l.is_active and not self.pbvs_both.is_active:
                self.pbvs_r.start()
            if inputs['LT_PRESSED'] and inputs['RT_PRESSED'] and not inputs['Y'] \
               and not self.pbvs_l.is_active and not self.pbvs_r.is_active:
                self.pbvs_both.start()

            # [Manual Control] - 자동 작업 아닐 때만
            if not self.homing.is_active and not self.loading.is_active \
               and not self.pbvs_l.is_active and not self.pbvs_r.is_active \
               and not self.pbvs_both.is_active:
                
                # 1. Pin Manual (START/SEL)
                if inputs['START'] or inputs['SEL']:
                    # START: Right Pin (4, 6)
                    if inputs['START']:
                        # Rotation (X/B)
                        dir_rot = 0
                        if inputs['X']: dir_rot = 1
                        elif inputs['B']: dir_rot = -1
                        self.manual.pin_control(4, dir_rot, rot=1)
                        
                        # Lift (Y/A)
                        dir_lift = 0
                        if inputs['Y']: dir_lift = 1
                        elif inputs['A']: dir_lift = -1
                        self.manual.pin_control(6, dir_lift, rot=0)

                    # SEL: Left Pin (3, 5)
                    if inputs['SEL']:
                        # Rotation
                        dir_rot = 0
                        if inputs['X']: dir_rot = 1
                        elif inputs['B']: dir_rot = -1
                        self.manual.pin_control(3, dir_rot, rot=1)
                        
                        # Lift
                        dir_lift = 0
                        if inputs['Y']: dir_lift = 1
                        elif inputs['A']: dir_lift = -1
                        self.manual.pin_control(5, dir_lift, rot=0)

                # 2. Lift & Aux Manual (R Button)
                if inputs['R']:
                     # Lift Axis 1 (Forward/Back)
                     self.manual.lift(1, inputs['HAT_Y'], rot=0)
                     
                     # Lift Axis 0 (Left/Right)
                     self.manual.lift(0, inputs['HAT_X'], rot=0)
                     
                     # Rotate Axis 2 (X/B)
                     dir_rot = 0
                     if inputs['X']: dir_rot = 1
                     elif inputs['B']: dir_rot = -1
                     self.manual.lift(2, dir_rot, rot=1)
                     
                     # Wing Gap (Left Stick Y)
                     if self.robot.was_home and self.robot.ikmode:
                         ly = inputs['AXIS_LY']
                         if ly < -0.5: self.manual.wing_gap(1) # Expand
                         elif ly > 0.5: self.manual.wing_gap(-1) # Shorten
                
                # 3. System Control (R_JOY)
                if inputs['R_JOY_BTN']:
                    if inputs['HAT_X'] == -1: self.manual.system_control('reset')
                    elif inputs['HAT_X'] == 1: self.manual.system_control('buzzer_stop')
                    
                    if inputs['HAT_Y'] == -1: self.manual.system_control('servo', 1)
                    elif inputs['HAT_Y'] == 1: self.manual.system_control('servo', 0)

                if inputs['L']:
                    if inputs['R']:
                        self.vision.save_annotation()
                    else:
                        self.vision.save_data()

            self.robot.t_pos_fastech = self.robot.t_pos[:]
            self.robot.t_action_fastech = self.robot.t_action[:]
            
            # (기존 코드) 상태 업데이트 및 sleep
            self.robot.servo_states.update_topplate(self.robot.c_pos, self.robot.t_pos)
            time.sleep(self.robot.config.loop_sleep_main)
        print("*** Top plate Supervisor Disconnect ***")