import sys
import os
# 상위 디렉토리 모듈 import를 위해 경로 추가
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import threading
import time
import platform
from fastech import protocol as fs
from fastech import servolist as sl
from fastech import servo_state as ss
from leadshine import agv_dummy as aj
from controller import ik
from sky import pos_target_oper

# 분리된 모듈 import
from .top_plate import topplate_controller
from .core import control_servo, control_agv
from .input import joystick, pygame_gui
from .ros_if import agv_planning_main

if platform.system() == 'Linux':
    is_windows=False
    is_ros_if=True
else:
    is_windows=True
    is_ros_if=False

try: from ros2 import interface as rif
except: is_ros_if=False

version=4
ip_s=sl.udp_addresses
reacquire_rate = 100 #0~100% (_sky)
is_real_robot=True

class robot:
    
    process_command=0
    mgresults=[[0,0,0,0,False],[0,0,0,0,False]]
    axes=[0,0,0,0,0,0] # [laxes,raxes,triggers]
    buttons=[0,0,0,0,0,0,0,0,0,0,0] # [A,B,X,Y,L,R,SEL,START,LOGO,L_JOY(s1),R_JOY(s2)]
    hat=[0,0] # [x,y]
    
    is_ros_if=is_ros_if
    robot_id=0
    c_pos=[0,0,0,0,0,0,0] # current position
    t_pos=[0,0,0,0,0,0,0] # target position
    t_pos_fastech=[0,0,0,0,0,0,0] # target position
    t_vel=[10000,10000,2000,600,600,50000,50000] # target velocity
    t_action=[0,0,0,0,0,0,0] # 2: servo on, 3: servo off, 1: servo homing, 0: servo amov, 4: servo stop, 5: servo reset
    t_action_fastech=[0,0,0,0,0,0,0]
    t_state=[0,0,0,0,0,0,0]
    estop_2ls=False
    estop_ls2=False
    can_movetop=True
    blue_lamp = False
    drv_srv=False
    laser=False
    alarm=False
    buzzer_stop=False
    reset=False
    job_fin_reset=False
    obs_f=False
    obs_r=False
    home_y_off=0
    estop_total=False
    estop_joy=False
    estop_server=False
    mg_pos=[0,0]
    mg_pos_minor=0
    safety_mode=True
    mglog=True
    agv_comm_time=0.0
    is_fastech_on=False
    is_leadshine_on=False
    is_all_servo_on=False

    # ... (상수 정의 생략, 필요시 추가) ...
    was_home=False
    was_home_checked=False
    global_T=0.0
    thereadcheck=[0,0,0,0,0,0,0,0,0,0,0,0,0]
    toff_st=[[0,0,0],[-0.05,0,0],[0,0,0],[0,0,0],[0.0,0.0,0.0],[0,0,0]]
    toff_pit=[[0,0,0],[-0.0158,-0.006,0],[0,0,0],[0,0,0],[0.0,0.0,0.0],[0,0,0]]
    toff_pos=[[0,0,0],[0.0,0.0,0],[-0.0136,+0.0128,0]]

    # 메서드 할당
    topplate_controller = topplate_controller
    control_servo = control_servo
    control_agv = control_agv
    joystick = joystick
    pygame_gui = pygame_gui
    agv_planning_main = agv_planning_main

    def __init__(self, addresses):
        self.version = version # 클래스 변수 사용을 위해
        self.servos = [fs.fastech(address) for address in addresses]
        self.servo_states = ss.ServoState()
        self.agv=aj.AGV("",is_real_robot)
        self.master_homming=False
        self.topik=ik.Topik(version)
        self.ikmode=True
        self.pingap=ik.niro_gap
        self.maxpin_height=270000
        self.manual_servo_flag = False
        self.is_real_robot=is_real_robot
        self.is_windows = is_windows # [추가] 분리된 모듈에서 접근 가능하도록 self에 저장
        
        self.pto=pos_target_oper.pos_target_oper(version,reacquire_rate)
        self.Lx=[]
        self.Ly=[]
        self.Rx=[]
        self.Ry=[]
        self.target_mid_state=[]*7
        self.repeat_time=0;
        self.abs_error=[1e10]*2
        self.pause_ratio=[100,100,100,100]+[100]*10

        threads = []
        if self.is_ros_if:
            thread= threading.Thread(target=self.agv_planning_main,args=())
            thread.start()
            threads.append(thread)
        
        # ... (스레드 시작 로직 동일) ...
        
if __name__ == "__main__":
    addresses = ip_s
    controller = robot(addresses)
    print("Program was down successfully")