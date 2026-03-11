import threading
from fastech import protocol as fs
from fastech import servolist as sl
from fastech import servo_state as ss
import time
from leadshine import agv_con as aj
import platform
from controller import ik
if platform.system() == 'Linux':
    is_windows=False
    is_ros_if=True
else:
    is_windows=True
    is_ros_if=False

try: from ros2 import interface as rif
except: is_ros_if=False
if is_windows:
    from controller import joy

'''
"sudo chmod 666 /dev/input/*" in Linux to get permisson of controller
"python3 -m evdev.evtest" in terminal, if you want to check controller number or name
'''
version=2
ip_s=sl.udp_addresses
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

    pinh_bt_carry=180000
    pinh_pit_1st_bt_detach=120000
    pinh_pit_1st_bt_attach=200000
    pinh_pit_2nd=260000
    pinh_st_load=150000
    pinh_st_attach=46000
    pinh_attach_ready=200000
    maxpin_height_small=270000
    maxpin_height_big=733000

    lh_pit2nd=1.72
    lh_pit1st=1.630
    lh_st1st=1.36
    lh_st2nd=1.46
    lh_standby=1.13
    was_home=False
    was_home_checked=False
    global_T=0.0
    thereadcheck=[0,0,0,0,0,0,0,0,0,0,0,0,0]
    toff_st=[[0,0,0],[-0.05,0,0],[0,0,0],[0,0,0],[0.0,0.0,0.0],[0,0,0]] # offset for loading and unloading at storage [0] niro, [1] ioniq5
    toff_pit=[[0,0,0],[-0.0158,-0.006,0],[0,0,0],[0,0,0],[0.0,0.0,0.0],[0,0,0]] # offset for loading and unloading at pit [0] niro, [1] ioniq5
    toff_pos=[[0,0,0],[0.0,0.0,0],[-0.0136,+0.0128,0]] # offset for loading and unloading at position dependent value, [0] storage, [1] pit2, [2] pit1

    def __init__(self, addresses):
        self.servos = [fs.fastech(address) for address in addresses]
        self.servo_states = ss.ServoState()
        self.agv=aj.AGV("",is_real_robot)
        self.agv.is_minorcontrol=self.safety_mode
        self.master_homming=False
        print(f'C&T position: {self.agv.c_pos},{self.agv.t_pos}')
        self.topik=ik.Topik(2)
        self.ikmode=True
        self.pingap=ik.niro_gap
        self.maxpin_height=270000
        self.manual_servo_flag = False

        threads = []
        if self.is_ros_if:
            # ros installed system
            thread= threading.Thread(target=self.agv_planning_main,args=())
            thread.start()
            threads.append(thread)
        else: 
            # non-ros installed system
            pass
        if is_windows:
            # windows labtop 
            thread= threading.Thread(target=self.pygame_gui,args=())
            thread.start()
            threads.append(thread)
            pass
        else: 
            #linux os
            thread= threading.Thread(target=self.joystick, args=())
            thread.start()
            threads.append(thread)

        if is_real_robot: 
            # real robot system
            for servo in self.servos:
                thread = threading.Thread(target=self.control_servo, args=(servo,))
                thread.start()
                threads.append(thread)
        else: 
            # simulation system
            pass        
        
        # always running threads
        thread = threading.Thread(target=self.control_agv, args=())
        thread.start()
        threads.append(thread)
        
        thread= threading.Thread(target=self.topplate_controller, args=())
        thread.start()
        threads.append(thread)
        
        # Wait for all threads to finish
        for thread in threads:
            thread.join()
    
    # ROS2 Interface / Get requests and callback responds
    def agv_planning_main(self):
        rif.rclpy.init(args=None)
        self.ros = rif.SrvreqSubscriber(self.agv,self.servo_states)
        self.thereadcheck[0]=1
        rif.rclpy.spin(self.ros)

    def control_servo(self, servo):
        # Connect to the client
        servo._state_init()
        servo._state_get(self.t_pos_fastech[servo.id],self.t_vel[servo.id],self.estop_total,self.can_movetop,self.t_action_fastech[servo.id])
        self.t_pos_fastech[servo.id] = servo.pos
        #servo.servo_on() #### for debugging
        while not self.process_command:
            # if servo.id==3:
            #     print(f'servo commstate for id 3 is {servo.thread_check_commflag}')
            if self.is_real_robot:
                servo._state_get(self.t_pos_fastech[servo.id],self.t_vel[servo.id],self.estop_total,self.can_movetop,self.t_action_fastech[servo.id])
                servo._state_check()
                servo._state_set()
                self.c_pos[servo.id] = servo.pos
                self.thereadcheck[servo.id+1]=servo.thread_check_fs
            else : 
                if self.t_action_fastech[servo.id]==2:
                    servo.state=1
                elif self.t_action_fastech[servo.id]==3:
                    servo.state=0
                elif self.t_action_fastech[servo.id]==0:
                    servo.pos=self.t_pos[servo.id]
                elif self.t_action_fastech[servo.id]==1:
                    servo.is_originfinding_ok=True
                else:
                    pass
            time.sleep(0.05)
        servo.servo_off()
        
        # Close the connection

    def joystick(self):
        multi_command_flag = True

        # Try to connect to the controller once
        if not is_windows:
            from controller import evdev_controlthread as evct
            # initialize the top plate controller
            controller = evct.controller()
            controller.version=2
            controller.controller_connect()

        # While the process is not finished, keep getting the data from the controller
        while not self.process_command:

            if not is_windows:
                #in Windows, the controller is connected in the pygame_gui function
                #in Linux, the controller is connected here
                # while there are commands in joystick, keep getting the data
                # there is chance to ignore short command. 
                while multi_command_flag:
                    multi_command_flag = controller.joystick_get_data()
                self.axes=controller.axes
                self.buttons=controller.buttons
                self.hat=controller.hat
                multi_command_flag = True
            time.sleep(0.05)

    def topplate_controller(self):
        
        inching_timer = 0
        pin_timer = 0.0
        inching_flag = False
        taction_flag = False
        task_loading=False
        homming_state=0
        homming_state_timer=0.0
        homming_state_flag=False
        self.global_T = time.time()

        tp_time_p=time.time()
        tp_time_c=0.0
        tp_time_t=0.0
        thread_check_flag_tp=True
        thread_check_flag_gt=True

        
        while not self.process_command:
            
            if thread_check_flag_gt:
                if time.time()-self.global_T>20.0:
                    thread_check_flag_gt=False
                thread_total=0
                for i in range(7):
                    thread_total+=self.thereadcheck[i+1]
                if thread_total==7:
                    self.agv.thread_check_total=True
                else:
                    self.agv.thread_check_total=False
            else:
                thread_total=0
                for i in range(7):
                    thread_total+=self.thereadcheck[i+1]
                if thread_total==7:
                    self.agv.lift_state_ros=self.agv.lift_state_ros|0x0008
                else:
                    self.agv.lift_state_ros=self.agv.lift_state_ros&0xfff7
                    
            
            if not self.was_home_checked :
                if time.time()-self.global_T>2.0:
                    self.was_home_checked=True
                    self.was_home=self.servos[0].is_originfinding_ok and self.servos[1].is_originfinding_ok and self.servos[2].is_originfinding_ok and self.servos[3].is_originfinding_ok and self.servos[4].is_originfinding_ok and self.servos[5].is_originfinding_ok and self.servos[6].is_originfinding_ok
                    if self.was_home:
                        self.agv.lift_state_ros=self.agv.lift_state_ros|0x0004
                    else:
                        self.agv.lift_state_ros=self.agv.lift_state_ros&0xfffb
            
            # pin height setting
            if self.agv.cartype==3:
                self.maxpin_height=self.maxpin_height_big
            else:
                self.maxpin_height=self.maxpin_height_small

            # task_clear process
            if self.agv.task_clear_top:
                self.agv.task_clear_top=False
                self.agv.presetnum=0
                inching_flag = False
                self.agv.lift_manual_positionmode=False

            if self.agv.pin_state==1:
                if time.time()-pin_timer>5.0:
                    self.agv.pin_state=0
               
            if self.agv.reqid[0][0] == 0x141 and self.agv.op_state[0]!=255:
                if homming_state==0:
                    if not homming_state_flag:
                        homming_state_timer=time.time()
                        homming_state_flag=True
                        self.agv.master_homming=True
                    # Y,X, LRpin servo on and Z state is set zero once
                    self.t_action=[2,2,1,3,3,2,2]
                    if version!=1: 
                        if time.time()-homming_state_timer>2.0 and self.servos[2].is_originfinding_ok:
                            homming_state=1
                            homming_state_flag=False
                            self.t_pos[2] = 0
                    elif time.time()-homming_state_timer>2.0:
                        homming_state=1
                        homming_state_flag=False
                        self.t_pos[2] = 0
                elif homming_state==1:
                    if not homming_state_flag:
                        homming_state_timer=time.time()
                        homming_state_flag=True
                    # Y,X, LRpin find home and z servo on
                    self.t_action=[1,1,0,3,3,1,1]
                    if time.time()-homming_state_timer>10.0 and self.servos[0].is_originfinding_ok and self.servos[1].is_originfinding_ok and self.servos[5].is_originfinding_ok and self.servos[6].is_originfinding_ok and self.agv.lift_was_home_fin:
                        homming_state=2
                        homming_state_flag=False
                elif homming_state==2:
                    if not homming_state_flag:
                        homming_state_timer=time.time()
                        homming_state_flag=True
                    # LR servo on
                    self.t_action=[0,0,0,2,2,0,0]
                    if time.time()-homming_state_timer>2.0:
                        homming_state=3
                        homming_state_flag=False
                elif homming_state==3:
                    if not homming_state_flag:
                        homming_state_timer=time.time()
                        homming_state_flag=True
                    # LR find home
                    self.t_action=[0,0,0,1,1,0,0]
                    if time.time()-homming_state_timer>15.0 and self.servos[3].is_originfinding_ok and self.servos[4].is_originfinding_ok:
                        homming_state=4
                        homming_state_flag=False
                elif homming_state==4:
                    if not homming_state_flag:
                        homming_state_timer=time.time()
                        homming_state_flag=True
                    # all zero
                    self.t_action=[0,0,0,0,0,0,0]
                    self.t_pos=[0,0,0,0,0,0,0]
                    if time.time()-homming_state_timer>2.0:
                        homming_state=5
                        homming_state_flag=False
                else: 
                    self.agv.op_state[0]=255
                    homming_state=0
                    self.agv.presetnum=0
                    self.was_home=True
                    self.agv.lift_state_ros=self.agv.lift_state_ros|0x0004

            # St down (Lift 하단 하강, Loading 후 하강)
            # After 5 sec, send finish signal to server automatically. And change pin height properly.
            elif self.agv.reqid[0][0] == 0x131 and self.agv.presetnum==3: 
                self.t_action=[0,0,0,0,0,0,0]
                self.t_pos[0] = self.servos[0].com_apos # y
                self.t_pos[1] = self.servos[1].com_apos # x
                self.t_pos[2] = self.servos[2].com_apos # z
                self.t_pos[3] = self.servos[3].com_apos # L 날개
                self.t_pos[4] = self.servos[4].com_apos # R 날개
                if self.agv.cartype==3: # 포터/봉고 시
                    self.t_pos[5] = 0
                    self.t_pos[6] = 0
                else:
                    if self.agv.is_loaded:
                        self.t_pos[5] = 0 # L_Pin
                        self.t_pos[6] = 0 # R_Pin
                    else:
                        self.t_pos[5] = self.pinh_st_load # L_Pin
                        self.t_pos[6] = self.pinh_st_load # R_Pin
                self.agv.lift_manual_positionmode=True
                self.agv.lift_target=self.lh_standby
                self.laser = False
                # self.t_action[3] = 2
                # self.t_action[4] = 2
                if inching_flag==False:
                    inching_flag = True
                    taction_flag = True
                    inching_timer = time.time()
                if time.time()-inching_timer>4.0 and taction_flag:
                    for i in range(len(self.t_action)):
                        self.t_action[i] = 2
                    taction_flag = False
                if time.time()-inching_timer>5.0 and self.agv.lift_arrived:
                    for i in range(len(self.t_action)):
                        self.t_action[i] = 0
                    self.agv.op_state[0]=255
                    self.agv.presetnum=0
                    inching_flag = False
                    if self.agv.cartype==3:
                        self.t_pos[5] = 0
                        self.t_pos[6] = 0
                    else:
                        self.t_pos[5] = self.pinh_attach_ready # L_Pin
                        self.t_pos[6] = self.pinh_attach_ready # R_Pin
                    self.agv.lift_manual_positionmode=False
                    if self.agv.is_loaded:
                        self.agv.is_loaded = False
                        self.t_pos[5] = 0 # L_Pin
                        self.t_pos[6] = 0 # R_Pin
                    elif not self.agv.is_loaded:
                        self.agv.is_loaded = True

            # Loading 1차 상승
            # After 5 sec, send finish signal to server automatically.
            elif self.agv.reqid[0][0] == 0x131 and self.agv.presetnum==4: 
                self.t_action=[0,0,0,0,0,0,0]
                qtemp=self.topik.set_xd(self.agv.cartype,self.toff_st[self.agv.cartype])
                self.t_pos[0] = qtemp[0]#-int(self.home_y_off*2000000) # y
                self.t_pos[1] = qtemp[1]
                self.t_pos[2] = qtemp[2]
                self.t_pos[3] = qtemp[3]
                self.t_pos[4] = qtemp[4]
                self.t_pos[5] = self.pinh_st_attach # L_Pin
                self.t_pos[6] = self.pinh_st_attach # R_Pin
                self.agv.lift_manual_positionmode=True
                self.agv.lift_target=self.lh_st1st
                self.laser = True
                if inching_flag==False:
                    inching_flag = True
                    inching_timer = time.time()
                if time.time()-inching_timer>5.0 and self.agv.lift_arrived:
                    self.agv.op_state[0]=255
                    self.agv.presetnum=0
                    inching_flag = False
                    self.agv.lift_manual_positionmode=False

            # Loading 2차 상승
            # start with L_Tri + X_Btn. And It only works c_pos 0.
            # First 3 sec, Loading Battery on AGV.
            # next 2 sec, change pin height properly without wing motor torque
            # if self.agv.reqid[0][0] == 0x131 and self.agv.presetnum==5: 
            elif ((self.axes[4]>0.8 and self.buttons[2]) or task_loading) and self.agv.c_pos==0:
                self.t_action=[0,0,0,0,0,0,0]
                task_loading=True
                if inching_flag == False:
                    inching_flag = True
                    inching_timer = time.time()
                    # print(f'inching start position : { self.t_pos[0]}: {self.c_pos[0]}, {self.servos[0].com_apos}, {self.t_pos[1]}: {self.c_pos[1]}, {self.servos[1].com_apos}, {self.t_pos[2]}: {self.c_pos[2]}, {self.servos[2].com_apos}')

                if inching_flag:
                    if time.time()-inching_timer<3.0:
                        
                        self.t_pos[0] = self.servos[0].com_apos # y
                        self.t_pos[1] = self.servos[1].com_apos # x
                        self.t_pos[2] = self.servos[2].com_apos # z
                        self.t_pos[3] = self.servos[3].com_apos # L 날개
                        self.t_pos[4] = self.servos[4].com_apos # R 날개
                        self.t_pos[5] = self.servos[5].com_apos # L_Pin
                        self.t_pos[6] = self.servos[6].com_apos # R_Pin
                        
                        self.agv.lift_manual_positionmode=True
                        self.agv.lift_target=self.lh_st2nd
                    
                    elif time.time()-inching_timer<5.0:
                        self.t_action[3] = 3
                        self.t_action[4] = 3

                        self.t_pos[0] = self.servos[0].com_apos # y
                        self.t_pos[1] = self.servos[1].com_apos # x
                        self.t_pos[2] = self.servos[2].com_apos # z
                        self.t_pos[3] = self.servos[3].com_apos # L 날개
                        self.t_pos[4] = self.servos[4].com_apos # R 날개
                        self.t_pos[5] = self.pinh_st_load # L_Pin
                        self.t_pos[6] = self.pinh_st_load # R_Pin
                        self.agv.lift_manual_positionmode=True
                        self.agv.lift_target=self.lh_st2nd

                    elif self.agv.lift_arrived:
                        # self.agv.op_state[0]=255
                        self.t_action[3] = 0
                        self.t_action[4] = 0
                        self.agv.pin_state=1
                        pin_timer = time.time()
                        self.agv.presetnum=0
                        # print(f'inching end position : { self.t_pos[0]}: {self.c_pos[0]}, {self.servos[0].com_apos}, {self.t_pos[1]}: {self.c_pos[1]}, {self.servos[1].com_apos}, {self.t_pos[2]}: {self.c_pos[2]}, {self.servos[2].com_apos}')
                        inching_flag=False
                        task_loading=False
                        self.agv.lift_manual_positionmode=False
                    else:
                        pass

            # pin vservo control (L_Tri + Y_Btn)
            # ik calcaultion for pin position by camera position
            elif (self.axes[4]>0.8 and self.buttons[3]) : # Fixed Movement by location
                self.t_action=[0,0,0,0,0,0,0]
                # get error correction vector and set target position 
                # error2xd have many option to solve ik for the stabel control, further information is in the topik.py
                # In basic, pin gap distance is known, so there is adjusted indirect method. camera based position is pin hole position of camera own coordinate.
                qtemp=self.topik.error2xd(self.agv.cartype,self.c_pos,self.agv.lcam_hole_pos,self.agv.rcam_hole_pos,3)
                print(f"campos L/R:{self.agv.lcam_hole_pos} , {self.agv.rcam_hole_pos},  current q : {self.c_pos}  ")
                print(f"rcam_cor: {self.topik.rcam_err}  lcam_cor: {self.topik.lcam_err} target q : {qtemp}")
                self.t_pos[0] = qtemp[0]
                self.t_pos[1] = qtemp[1]
                self.t_pos[2] = qtemp[2]
                self.t_pos[3] = qtemp[3]
                self.t_pos[4] = qtemp[4]

            
            # pin manual control
            elif (self.buttons[7] or self.buttons[6]) and self.agv.reqid[0][0] != 0x131:
                if self.buttons[7]: # R_Pin Command
                    if not self.safety_mode:
                        self.t_action[4] = 0
                        if self.buttons[2]: # R_Pin rotate left (= + X)
                            self.t_pos[4] = self.c_pos[4] + 100
                        
                        elif self.buttons[1]: # R_Pin rotate right (= + B)
                            self.t_pos[4] = self.c_pos[4] - 100
                            
                        else:
                            self.t_pos[4] = self.c_pos[4]

                    if self.buttons[3]: # R_Pin up (= + Y)
                        self.t_action[6] = 0    
                        temp_height = self.servos[6].com_apos + 10000
                        if temp_height>self.maxpin_height:
                            self.t_pos[6]=self.maxpin_height
                        else:
                            self.t_pos[6] = temp_height
                    
                    elif self.buttons[0]: # R_Pin down (= + A)
                        self.t_action[6] = 0
                        self.t_pos[6] = self.servos[6].com_apos - 10000
                        
                    else:
                        self.t_action[6] = 0
                        self.t_pos[6] = self.servos[6].com_apos

                if self.buttons[6]: # L_Pin Command
                    if not self.safety_mode:
                        self.t_action[3] = 0
                        if self.buttons[2]: # L_Pin rotate left (ㅁ + X)
                            self.t_pos[3] = self.c_pos[3] + 100
                            
                        elif self.buttons[1]: # L_Pin rotate right (ㅁ + B)
                            self.t_pos[3] = self.c_pos[3] - 100
                            
                        else:
                            self.t_pos[3] = self.c_pos[3]  

                    if self.buttons[3]: # L_Pin up (ㅁ + Y)
                        self.t_action[5] = 0
                        temp_height = self.servos[5].com_apos + 10000
                        if temp_height>self.maxpin_height:
                            self.t_pos[5]=self.maxpin_height
                        else:
                            self.t_pos[5] = temp_height

                    elif self.buttons[0]: # L_Pin down (ㅁ + A)
                        self.t_action[5] = 0
                        self.t_pos[5] = self.servos[5].com_apos - 10000 
                    else:
                        self.t_action[5] = 0
                        self.t_pos[5] = self.c_pos[5]

            # Manual Control of topplate and lift
            if self.buttons[5]: # Lift Command
                if self.hat[1] == 1: # Lift forward (R + Hat_Y_Up)
                    self.t_pos[1] = self.c_pos[1] + 10000
                    self.t_action[1] = 0

                elif self.hat[1] == -1: # Lift backward (R + Hat_Y_Down)
                    self.t_pos[1] = self.c_pos[1] - 10000
                    self.t_action[1] = 0

                else:
                    self.t_pos[1] = self.c_pos[1]
                    self.t_action[1] = 0
                
                if self.hat[0] == 1: # Lift left (R + Hat_X_Left)
                    self.t_pos[0] = self.c_pos[0] + 10000
                    self.t_action[0] = 0

                elif self.hat[0] == -1: # Lift right (R + Hat_X_Right)
                    self.t_pos[0] = self.c_pos[0] - 10000
                    self.t_action[0] = 0         
                else:
                    self.t_pos[0] = self.c_pos[0]
                
                if self.buttons[2]: # Lift rotate left (R + X)
                    self.t_pos[2] = self.c_pos[2] + 500
                    self.t_action[2] = 0
                    
                elif self.buttons[1]: # Lift rotate right (R + B)
                    self.t_pos[2] = self.c_pos[2] - 500
                    self.t_action[2] = 0
                else:
                    self.t_pos[2] = self.c_pos[2]

                if self.axes[1]>0.5 and self.ikmode: # pin gap extend
                    self.pingap=self.pingap+0.001
                    if self.pingap>self.topik.dismax:
                        self.pingap=self.topik.dismax
                        
                    qd=self.topik.get_wing_q(self.pingap)
                    self.t_action[3]=0
                    self.t_action[4]=0
                    self.t_pos[3] = int(qd[3])
                    self.t_pos[4] = int(qd[4])
                    # print("wing gap expand",qd[3],qd[4])
                elif self.axes[1] < -0.5 and self.ikmode: # pin gap shorten
                    self.pingap=self.pingap-0.001
                    if self.pingap<self.topik.dismin:
                        self.pingap=self.topik.dismin
                    
                    qd=self.topik.get_wing_q(self.pingap)
                    self.t_pos[3] = int(qd[3])
                    self.t_pos[4] = int(qd[4])
                    self.t_action[3]=0
                    self.t_action[4]=0
                    # print("wing gap expand",qd[3],qd[4])

            # Servo On/Off Command
            if self.agv.reqid[0][0]==9:
                if self.agv.ros_servo_command:
                    self.drv_srv = True
                    for i in range(len(self.t_action)):
                        self.t_action[i] = 2
                        self.t_pos[i] = self.servos[i].pos
                    if self.servos[0].servo_state+self.servos[1].servo_state+self.servos[2].servo_state+self.servos[3].servo_state+self.servos[4].servo_state+self.servos[5].servo_state+self.servos[6].servo_state==7:
                        self.is_fastech_on=True
                    if self.is_fastech_on:
                        for i in range(len(self.t_action)):
                            self.t_action[i] = 0
                        self.agv.op_state[0]=255
                        self.reset=False # 서보온에서 leadshine 에러 발현시 agv 루프에서 자동 리셋 시도. 다시 사용하기 위해 false 복귀
                else :
                    for i in range(len(self.t_action)):
                        self.t_action[i] = 3
                        self.drv_srv = False
                        self.t_pos[i] = self.servos[i].pos
                    if self.servos[0].servo_state+self.servos[1].servo_state+self.servos[2].servo_state+self.servos[3].servo_state+self.servos[4].servo_state+self.servos[5].servo_state+self.servos[6].servo_state==0:
                        self.is_fastech_on=False
                    if  not self.is_fastech_on:
                        for i in range(len(self.t_action)):
                            self.t_action[i] = 0
                        self.agv.op_state[0]=255

            elif self.manual_servo_flag:
                for i in range(len(self.t_action)):
                    self.t_action[i] = 0
                self.manual_servo_flag = False

            elif self.buttons[10] and self.hat[1] == -1: # Servo On (R_Joy_Btn + Hat_X_Up)
                for i in range(len(self.t_action)):
                    self.t_action[i] = 2
                    self.drv_srv = True
                    self.t_pos[i] = self.servos[i].pos
                self.manual_servo_flag = True
                self.is_fastech_on = True
                
            elif (self.buttons[10] and self.hat[1] == 1): # Servo Off (R_Joy_Btn + Hat_X_Down)
                for i in range(len(self.t_action)):
                    self.t_action[i] = 3
                    self.drv_srv = False
                self.manual_servo_flag = True
                self.is_fastech_on = False
                
            else:
                pass

            # emergency Soft stop (only A + B)
            if self.buttons[0] and self.buttons[1] and not self.buttons[5]:
                self.agv.lt_soft_stop=True
            else:
                self.agv.lt_soft_stop=False

            # manual laser control
            if self.buttons[6] and self.buttons[7]: # Laser Command
                if self.buttons[4]: # Laser On (ㅁ + = + L)
                    self.laser = True
                elif self.buttons[5]: # Laser Off (ㅁ + = + R)
                    self.laser = False
            if self.buttons[10]: # 해제 Command 
                if self.hat[0] == -1: # 이상 해제 (R_Joy_Btn + Hat_X_Left)
                    print("Reset Command was called")
                    self.reset = True
                    self.estop_joy = False
                    self.agv.estop_server2 = False

                elif self.hat[0] == 1: # 부저 해제 (R_Joy_Btn + Hat_X_Right)
                    self.buzzer_stop = True
            # buzzer and reset flag reset for joystick 기본적으로 false, 한번이라도 true가 되면 plc단에서 신호를 홀딩하고 있음.
            elif self.buzzer_stop==True or self.reset==True:
                if self.buzzer_stop == True:
                    self.buzzer_stop = False
                if self.reset == True:
                    self.reset = False

            # check the thread time once in 0.5 sec
            if thread_check_flag_tp:
                tp_time_c=time.time()
                tp_time_t=tp_time_c-tp_time_p
                tp_time_p=tp_time_c
                if tp_time_t>0.05 and tp_time_t<0.5:
                    self.thereadcheck[12]=self.thereadcheck[12]|1
                if tp_time_c-self.global_T>0.5:
                    thread_check_flag_tp=False
            
            # update the top plate position and action 
            self.t_pos_fastech=[self.t_pos[0],self.t_pos[1],self.t_pos[2],self.t_pos[3],self.t_pos[4],self.t_pos[5],self.t_pos[6]]
            self.t_action_fastech=[self.t_action[0],self.t_action[1],self.t_action[2],self.t_action[3],self.t_action[4],self.t_action[5],self.t_action[6]]
            self.servo_states.update_topplate(self.c_pos,self.t_pos)
            time.sleep(0.05)
            # [self.axes,self.buttons,self.hat]=-> t_pos, t_vel, t_action, t_state
        # close the controller
        print("*** Top plate Controller Disconnect ***")
    
    def pygame_gui(self):
        # initialize joystick if it is connected
        viz=joy.joy_gui()
        
        viz.is_joy()
        
        while True:
            ### cprofile code
            # if is_windows:
            #     pr = cProfile.Profile() 
            #     pr.enable()
            axes,buttons,hat=viz.get_joy()
            self.axes=axes
            self.buttons=buttons
            self.hat=hat
            viz.screen_set([axes[0],axes[1]])
            viz.check_event()
            if self.process_command:
                viz.quit_event()        
                break
            time.sleep(0.15)

            ### cprofile code
            # if is_windows:
            #     pr.disable()
            #     ss = io.StringIO()
            #     sortby = SortKey.CUMULATIVE
            #     ps = pstats.Stats(pr, stream=ss).sort_stats(sortby)
            #     ps.print_stats()
            #     print(ss.getvalue())
            
        joy.pygame.quit()   
        
        print("**************************")
        print("pygame quit")
        print("**************************")
                   
    def control_agv(self):
        # Main game loop
        running = True
        # self.agv.is_auto=True
        temp_time = time.time()
        #self.agv.get_state()
        self.agv.initial_drive_value_get()
        if is_real_robot: 
            self.thereadcheck[11]=1

        while running:
            # update input data
            #_state_get update data from AGV drive, joystick, and magnet sensor
            # and also from ros topic
            if  self.is_fastech_on:
                self.is_all_servo_on=True
            else:
                self.is_all_servo_on=False

            self.agv._state_get([self.axes,self.buttons,self.hat],self.is_all_servo_on,self.mgresults,self.mg_pos,self.mg_pos_minor)
            self.agv.process_command=self.process_command
            self.agv.was_home_top=self.was_home
           
            
            self.agv.t_action_monitoring=self.t_action_fastech
            self.agv.thread_check_all=self.thereadcheck
            

            ##
            if (self.buttons[9] and self.buttons[10]) or self.agv.will_terminate: # Robot Off (L_Joy_Btn + R_Joy_Btn)
                running = False
                self.process_command=1
                break
            
            self.agv.is_minorcontotal= self.agv.is_minorcontrol
            if self.agv.is_minorcontotal:
                self.agv.driving_state=self.agv.driving_state&0xfffd
            else:
                self.agv.driving_state=self.agv.driving_state|0x0002
            
            # decision making and calculate the desired motion(speed)
            self.agv._state_check()
            ############################################################
            ################### ESTOP ##################################
            if self.agv.estop_fin_command_2server:
                self.reset = True
                self.estop_joy = False
                self.agv.estop_fin_command_2server = False
            self.estop_total= self.estop_joy or self.estop_ls2 or self.agv.estop_server2 !=0
            self.estop_2ls= self.estop_joy or self.agv.estop_server2
            self.agv.estop=self.estop_total
            ##############################################################
            
            self.safety_mode=self.agv.is_minorcontotal

            ############################################### 
            ######### timeout check ######################
            temp_time2 = time.time()
            dt_time=temp_time2-temp_time
            self.agv_comm_time=dt_time
            if (dt_time>0.17 and not self.agv.lift_manual_positionmode) or (dt_time>0.35 and self.agv.lift_manual_positionmode):
                print(f"ALARM control time is longer thant expected : {dt_time}")
            temp_time = temp_time2
            # in case of virtual loop, loop speed is 0.16sec 
            if dt_time<0.01:
                time.sleep(0.16-dt_time)
            ###############################################
            
        while (self.agv.lw_op!=19 or self.agv.rw_op!=19) and (self.agv.lw_op!=1 or self.agv.rw_op!=1):
            self.agv.send2driver([0,0,0])
            self.agv.get_state()
            if not is_real_robot:
                break
        if self.is_ros_if:
            self.ros.destroy_node()
            rif.rclpy.shutdown()
        print(f"wheel speed at termination {self.agv.lw_v}{self.agv.rw_v}")

        
if __name__ == "__main__":
    addresses = ip_s
    controller = robot(addresses)
    print("Program was down successfully")
    print("****************************************")
    print("entire process was down successfully")
    print("****************************************")