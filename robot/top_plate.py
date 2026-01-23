import time
import numpy as np
import spatialmath.base as sb

def topplate_controller(self):
    factor=1 #sky, control moving distance factor
    inching_timer = 0
    inching_flag = False
    task_loading=False
    task_homming=False
    task_approach=False
    homming_state=0
    homming_state_timer=0.0
    homming_state_flag=False
    self.global_T = time.time()

    tp_time_p=time.time()
    tp_time_c=0.0
    tp_time_t=0.0
    thread_check_flag_tp=True
    
    while not self.process_command:
        thread_total=0
        for i in range(7):
            thread_total+=self.thereadcheck[i+1]
        if thread_total==7:
            self.agv.lift_state_ros=self.agv.lift_state_ros|0x0008
            self.agv.thread_check_total=True
        else:
            self.agv.lift_state_ros=self.agv.lift_state_ros&0xfff7
            self.agv.thread_check_total=False
                
        # homming check. if all servos are homming, then set was_home flag 
        if not self.was_home_checked :
            if time.time()-self.global_T>2.0:
                self.was_home_checked=True
                self.was_home=self.servos[0].is_originfinding_ok and self.servos[1].is_originfinding_ok and self.servos[2].is_originfinding_ok and self.servos[3].is_originfinding_ok and self.servos[4].is_originfinding_ok and self.servos[5].is_originfinding_ok and self.servos[6].is_originfinding_ok
                if self.was_home:
                    self.agv.lift_state_ros=self.agv.lift_state_ros|0x0004
                else:
                    self.agv.lift_state_ros=self.agv.lift_state_ros&0xfffb

        # task_clear process
        if self.agv.task_clear_top:
            self.agv.task_clear_top=False
            self.agv.presetnum=0
            inching_flag = False

        #(_sky) , target mode operation
        if ( self.buttons[5] and self.buttons[4]) or self.pto.target_mode : 
            if self.pto.target_mode == False:
                self.repeat_time=0;
                self.pto.target_mode = self.pto.mode[7]

            if self.pto.target_mode == self.pto.mode[7]:   # 재수집
                rot_L=np.array([[ 0,1],[-1,0]]) 
                rot_R=np.array([[ 0.1627,-0.9866],[0.9866,0.1628]]) 
                
                # ROS 노드가 실행 중일 때만 접근
                if hasattr(self, 'cali_node'):
                    Lc, Rc=self.cali_node.latest_cam_L,self.cali_node.latest_cam_R
                    Lc=Lc[:2]
                    Rc=Rc[:2]
                    Lc=np.array(Lc)*100
                    Rc=np.array(Rc)*100
                    self.pto.compute_FK(self.topik.q);
                    X0=np.array(self.pto.X0)
                    X1=np.array(self.pto.X1)
                    L=X0+np.array([[-1,0],[0,1]])@sb.rot2(self.topik.qm[2]+self.topik.qm[3])@(rot_L@Lc+np.array([-5.7, 3.2]))
                    R=[L[0]-144,L[1]]
                
                    print("detected L coord:",L)
                    print("detected R coord:",R)
                    self.Lx,self.Ly=L[0],L[1]
                    self.Rx,self.Ry=R[0],R[1]

                    self.target_data=self.pto.compute_IK(self.Lx, self.Ly, self.Rx, self.Ry)
                    X=self.pto.compute_FK(self.topik.q)
                    pin_error=self.pto.Error_evaluate(X[0],X[1],self.Lx, self.Ly, self.Rx, self.Ry)
                    vision_error=self.pto.Error_evaluate(L,R, 71.866, 11.250, -64.727, 18.837) 
                    value=self.pto.Error_evaluate(X[0],X[1], 71.866, 11.250, -64.727, 18.837)    #마커를 설치한 위치 cm 기입
                    
                    if (np.linalg.norm(np.array(self.abs_error))>np.linalg.norm(np.array(value))) and ((pin_error[0] > 5e-4) or (pin_error[1] > 5e-4)):
                        self.abs_error=value  
                        self.target_mid_state=(np.array(self.target_data)-np.array(self.c_pos))*self.pause_ratio[self.repeat_time]/100+np.array(self.c_pos)
                        self.target_mid_state=np.round(self.target_mid_state,0)
                        self.target_mid_state = self.target_mid_state.astype(int)
                        self.target_mid_state = self.target_mid_state.tolist()
                        
                        self.repeat_time=self.repeat_time+1
                        print("Try -",self.repeat_time,", vision error (vision - marker loc):",vision_error)
                        print("controll error (abs marker pose - controll):",self.abs_error)
                        print(self.target_mid_state)
                        self.pto.target_mode = self.pto.mode[3]
                    else:   
                            print("Task complete!!!!!!!!!")
                            self.pto.target_mode = False;   #완료.
                            was_home = False
            
            # 정렬순서: q[3],q[4] 정렬 → q[2] 정렬 → q[0] 정렬 → q[1] 정렬
            if (self.pto.target_mode == self.pto.mode[3]) or (self.pto.target_mode == self.pto.mode[4]) :   # q[3],q[4] 제어
                error3 = abs(self.target_mid_state[3] - self.c_pos[3])
                if error3 > self.pto.target_tol_error [3]:
                    self.t_action[3] = 0;   # 3th servo amov control
                    self.t_pos[3] = self.target_mid_state[3]   # goal position control
                error4 = abs(self.target_mid_state[4] - self.c_pos[4])
                if error4 > self.pto.target_tol_error [4]:
                    self.t_action[4] = 0;  
                    self.t_pos[4] = self.target_mid_state[4]
                if (error3 <= self.pto.target_tol_error [3]) and (error4 <= self.pto.target_tol_error [4]) :
                    self.pto.target_mode = self.pto.mode[2];

            if self.pto.target_mode == self.pto.mode[2]:   # q[2] 제어
                error = abs(self.target_mid_state[2] - self.c_pos[2])
                if error > self.pto.target_tol_error [2]:
                    self.t_action[2] = 0; 
                    self.t_pos[2] = self.target_mid_state[2]  
                else:
                    self.pto.target_mode = self.pto.mode[0];

            if self.pto.target_mode == self.pto.mode[0]:   # q[0] 제어
                error = abs(self.target_mid_state[0] - self.c_pos[0])
                if error > self.pto.target_tol_error [0]:
                    self.t_action[0] = 0;  
                    self.t_pos[0] = self.target_mid_state[0]  
                else:
                    self.pto.target_mode = self.pto.mode[1];

            if self.pto.target_mode == self.pto.mode[1]:   # q[1] 제어
                error = abs(self.target_mid_state[1] - self.c_pos[1])
                if error > self.pto.target_tol_error [1]:
                    self.t_action[1] = 0;   
                    self.t_pos[1] = self.target_mid_state[1];
                else:
                    self.pto.target_mode = self.pto.mode[7];   
           
        # homming process command    # start with L_Bump + A_Btn. 
        # [호밍 시퀀스 시작 조건] AGV 요청(0x141) 또는 조이스틱 버튼 조합
        if (self.agv.reqid[0][0] == 0x141 and self.agv.op_state[0]!=255) or ((self.axes[4]>0.8 and self.buttons[0]) or task_homming):
            task_homming=True
            
            # [State 0] 초기화 및 Z축 회전 호밍 시작
            if homming_state==0:
                if not homming_state_flag:
                    homming_state_timer=time.time()
                    homming_state_flag=True
                    self.agv.master_homming=True
                self.t_action=[2,2,1,3,3,2,2] # Y,X,핀:On(2) / Z회전:Homing(1) / 날개:Off(3)
                if self.version!=1: # [수정] self.version 사용
                    if time.time()-homming_state_timer>2.0 and self.servos[2].is_originfinding_ok:
                        homming_state=1
                        homming_state_flag=False
                        self.t_pos[2] = 0
                elif time.time()-homming_state_timer>2.0:
                    homming_state=1
                    homming_state_flag=False
                    self.t_pos[2] = 0 # Z회전 0점 설정
            
            # [State 1] 주요 축(Y, X, 핀) 원점 복귀
            elif homming_state==1:
                if not homming_state_flag:
                    homming_state_timer=time.time()
                    homming_state_flag=True
                self.t_action=[1,1,0,3,3,1,1] # Y,X,핀:Homing(1) / Z회전:Move(0)
                if time.time()-homming_state_timer>10.0 and self.servos[0].is_originfinding_ok and self.servos[1].is_originfinding_ok and self.servos[5].is_originfinding_ok and self.servos[6].is_originfinding_ok:
                    homming_state=2
                    homming_state_flag=False
            
            # [State 2] 날개(Wing) 서보 켜기
            elif homming_state==2:
                if not homming_state_flag:
                    homming_state_timer=time.time()
                    homming_state_flag=True
                self.t_action=[0,0,0,2,2,0,0] # 날개:On(2)
                if time.time()-homming_state_timer>2.0:
                    homming_state=3
                    homming_state_flag=False
            
            # [State 3] 날개(Wing) 원점 복귀
            elif homming_state==3:
                if not homming_state_flag:
                    homming_state_timer=time.time()
                    homming_state_flag=True
                self.t_action=[0,0,0,1,1,0,0] # 날개:Homing(1)
                if time.time()-homming_state_timer>15.0 and self.servos[3].is_originfinding_ok and self.servos[4].is_originfinding_ok:
                    homming_state=4
                    homming_state_flag=False
            
            # [State 4] 전체 축 0점 정렬
            elif homming_state==4:
                if not homming_state_flag:
                    homming_state_timer=time.time()
                    homming_state_flag=True
                self.t_action=[0,0,0,0,0,0,0] # 전체:Move(0)
                self.t_pos=[0,0,0,0,0,0,0]
                if time.time()-homming_state_timer>2.0:
                    homming_state=5
                    homming_state_flag=False
            else: 
                self.agv.op_state[0]=255
                homming_state=0
                self.agv.presetnum=0
                self.was_home=True
                task_homming=False
                self.agv.lift_state_ros=self.agv.lift_state_ros|0x0004

            print("homming res: 0_", self.c_pos[0], " / 1_", self.c_pos[1], " / 2_", self.c_pos[2], " / 3_", self.c_pos[3], " / 4_", self.c_pos[4]); 
        # [Loading 1차 상승] 배터리 로딩 준비 위치로 이동
        elif self.was_home and ((self.agv.reqid[0][0] == 0x131 and self.agv.presetnum==4) or ((self.axes[4]>0.8 and self.buttons[1]) or task_approach)): 
            task_approach=True
            self.t_action=[0,0,0,0,0,0,0]
            qtemp=self.topik.set_xd(self.agv.cartype,self.toff_st[self.agv.cartype])
            self.t_pos[0] = qtemp[0]
            self.t_pos[1] = qtemp[1]
            self.t_pos[2] = qtemp[2]
            self.t_pos[3] = qtemp[3]
            self.t_pos[4] = qtemp[4]
            self.t_pos[5] = self.pinh_st_attach # L_Pin
            self.t_pos[6] = self.pinh_st_attach # R_Pin
            self.laser = True
            if inching_flag==False:
                inching_flag = True
                inching_timer = time.time()
            if time.time()-inching_timer>5.0:
                self.agv.op_state[0]=255
                self.agv.presetnum=0
                inching_flag = False
                task_approach=False

        # [Loading 2차 상승] 실제 배터리 체결 동작 (Inching 제어)
        elif self.was_home and ((self.agv.reqid[0][0] == 0x131 and self.agv.presetnum==5) or ((self.axes[4]>0.8 and self.buttons[2]) or task_loading)):
            self.t_action=[0,0,0,0,0,0,0]
            task_loading=True
            if inching_flag == False:
                inching_flag = True
                inching_timer = time.time()

            if inching_flag:
                if time.time()-inching_timer<3.0:
                    self.t_pos[0] = self.servos[0].com_apos # y
                    self.t_pos[1] = self.servos[1].com_apos # x               print(5/3)
                    self.t_action[4] = 3
                    self.t_pos[0] = self.servos[0].com_apos # y
                    self.t_pos[1] = self.servos[1].com_apos # x
                    self.t_pos[2] = self.servos[2].com_apos # z
                    self.t_pos[3] = self.servos[3].com_apos # L 날개
                    self.t_pos[4] = self.servos[4].com_apos # R 날개
                    self.t_pos[5] = self.pinh_st_load # L_Pin
                    self.t_pos[6] = self.pinh_st_load # R_Pin

                if inching_flag==False:
                    inching_flag = True
                    inching_timer = time.time()
                if time.time()-inching_timer>5.0:
                    self.agv.op_state[0]=255
                    inching_flag = False
                    task_approach=False
                    self.agv.presetnum=0
                    task_loading=False
                else:
                    pass
        # ... (Manual control logic omitted for brevity, similar structure)
        
        # update the top plate position and action 
        self.t_pos_fastech=[self.t_pos[0],self.t_pos[1],self.t_pos[2],self.t_pos[3],self.t_pos[4],self.t_pos[5],self.t_pos[6]]
        self.t_action_fastech=[self.t_action[0],self.t_action[1],self.t_action[2],self.t_action[3],self.t_action[4],self.t_action[5],self.t_action[6]]
        self.servo_states.update_topplate(self.c_pos,self.t_pos)
        time.sleep(0.04)
    print("*** Top plate Controller Disconnect ***")