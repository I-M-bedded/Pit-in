from pymodbus.client import ModbusTcpClient as mbus
import time

class AGV:
    def __init__(self, server_address, is_real_robot=False):
        #self.client = mbus(host=server_address[0],port=server_address[1])
        if is_real_robot:
            while not self.client.connected:
                try: self.client.connect()
                except: pass
            self.mbus_is_connected=self.client.connected
        else: self.mbus_is_connected=False
        
        self.version = 1
        self.initialize_settings()
        self.initialize_data()
        self.initialize_msg()
        self.initialize_task_attr()
        self.initialize_lt()
    
    def initialize_settings(self):
        self.control_amp=1.0
        self.wheel_offset=0.525
        self.w_rpm2rads=3.14159*2.0/60.0
        self.w_rads2ms=0.25/2.0
        self.w_qrpm2rpm=1/60.0
        self.w_qrpm2ms=0.25/3600.0*3.1415926 # self.w_qrpm2rpm*self.w_rpm2rads*self.w_rads2ms
        self.wheel_effect=self.w_qrpm2ms/self.control_amp
        self.decay=1.0
        self.pp_gain=1.5
        self.ppd_gain=0.7
        self.ppi_gain=0.0
        self.ppi_max=0.01

        self.lift_max=1.9
        if self.version == 0:
            self.lift_origin=1.115
        else:
            self.lift_origin=1.114
        self.lift_min=0
        self.lift_v_max=500
        self.lift_v_slow=100
        self.lift_m2cnt=3711300.0
        self.lift_cnt2m=1/self.lift_m2cnt
        self.lift_v_max_m=self.lift_v_max*self.lift_cnt2m
        self.lift_tolerance=0.01
        self.control_hertz=10.0
        self.wv_max=3800
        self.mag_x_off=0.4
        self.mag__foff=0.0
        self.mag__roff=0.0
        self.acc=100
        self.dec=100
        self.acc_slow=500
        self.dec_slow=500
        self.acc_lift=300
        self.dec_lift=300
        self.acc_lift_p=300
        self.dec_lift_p=300
        self.vel_posmode=1300
        
    def initialize_lt(self):
        self.lt_time_gap=3.0
        self.lt_acc_stime=0.0
        self.lt_svel=0.0
        self.lt_stopvel=0.0
        self.lt_tvel=0.0
        self.lt_init_flag_f=False
        self.lt_init_flag_stop=False
        self.lt_flag_reset=False
        self.lt_flag_reset_start=None
        self.lt_init_flag_srv=False
        self.lt_svel_srv=False
        self.lt_tvel_srv=False
        self.lt_acc_stime_srv=0.0
        self.lt_dec_stime=0.0
        self.lt_soft_stop=False
        self.lt_soft_stop_lidar=False
        self.lt_soft_stop_server=False
        self.lt_soft_stop_line=False
        self.mag_count_offset = 0
        self.cpos_change_time= 0.0
        self.cpos_time_buffer= 3.0

    def initialize_data(self):
        self.estop=False
        self.estop_server2=False
        self.estop_fin_command_2server=False
        self.error=0
        self.is_auto = False
        self.is_assigned=False
        self.is_loaded=False
        self.is_semiauto=True
        self.is_minorcontrol=True
        self.is_minorcontotal=True
        self.ros_servo_command=False
        self.process_command=False
        self.drv_srv=False
        self.was_home_top=False
        self.cartype=0 # default car type is niro
        self.vel=0      # target velocity
        self.angvel=0   # target angular velocity
        self.vel_p=0    # previous target velocity
        self.angvel_p=0 # previous target angular velocity
        self.angvel_integral=0 # angular velocity integral
        self.angvel_d_p=0 # previous angular velocity derivative
        
        self.lift_target=0  # target lift height 
        self.lift_vt=0.0    # target lift velocity
        self.lift_vt_p=0.0  # previous target lift velocity
        self.lift_io=0      # lift drive digital input output bits
        self.lift_dpos=0.0
        self.lift_dpos_p=0.0
        self.lift_tpos=0.0
        self.lift_current_target_vel=0
        self.lift_manual_positionmode=False
        self.lift_p_flag=False
        self.lift_arrived=False
        
        self.lw_state=0     # left wheel drive state
        self.rw_state=0     # right wheel drive state
        self.lift_state=0   # lift drive state
        self.lift_state2=0   # lift drive state2 which include lift position
        
        self.t_pos=-1    # target position of AGV 
        self.c_pos=-1    # current position of AGV
        self.c_pos_p=0
        self.c_pos_minor=0
        self.tan_theta=0 # tangent of AGV angle
        self.tan_theta_p=0 # previous tangent of AGV angle
        self.theta=0    # current angle of AGV
        self.theta_p=0  # previous angle of AGV
        
        self.lift_was_home=False
        self.master_homming=False
        self.lift_height=0.0    # lift height from driver to m
        self.lift_q=0   # lift motor position from driver
        self.lift_v=0.0  # lift velocity from driver to m/s
        self.lift_qdot=0 # lift qdot from driver rpm
        self.lift_op=0 # lift op bits
        self.lift_ready=False # lift ready flag from driver
        self.lift_run=False # lift run flag from driver
        self.lift_error=False # lift error from driver
        self.lift_was_home_fin=False
        self.lift_inposition=False # lift motion finish flag from driver
        self.lift_torque=0 # lift torque from driver : %
        
        self.lw_op=0 # left wheel drive op bits
        self.lw_ready=False # left wheel ready flag from driver
        self.lw_run=False # left wheel run flag from driver
        self.lw_error=False # left wheel error from driver
        self.lw_inposition=False # left wheel motion finish flag from driver
        self.lw_qdot=0  # left wheel drive qdot :motor rpm
        self.lw_v=0.0   # left wheel drive velocity : m/s
        
        self.rw_op=0 # right wheel drive op bits
        self.rw_ready=False # right wheel ready flag from driver
        self.rw_run=False # right wheel run flag from driver
        self.rw_error=False # right wheel error from driver
        self.rw_inposition=False # right wheel motion finish flag from driver
        self.rw_qdot=0  # right wheel drive qdot :motor rpm 
        self.rw_v=0.0   # right wheel drive velocity : m/s
        
        self.lw_vt=0.0      # left wheel drive velocity target  :m/s *
        self.rw_vt=0.0      # right wheel drive velocity target  :m/s *
        self.lw_qdott=0.0       # left wheel drive qdot target :motor rpm
        self.lw_qdott_p=0.0  # left wheel drive qdot target previous :motor rpm
        self.rw_qdott=0.0       # right wheel drive qdot target :motor rpm
        self.rw_qdott_p=0.0 # right wheel drive qdot target previous :motor rpm

        self.lift_state_ros=4   # lift drive state for agv monitoring
        self.driving_state=0    # driving state
        self.d_command=[0,0,0]  # wheel velocity cnt command 2's complement
        
        self.tracing_line=2  # tracing line, 0: right, 1: center, 2: left
        self.home_y_off=0.0  # home y offset

    
    def initialize_msg(self):
        # reqid= [[current job, previous job],[secondjob, previous], [received job, previous]] 
        self.battery_id=""
        self.op_id=""
        self.pin_state=0
        self.presetnum=0
        
    def initialize_task_attr(self):
        self.can_move=True
        self.can_lift=True
        self.can_adjust=True
        self.will_terminate=False
        self.task_clear=False
        self.task_clear_top=False
        self.control_mode=0
        self.reqid=[[0,0],[0,0],[0,0]]
        self.reqid_ros2=[[0,0],[0,0],[0,0]]
        self.op_state=[0,0,0]
        self.param=[0,0,0.0,0.0,"",""]
        self.axes=[0,0,0,0,0,0]
        self.buttons=[0,0,0,0,0,0,0,0,0,0,0] # [A,B,X,Y,L,R,SEL,START,LOGO,L_JOY(s1),R_JOY(s2)]
        self.hat=[0,0]
        self.mag_fdata=[0,0,0,0,False]
        self.mag_fdata_p=[0,0,0,0,False]
        self.mag_rdata=[0,0,0,0,False]
        self.mag_rdata_p=[0,0,0,0,False]
        self.lift_s_time=0
        self.lift_e_time=0
        self.drive_s_time=0
        self.mag_y_front=0.0
        self.mag_y_rear=0.0
        self.mag_frising_e=[False,0.0]
        self.mag_frising_el=[False,0.0]
        
        self.lcam_hole_pos=[0.0,0.0,0.0]
        self.rcam_hole_pos=[0.0,0.0,0.0]
        self.mag_fsettling_e=0
        self.mag_f_pos=0
        self.mag_fr_pos=0
        self.mag_fl_pos=0
        self.mag_fsettling_el=0
        self.is_tracing_srv= False
        self.f_lidar_is_collision_1 = False
        self.f_lidar_is_collision_2 = False
        self.f_lidar_is_collision_4 = False
        self.b_lidar_is_collision_1 = False
        self.b_lidar_is_collision_2 = False
        self.b_lidar_is_collision_4 = False
        self.lift_motion_start_time=0.0
        self.lift_motion_time_buffer=5.0
        self.arrival_time=0.0
        self.arrival_time_buffer=4.0
        self.arrival_pos_buffer=0.0
        self.thread_check_all=[0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.thread_check_total=False
        self.t_action_monitoring=[0,0,0,0,0,0,0]

         
    def cal_wv(self):
        """calculate wheel velocity from target speed and angular speed
            vel, angvel -> w_vt(float),w_qdott(int)
        """
        self.lw_vt=(self.vel-self.wheel_offset*self.angvel)
        self.rw_vt=-1.0*(self.vel+self.wheel_offset*self.angvel)
        lw_qdott=self.lw_vt/self.wheel_effect
        rw_qdott=self.rw_vt/self.wheel_effect
        if lw_qdott>self.wv_max:
            lw_qdott=self.wv_max
        elif lw_qdott<-self.wv_max:
            lw_qdott=-self.wv_max
        if rw_qdott>self.wv_max:
            rw_qdott=self.wv_max
        elif rw_qdott<-self.wv_max:
            rw_qdott=-self.wv_max
        
        lw_qdott= self.lw_qdott_p+self.decay*(lw_qdott-self.lw_qdott_p)
        rw_qdott= self.rw_qdott_p+self.decay*(rw_qdott-self.rw_qdott_p)
        
        self.lw_qdott_p=lw_qdott
        self.rw_qdott_p=rw_qdott  
        self.lw_qdott=int(lw_qdott)
        self.rw_qdott=int(rw_qdott)
             
    def convert2driver(self):
        """convert wheel & lift velocity to driver command
        lw_qdott, rw_qdott, lift_vt -> d_command
        """
        d_command=[0,0,0]
        d_command[0]=int(self.lw_qdott) 
        d_command[1]=int(self.rw_qdott)
        d_command[2]=int(self.lift_vt/self.lift_cnt2m)
        
        max=[self.wv_max,self.wv_max,int(self.lift_v_max_m/self.lift_cnt2m)]
        
        for i in range(3):
            if d_command[i]>max[i]:
                d_command[i]=max[i]
            elif d_command[i]<-max[i]:
                d_command[i]=-max[i] 
            if d_command[i]<0:
                d_command[i]=65536+d_command[i]
        
        self.d_command=d_command
        
    def send2driver(self,d_command):
        """send driver command to driver
        if descrete mode, write_registers to driver
        if not, readwrite_registers to driver : x_state will be updated
        Args:
            d_command ([int,int,int]): lw_qdott, rw_qdott, lift_qdott
        """
        self.lift_tpos=int((self.lift_target-self.lift_origin)*self.lift_m2cnt)
        # print(self.d_command)
        if d_command==None:
            d_command=self.d_command
        
        if not self.can_move:
            d_command[0]=0
            d_command[1]=0

        if not self.can_lift:
            d_command[2]=0

        if self.mbus_is_connected:
            try: self.client.write_registers(0x6200,[2,0,0,d_command[0],self.acc,self.dec,0,0x10],slave=1)
            except: print("Error in writing left wheel")
            try: self.client.write_registers(0x6200,[2,0,0,d_command[1],self.acc,self.dec,0,0x10],slave=2)
            except: print("Error in writing right wheel")
            
            if self.lift_was_home and not self.lift_was_home_fin:
                self.get_liftstate()
                self.get_liftstate2()
                pass
            else:
                if not self.lift_manual_positionmode:
                    if self.lift_current_target_vel==d_command[2]:
                        # if not self.can_move:
                        #     self.get_liftstate2()
                        pass
                    else:
                        try: 
                            self.client.write_registers(0x6200,[2,0,0,d_command[2],self.acc_lift,self.dec_lift,0,0x10],slave=3)
                            self.lift_current_target_vel=d_command[2]
                        except: 
                            print("Error in writing lift") 
                        self.lift_p_flag=False
                elif self.lift_manual_positionmode and self.can_lift :
                    # if not self.can_move or self.reqid[0][0]==305:
                    #     #self.get_liftstate()
                    self.get_liftstate2()
                    if self.lift_height-self.lift_target<0.01 and self.lift_height-self.lift_target>-0.01:
                        self.lift_arrived=True
                    else:
                        self.lift_arrived=False
                    if self.lift_tpos==self.lift_dpos and self.lift_p_flag:
                        pass
                        #################### lift ignoring bug fix test
                        hbit=self.lift_tpos>>16 & 0xffff #int.from_bytes((self.lift_tpos>>16 & 0xff).to_bytes(2,'little'),byteorder="little") ### test 필요함
                        lbit=self.lift_tpos & 0xffff #int.from_bytes((self.lift_tpos & 0xff).to_bytes(2,'little'),byteorder="little")
                        # print(self.lift_tpos, hbit,lbit)
                        try: 
                            self.client.write_registers(0x6200,[1,hbit,lbit,self.vel_posmode,self.acc_lift_p,self.dec_lift_p,0,0x10],slave=3)
                            self.lift_p_flag=True
                        except: 
                            print("Error in writing lift")  
                        #################### lift ignoring bug fix test
                    else:
                        hbit=self.lift_tpos>>16 & 0xffff #int.from_bytes((self.lift_tpos>>16 & 0xff).to_bytes(2,'little'),byteorder="little") ### test 필요함
                        lbit=self.lift_tpos & 0xffff #int.from_bytes((self.lift_tpos & 0xff).to_bytes(2,'little'),byteorder="little")
                        # print(self.lift_tpos, hbit,lbit)
                        try: 
                            self.client.write_registers(0x6200,[1,hbit,lbit,self.vel_posmode,self.acc_lift_p,self.dec_lift_p,0,0x10],slave=3)
                            self.lift_p_flag=True
                        except: 
                            print("Error in writing lift")  
                        self.lift_dpos_p=self.lift_dpos
                        self.lift_dpos=self.lift_tpos
                        self.lift_motion_start_time=time.time()
                else: pass
        else: 
            # not connected to driver
            if self.lift_manual_positionmode: 
                self.lift_height=self.lift_target
            pass
        
    def get_liftstate(self):
        if self.mbus_is_connected:
            self.lift_state=self.client.read_holding_registers(address=0x0b05,count=3,slave=3)                
            if not 'fcode' in dir(self.lift_state)and self.lift_state!=0:
                self.lift_op=self.lift_state.registers[0]
                self.lift_qdot=int.from_bytes(self.lift_state.registers[1].to_bytes(2,'little'),'little',signed=True)
                self.lift_v=self.lift_qdot*self.lift_cnt2m
                #op parsing
                self.lift_ready= bool(self.lift_op&1)
                self.lift_run= bool(self.lift_op>>1&1)
                self.lift_error= bool(self.lift_op>>2&1)
                self.lift_was_home_fin= bool(self.lift_op>>3&1)
                self.lift_inposition= bool(self.lift_op>>4&1)
                self.lift_torque=self.lift_state.registers[2]
        else:
            pass

    def get_liftstate2(self):
        if self.mbus_is_connected:
            self.lift_state2= self.client.read_holding_registers(address=0x602c,count=2,slave=3)
            if not 'fcode' in dir(self.lift_state2)and self.lift_state2!=0:
                # self.lift_io=self.lift_state2.registers[2]
                q_temp=(self.lift_state2.registers[0]<<16)+self.lift_state2.registers[1]
                self.lift_q=int.from_bytes(q_temp.to_bytes(4,'little'),'little',signed=True)
                #print(q_temp, self.lift_q, self.lift_state2.registers)
                self.lift_height=self.lift_q*self.lift_cnt2m+self.lift_origin
            else: pass
        else:
            pass

    def get_state(self):
        """ get state from driver and update state variables: error, op, qdot, v
        if descrete mode, read_registers from driver. if not, just convert state variables 
        """
        if self.mbus_is_connected:
            
            self.lw_state=self.client.read_holding_registers(address=0x0b05,count=2,slave=1)
            self.rw_state=self.client.read_holding_registers(address=0x0b05,count=2,slave=2)
            self.lift_state=self.client.read_holding_registers(address=0x0b05,count=3,slave=3)
            self.lift_state2= self.client.read_holding_registers(address=0x602c,count=2,slave=3)
           
                
            if not 'fcode' in dir(self.lw_state) and self.lw_state!=0:
                # print(dir(self.lw_state))
                self.lw_op=self.lw_state.registers[0]
                self.lw_qdot=int.from_bytes(self.lw_state.registers[1].to_bytes(2,'little'),'little',signed=True)
                self.lw_v=self.lw_qdot*self.w_qrpm2ms
                self.lw_ready= bool(self.lw_op&1)
                self.lw_run= bool(self.lw_op>>1&1)
                self.lw_error= bool(self.lw_op>>2&1)
                self.lw_inposition= bool(self.lw_op>>4&1)
            else: pass

            if not 'fcode' in dir(self.rw_state) and self.rw_state!=0:
                self.rw_op=self.rw_state.registers[0]
                self.rw_qdot=int.from_bytes(self.rw_state.registers[1].to_bytes(2,'little'),'little',signed=True)
                self.rw_v=self.rw_qdot*self.w_qrpm2ms
                self.rw_ready= bool(self.rw_op&1)
                self.rw_run= bool(self.rw_op>>1&1)
                self.rw_error= bool(self.rw_op>>2&1)
                self.rw_inposition= bool(self.rw_op>>4&1)
            else: pass

            if not 'fcode' in dir(self.lift_state)and self.lift_state!=0:
                self.lift_op=self.lift_state.registers[0]
                self.lift_qdot=int.from_bytes(self.lift_state.registers[1].to_bytes(2,'little'),'little',signed=True)
                self.lift_v=self.lift_qdot*self.lift_cnt2m
                #op parsing
                self.lift_ready= bool(self.lift_op&1)
                self.lift_run= bool(self.lift_op>>1&1)
                self.lift_error= bool(self.lift_op>>2&1)
                self.lift_was_home_fin= bool(self.lift_op>>3&1)
                self.lift_inposition= bool(self.lift_op>>4&1)
                self.lift_torque=self.lift_state.registers[2]
            else: pass
           
            if not 'fcode' in dir(self.lift_state2)and self.lift_state2!=0:
                # self.lift_io=self.lift_state2.registers[2]
                q_temp=(self.lift_state2.registers[0]<<16)+self.lift_state2.registers[1]
                self.lift_q=int.from_bytes(q_temp.to_bytes(4,'little'),'little',signed=True)
                self.lift_height=self.lift_q*self.lift_cnt2m+self.lift_origin
            else: pass
        else:
            pass
            
    def initial_drive_value_get(self):
        self.lift_dpos=self.lift_q
        self.lift_dpos_p=self.lift_q
        self.lift_tpos=self.lift_q
        self.lift_target= self.lift_origin + self.lift_tpos*self.lift_cnt2m
    
    def auto_drive_server(self):
        if self.reqid[0][0]==0x0101 and self.op_state[0]>1: # Forward Line Tracing
            self.is_tracing_srv = True
            if self.lt_init_flag_srv==False:
                self.lt_init_flag_srv=True
                self.lt_init_flag_stop=False
                self.lt_svel_srv=self.vel
                self.lt_acc_stime_srv = time.time()
                # cpos, tpos 업데이트시 lt_init_flag = False 해주기
                # lt_f 밖에서 초기화
            if self.c_pos == self.t_pos:
                self.lt_tvel_srv = 0.0  # 목표 속도
                if self.cartype==1 and (self.c_pos==6 or self.c_pos==18):
                    self.lt_time_gap=0.6
                elif self.cartype==3 and (self.c_pos==6 or self.c_pos==18):
                    # porter 2.0, bongo 5.0
                    self.lt_time_gap=6.0
                else:
                    self.lt_time_gap = 1.0 # 가감속 시간
            elif self.c_pos == 0:
                self.lt_tvel_srv = 0.6  # 목표 속도
                self.lt_time_gap = 3.0 # 가감속 시간
            elif self.c_pos == 1:
                self.lt_tvel_srv = 0.3  # 목표 속도
                self.lt_time_gap = 1.0 # 가감속 시간
            elif self.c_pos == 2:
                self.lt_tvel_srv = 0.3  # 목표 속도
                self.lt_time_gap = 1.0 # 가감속 시간
            elif self.c_pos == 3:
                self.lt_tvel_srv = 0.6
                self.lt_time_gap = 3.0
            elif self.c_pos == 4:
                self.lt_tvel_srv = 0.3  # 목표 속도
                self.lt_time_gap = 2.0 # 가감속 시간
            elif self.c_pos == 5:
                if self.cartype==1:
                    # ioniq
                    self.lt_tvel_srv=0.07
                else:    
                    self.lt_tvel_srv = 0.1  # 목표 속도
                self.lt_time_gap = 2.0 # 가감속 시간
            elif self.c_pos == 6:
                self.lt_tvel_srv = 0.5
                self.lt_time_gap = 3.0
            elif self.c_pos == 7:
                self.lt_tvel_srv = 0.5  # 목표 속도
                self.lt_time_gap = 3.0 # 가감속 시간
            elif self.c_pos == 8:
                self.lt_tvel_srv = 0.7  # 목표 속도
                self.lt_time_gap = 3.0 # 가감속 시간
            elif self.c_pos == 9:
                self.lt_tvel_srv = 0.5  # 목표 속도
                self.lt_time_gap = 3.0 # 가감속 시간
            elif self.c_pos == 10:
                self.lt_tvel_srv = 0.7  # 목표 속도
                self.lt_time_gap = 1.0 # 가감속 시간
            elif self.c_pos == 11:
                self.lt_tvel_srv = 0.1  # 목표 속도
                self.lt_time_gap = 3.0 # 가감속 시간
            elif self.c_pos == 12:
                self.lt_tvel_srv = 0.6  # 목표 속도
                self.lt_time_gap = 3.0 # 가감속 시간

            elif self.c_pos == 13:
                self.lt_tvel_srv = 0.7  # 목표 속도
                self.lt_time_gap = 2.0 # 가감속 시간
            elif self.c_pos == 14:
                self.lt_tvel_srv = 0.5  # 목표 속도
                self.lt_time_gap = 3.0 # 가감속 시간
            elif self.c_pos == 15:
                self.lt_tvel_srv = 0.6  # 목표 속도
                self.lt_time_gap = 3.0 # 가감속 시간
            elif self.c_pos == 16:
                self.lt_tvel_srv = 0.3  # 목표 속도
                self.lt_time_gap = 1.0 # 가감속 시간
            elif self.c_pos == 17:
                if self.cartype==1:
                    # ioniq
                    self.lt_tvel_srv=0.07
                else:    
                    self.lt_tvel_srv = 0.1  # 목표 속도
                self.lt_time_gap = 1.0 # 가감속 시간
            elif self.c_pos == 18:
                self.lt_tvel_srv = 0.7  # 목표 속도
                self.lt_time_gap = 3.0 # 가감속 시간
            elif self.c_pos == 19:
                self.lt_tvel_srv = 0.6  # 목표 속도
                self.lt_time_gap = 3.0 # 가감속 시간
            elif self.c_pos == 20:
                self.lt_tvel_srv = 0.7  # 목표 속도
                self.lt_time_gap = 3.0 # 가감속 시간
            elif self.c_pos == 21:
                self.lt_tvel_srv = 0.5  # 목표 속도
                self.lt_time_gap = 3.0 # 가감속 시간
            elif self.c_pos == 22:
                self.lt_tvel_srv = 0.3  # 목표 속도
                self.lt_time_gap = 2.0 # 가감속 시간
                    
            self.vel = cubic_interpolation(self.lt_svel_srv,self.lt_tvel_srv,time.time(),self.lt_acc_stime_srv,self.lt_acc_stime_srv+self.lt_time_gap)           
            # if self.vel==0.0:
            #     self.home_y_off=self.mag_fdata[1]*0.0001
        else: 
            self.lt_init_flag_srv=False
            self.is_tracing_srv=False

    def softstop(self):
        if self.is_tracing_srv:       
            if not self.lt_init_flag_stop:
                self.lt_init_flag_srv=False
                self.lt_init_flag_stop=True
                self.lt_dec_stime=time.time()
                self.lt_stopvel=self.vel
                self.lt_time_gap=1.0

            if time.time()-self.lt_dec_stime<self.lt_time_gap+1.0:
                self.vel = cubic_interpolation(self.lt_stopvel,0.0,time.time(),self.lt_dec_stime,self.lt_dec_stime+self.lt_time_gap)
            else:
                self.vel=0.0
        else:
            pass

    def joy_convert(self): 
        """convert joystick input to target speed and deadzone compensation
        axes, buttons -> vel, angvel, lift_vt 
        """
        vel=0
        angvel=0
        axes=self.axes
        buttons=self.buttons
        zvel=0
        for i in range(6):
            if axes[i]<0.1 and axes[i]>-0.1:
                axes[i]=0.0
        if not self.is_minorcontotal:          
            if buttons[4]:
                #max(angvel)*0.5+max(vel)<0.654
                maxvel_h=0.5
                vel=-maxvel_h*(0.5+0.5*axes[5])*axes[1]
                #print(vel, axes)
                maxangvel_h=0.2
                angvel=-maxangvel_h*(0.5+0.5*axes[5])*axes[0]
        liftspeed=300
        if self.buttons[5]:
            if self.buttons[3]: # Lift up (R + Y)
                zvel=liftspeed
            elif self.buttons[0]: # Lift down (R + A)
                zvel=-liftspeed
            else:
                zvel=0
            self.lift_manual_positionmode=False

        # set and filter vel
        self.filter_vel(vel,angvel)
        self.lift_vt=zvel
        # override control command at program shutdown
        if buttons[9] and buttons[10]: 
            self.vel=0.0
            self.angvel=0.0
            self.lift_vt=0.0  
        # homming command
        if self.buttons[9]: # Homming Command
                if self.buttons[3]: # lift homming (L_Joy_Btn + Y)
                    self.lift_homming()
                if self.buttons[0]: # lift homming flag reset (L_Joy_Btn + A)
                    if self.lift_was_home == True:
                        self.lift_was_home = False          
        
    def incomming_msg(self):
        """message treatment.
        """
        if self.op_state[0]==1 and not self.estop:
            self.reqid[0][0]=self.reqid_ros2[0][0]
            self.reqid[0][1]=self.reqid_ros2[0][1]
            self.reqid[1][0]=self.reqid_ros2[1][0]
            self.reqid[1][1]=self.reqid_ros2[1][1]
            self.reqid[2][0]=self.reqid_ros2[2][0]
            self.reqid[2][1]=self.reqid_ros2[2][1]
            
            case=self.reqid[0][0]
            self.op_state[0]=2
            if case==0x0021:
                #alarm reset
                pass
            elif case==0x0005:
                # cpos update
                self.mag_count_offset -= self.c_pos-self.param[0]
                self.t_pos=self.param[0]
                self.op_state[0]=255
            elif case==0x0009:
                # servo on 
                #self.op_state[0]=255
                if self.param[0]==1:
                    self.ros_servo_command=True
                elif self.param[0]==0:
                    self.ros_servo_command=False
                else:
                    self.op_state[0]=255
            elif case==0x0031:
                #can move 
                if self.param[0]==1:
                    self.can_move=False
                    self.driving_state=self.driving_state|0x0010
                elif self.param[0]==0:
                    self.can_move=True
                    self.driving_state=self.driving_state&0xffef
                self.op_state[0]=255
                pass
            elif case==0x0041:
                #can lift 
                if self.param[0]==1:
                    self.can_lift=False
                    self.lift_state_ros=self.lift_state_ros|0x0010
                elif self.param[0]==0:
                    self.can_lift=True
                    self.lift_state_ros=self.lift_state_ros&0xffef
                self.op_state[0]=255
                pass
            elif case==0x0051:
                #joystick function limited mode request
                if self.param[0]==1:
                    self.is_minorcontrol=False
                    self.driving_state=self.driving_state|0x0002
                elif self.param[0]==0:
                    self.is_minorcontrol=True
                    self.driving_state=self.driving_state&0xfffd
                self.op_state[0]=255
                pass
            elif case==0x0101:
                #agv move to 
                self.t_pos=self.param[0]
                # cartype assign
                if self.param[1]==0 or self.param[1]==1 or self.param[1]==2 or self.param[1]==3 or self.param[1]==4 or self.param[1]==5 or self.param[1]==6:
                    self.cartype=self.param[1]-1
                else: print("cartype error~~~~~~~~~~~~~~~~~~~~~")

                if self.t_pos>12 or self.c_pos>12:
                    # in case of pit 1 driving, follow the middle line
                    self.tracing_line = 1
                else:
                    # in case of PIT 2 driving, follow the left line
                    self.tracing_line = 2
 
                if self.c_pos==1 and self.t_pos>12:
                    self.arrival_time=time.time()+1.0

                if self.c_pos==6:
                    self.arrival_time=time.time()

                if self.c_pos==18:
                    self.arrival_time=time.time()

                # while driving, lift position control is not available because of longer communication time
                self.lift_manual_positionmode=False
                #line tracing intialization flag reset
                self.lt_init_flag_srv=False
                self.drive_s_time=time.time()
                pass
            elif case==0x0111:
                #lift go to origin
                self.lift_target=self.lift_origin
                self.lift_s_time=time.time()
                pass
            elif case==0x0121:
                #lift go to 
                self.lift_target=self.param[2]
                self.lift_s_time=time.time()
                pass
            elif case==0x0131:
                # lift inching position
                # cartype assign
                if self.param[1]==0 or self.param[1]==1 or self.param[1]==2 or self.param[1]==3 or self.param[1]==4 or self.param[1]==5 or self.param[1]==6:
                    self.cartype=self.param[1]-1
                else: print("cartype error~~~~~~~~~~~~~~~~~~~~~")
                
                if self.param[0]==1 or self.param[0]==2 or self.param[0]==3 or self.param[0]==4 or self.param[0]==6 or self.param[0]==7 or self.param[0]==8: 
                    self.presetnum=self.param[0]
                else:
                    self.op_state[0]=255                  

            elif case==0x141:
                # 상판 원점 잡기 커맨드 -> robot.topplate_controller
                # default : op_state[0]=2 (상판 원점 잡기 시작)
                pass
            else:
                self.op_state[0]=255

        elif self.op_state[0]==0 and self.task_clear:
            self.reqid[0][0]=self.reqid_ros2[0][0]
            self.reqid[0][1]=self.reqid_ros2[0][1]
            self.reqid[1][0]=self.reqid_ros2[1][0]
            self.reqid[1][1]=self.reqid_ros2[1][1]
            self.reqid[2][0]=self.reqid_ros2[2][0]
            self.reqid[2][1]=self.reqid_ros2[2][1]
            self.lift_target =self.lift_height
            self.lift_manual_positionmode=False
            self.task_clear_top=True
            self.task_clear=False
            self.t_pos=self.c_pos

            
    def _state_check(self):
        if self.estop:
            # emergency stop
            self.zero_command()
            
        else:        
            # # Collision detection from front and back Lidar Sensor
            if self.f_lidar_is_collision_1 and self.vel > 0:
                if ((self.c_pos==5 or self.c_pos==6) and self.t_pos==6) or ((self.c_pos==17 or self.c_pos==18) and self.t_pos==18) or ((self.c_pos==11 or self.c_pos==12) and self.t_pos==12):
                    self.lt_soft_stop_lidar=False
                else:
                    self.lt_soft_stop_lidar=True
            else:
                self.lt_soft_stop_lidar=False
            
            # manual mode
            if self.lt_soft_stop or self.lt_soft_stop_lidar or self.lt_soft_stop_server or self.lt_soft_stop_line:
                self.softstop()
            else:
                if self.reqid[0][0]==0x0101:
                    self.auto_drive_server()
                # else: 
                #     self.auto_drive()
                    
            if self.can_move:
                if self.is_tracing_srv:
                    self.pure_pursuit()
                    if self.reqid[0][0]==0x0101 and self.op_state[0]!=1:
                        if self.t_pos!=self.c_pos:
                            if self.t_pos-self.c_pos==2 or self.t_pos-self.c_pos==-6:
                                self.op_state[0]=200
                            elif self.t_pos-self.c_pos==1 or self.t_pos-self.c_pos==-8:
                                self.op_state[0]=240
                            elif self.t_pos-self.c_pos==3 or self.t_pos-self.c_pos==-7:
                                self.op_state[0]=150
                            elif self.t_pos-self.c_pos==4 or self.t_pos-self.c_pos==-6:
                                self.op_state[0]=100
                            else: 
                                self.op_state[0]=50

                        if self.cartype==3 and (self.c_pos==6 or self.c_pos==18):
                            self.arrival_pos_buffer=4.0
                        else:
                            self.arrival_pos_buffer=0.0
                        
                        if self.c_pos == self.t_pos and time.time()-self.arrival_time>self.arrival_time_buffer+self.arrival_pos_buffer:
                            self.op_state[0]=255
                            self.is_tracing_srv = False
                            self.home_y_off=self.mag_fdata[1]*0.0001
                            if self.home_y_off>0.03 :
                                self.home_y_off=0.03
                            elif self.home_y_off<-0.03:
                                self.home_y_off=-0.03
                        else:
                            pass
                else:
                    # manual mode
                    self.joy_convert()
                    if self.master_homming:
                        self.lift_homming()
                pass
            else: 
                self.joy_convert()
                # cannot move
                self.vel=0.0
                self.angvel=0.0
                self.is_tracing_srv = False
                pass

            if self.can_lift:
                pass
            else:
                self.lift_manual_positionmode=False
                self.lift_vt = 0.0           
            
    def _state_get(self,manual_input,srv_state,mag_data,mg_count,mg_pos_minor):
        """get state from joystick, magnetic sensor, and driver

        Args:
            manual_input ([axes,buttons,hat]): joystick inputs
            srv_state (bool) : motor servo state command 
            mag_data ([magresults,magresults]): magnetic sensor data
        """
        self.axes=manual_input[0]
        self.buttons=manual_input[1]
        self.hat=manual_input[2]
        self.mag_fdata_p=self.mag_fdata[:]
        self.mag_rdata_p=self.mag_rdata[:]
        self.mag_fdata=mag_data[0] #[pright,pmiddle,pleft,pcell,valid]
        self.mag_rdata=mag_data[1] #[pright,pmiddle,pleft,pcell,valid]
        self.c_pos_minor=mg_pos_minor
        self.drv_srv=srv_state
        if srv_state:
            self.driving_state=self.driving_state|0x0001
            self.lift_state_ros=self.lift_state_ros|0x0001
        else:
            self.driving_state=self.driving_state&0xfffe
            self.lift_state_ros=self.lift_state_ros&0xfffe

        #self.get_state()
        self.incomming_msg()
        
        if self.mbus_is_connected:
            self.mag_sensor_position_counter(mg_count)
        else: 
            self.c_pos=self.t_pos
        pass
     
    def _state_set(self): 
        """from desired motion, calculate wheel velocity and send it to driver
        vel, angvel, lift_vt -> lw_qdott, rw_qdott, lift_qdott -> d_command -> send2driver
        """
        #print(self.vel, self.angvel, self.lift_vt)
        self.cal_wv()
        self.convert2driver()
        self.send2driver(self.d_command)
    
    def zero_command(self):
        """zero the command of AGV motion
        vel, angvel, lift_vt -> 0
        """
        self.vel=0.0
        self.angvel=0.0
        self.lift_vt=0.0
    
    def simple_traction(self,xc,x1,vc):
        """constant speed traction function

        Args:
            xc (float): current value
            x1 (float): end value
            vc (float): current speed
        Returns:
            float : target speed
        """
        
        if abs(x1-xc)<self.lift_tolerance:
            return 0
        else:
            vt=(x1-xc)*self.control_hertz
            if vt>self.lift_v_max_m:
                vt=self.lift_v_max_m
            elif vt<-self.lift_v_max_m:
                vt=-self.lift_v_max_m
            elif abs(vc)>0.05 and abs(x1-xc)<self.lift_tolerance*10.0:
                vt=vt*0.1
                                
            return vt
        
    def lamp_interpolation(self,xc, x0 ,x1,tc,ts):
        """constant speed traction function

        Args:
            xc (float): current value
            x0 (float): start value
            x1 (float): end value
            tc (float): current time
            ts (float): start time
        Returns:
            float : interpolated value
            float : interpolated speed
            float : p-controller speed 
        """
        max_v=self.lift_v_max_m
        ta=0.3
        if tc<ts: tc=ts
        
        if abs(x1-x0)<ta*max_v:
            te=ts+2*ta
            vt=(x1-x0)/ta
            if tc>ts+ta:
                xt=x1-vt*(te-tc)*0.5
                vt=0
            elif tc>ts+2*ta:
                vt=0
                xt=x1
            else:
                vt=(x1-x0)/ta
                xt=x0+vt*(tc-ts)*0.5
        else:
            te=ts+ta+(x1-x0)/max_v
            if tc<te-ta:
                vt=max_v
                if tc<ts+ta:
                    xt=x0+vt*(tc-ts)*0.5
                else:
                    xt=x0+vt*(tc-ts-ta)+0.5*vt*ta
            else:
                vt=0
                xt=x1-vt*(te-tc)*0.5     
        
        if abs(xt-xc)<self.lift_tolerance:
            xe=0
        else:
            xe=x1-xc
        va=xe*0.1
        
        return xt,vt,va    
    
    def lift_homming(self):
        if not self.lift_was_home:
            self.lift_was_home=True
            self.lift_was_home_fin=False
            self.master_homming=False
            try : self.client.write_registers(0x6200,[3,0,0,100,200,200,0,0x10],slave=3)
            except :   
                self.lift_was_home=False
                self.master_homming=True

    def mag_sensor_position_counter(self,mgcount):
        """calculate position of AGV from front magnetic sensor right position
        In rising edge, c_pos is increased by 1
        mag_fr_pos >0  decelleration, mag_fr_pos==2 and mag_fl_pos==2 stop, mag_fr_pos==2 and mag_fl_pos==0 go back to find left marker
        fr_pos>0 and fl_pos==1 start to stop 
        Returns:
            int :  0 : valid data, 2 : invalid data
        """
                
        if (self.c_pos - self.mag_count_offset) != mgcount[0]:
            # ignore magnet counting at fork and confluence
            if self.c_pos==7 and self.tracing_line==0 and (time.time()-self.arrival_time)<self.arrival_time_buffer:
                self.mag_count_offset -= 1
            elif self.c_pos==1 and self.tracing_line==0 and (time.time()-self.arrival_time)<self.arrival_time_buffer+0.5:
                self.mag_count_offset -= 1
            # ignore magnet double counting under thr lift
            elif self.c_pos==18 and  (time.time()-self.arrival_time)<self.arrival_time_buffer:
                self.mag_count_offset -= 1
            elif self.c_pos==6 and  (time.time()-self.arrival_time)<self.arrival_time_buffer:
                self.mag_count_offset -= 1
            else: 
                self.arrival_time=time.time()
                self.lt_init_flag_srv=False

            # Add offset to get global position
            if mgcount[0]+self.mag_count_offset == 3 and self.t_pos > 12:
                self.mag_count_offset += 10
            if mgcount[0]+self.mag_count_offset == 21:
                self.mag_count_offset -= 14
            if mgcount[0]+self.mag_count_offset==12:
                self.mag_count_offset -= 12
                if self.t_pos==12:
                    self.t_pos=0

            print(f"position: C{mgcount[0] + self.mag_count_offset}  T{self.t_pos}")
            self.c_pos=mgcount[0] + self.mag_count_offset
        
        # Changes tracing line to move smoothly at fork and confluence
        if self.c_pos==1 and (self.c_pos_minor>=30 or self.c_pos_minor==4) and self.tracing_line==1:
            self.tracing_line=0
        if self.c_pos ==2 and self.tracing_line==0:
            self.tracing_line=1
        if self.c_pos==7 and (self.c_pos_minor>=30 or self.c_pos_minor==4) and self.tracing_line==1:
            self.tracing_line=0
        elif self.c_pos ==8 and self.tracing_line==0:
            self.tracing_line=1
        # pure pursuit gain change to move smoothly
        if self.tracing_line==0: 
            self.pp_gain=0.5
        else:
            self.pp_gain=1.5
            
        if self.mag_rdata[4]:  
            # validity of rear sensor data check  
            self.mag_y_rear=self.mag_rdata[self.tracing_line]*0.0001+self.mag__roff
        if self.mag_fdata[4]:
            # validity of front sensor data check
            # y position of front sensor calculation with offset
            self.mag_y_front=self.mag_fdata[self.tracing_line]*0.0001+self.mag__foff
            # angle of AGV calculation -> tan_theta
            self.tan_theta_p=self.tan_theta
            self.tan_theta=(self.mag_y_front-self.mag_y_rear)*0.5/self.mag_x_off
      
    def cal_linvel(self):
        if self.c_pos < self.t_pos:
            if self.t_pos-self.c_pos>1:
                self.vel=0.4
            else:   
                self.vel=0.2
        elif self.c_pos == self.t_pos:
            if self.mag_fr_pos==2 and self.mag_fl_pos==2:
                self.acc=100
                self.dec=100
                self.vel=0.0
            elif self.mag_fr_pos==2 and self.mag_fl_pos==3:
                self.acc=100
                self.dec=100
                self.vel=-0.05
            elif self.mag_fr_pos==2:
                self.acc=100
                self.dec=100
                self.vel=0.05
            else:  
                self.acc=300
                self.dec=300
                self.vel=0.1
        else :
            if self.t_pos-self.c_pos<-1:
                self.vel=-0.4
            else:   
                self.vel=-0.2 
    
    def filter_vel(self,vel,angvel,decay=0.7):
        """simplified low pass filter for velocity and angular velocity
        Args:
            vel (float): target velocity
            angvel (float): target angular velocity
            decay (float): decay factor
        """
        self.vel=vel*(1.0-decay)+self.vel_p*decay
        self.angvel=angvel*(1.0-decay)+self.angvel_p*decay
        self.vel_p=self.vel
        self.angvel_p=self.angvel
                       
    def pure_pursuit(self):
        """pure pursuit control. sensor selection with velocity sign
        vel, mag_y_front , mag_y_rear -> angvel
        """
        x0=self.mag_x_off
        self.angvel_p=self.angvel
        temp_angvel=0.0
        
        if self.vel>0.0:
            y1=self.mag_y_front
            if not self.mag_fdata[4]:
                #self.vel=0.0
                self.lt_soft_stop_line=True
                print("front sensor invalid data",self.mag_fdata)
            else:
                self.lt_soft_stop_line=False
            if y1==0.0:
                temp_angvel=0.0
            else:
                temp_angvel=self.vel*y1*2.0/(x0**2+y1**2)
        elif self.vel==0.0:
            temp_angvel=self.tan_theta
        else:
            y1=self.mag_y_rear
            if not self.mag_rdata[4]:
                self.vel=0.0
                #print("rear sensor invalid data",self.mag_rdata)
            if y1==0.0:
                temp_angvel=0.0
            else:
                temp_angvel=self.vel*y1*2.0/(x0**2+y1**2)
        
        self.angvel_integral+=temp_angvel
        
        if self.angvel_integral>self.ppi_max:
            self.angvel_integral=self.ppi_max
        elif self.angvel_integral<-self.ppi_max:
            self.angvel_integral=-self.ppi_max
        else: pass
        
        self.angvel=self.pp_gain*temp_angvel-(temp_angvel-self.angvel_d_p)*self.ppd_gain+self.angvel_integral*self.ppi_gain
        # print(self.angvel,self.pp_gain*temp_angvel,(temp_angvel-self.angvel_d_p)*self.ppd_gain,self.angvel_integral*self.ppi_gain)
        self.angvel_d_p=temp_angvel
        
def cubic_interpolation(x0,x1,tc,ts,te):
    """cubic interpolation function

    Args:
        x0 (float): start value
        x1 (float): end value
        tc (float): current time
        ts (float): start time
        te (float): end time
    Returns:
        float : interpolated value
    """
    if tc<ts: tc=ts
    if tc>te: tc=te
    if te-ts<0.1: te=ts+0.1
    t=(tc-ts)/(te-ts)
    return x0+(x1-x0)*t*t*(3-2*t)
   
def cubic_interpolation_v(x0,x1,tc,ts,te):
    """cubic interpolation function

    Args:
        x0 (float): start value
        x1 (float): end value
        tc (float): current time
        ts (float): start time
        te (float): end time
    Returns:
        float : interpolated value
    """
    if tc<ts: tc=ts
    if tc>te: tc=te
    if te-ts<0.1: te=ts+0.1
    t=(tc-ts)/(te-ts)
    return (x1-x0)*t*(6-6*t) 

        
if __name__ == "__main__":
    
    import os
    import sys
    sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
    from controller import joy
    
    is_GUI=True
    # Initialize Pygame
    if is_GUI:
        viz=joy.joy_gui()
        # initialize joystick if it is connected
        viz.is_joy()
    # initialize AGV controller. default : pseudo mode dot not connect to real AGV
    # agv=AGV(('192.168.0.223',4001))
    agv=AGV(('192.168.0.222',4003),True)

    # Main game loop 
    running = True
    while running:
        # Get joystick data
        if is_GUI:
            axes,buttons,hat=viz.get_joy() 
        else:
            axes=[0,0,0,0,0,0]
            buttons=[0,0,0,0,0,0,0,0,0,0]
            hat=[0,0]
        mgresults=[[0,0,0,0,False],[0,0,0,0,False]]
        agv._state_get([axes,buttons,hat],mgresults)
        agv._state_check()
        
        
        # Draw joystick data
        if is_GUI:
            viz.screen_set([agv.lw_qdot,agv.rw_qdot],axes,buttons,hat)        
        # Check for quit events
        if buttons[0] and buttons[1]: 
            if is_GUI:
                viz.quit_event()
        else: pass
        
        agv._state_set()
        
        
        if is_GUI:
            running = viz.check_event()
        else: pass
        
        time.sleep(0.05)
             
    # Quit process1: wait for AGV to stop
    if agv.mbus_is_connected:
        while agv.lw_op!=19 or agv.rw_op!=19:
            agv.send2driver([0,0,0])
            agv.get_state()
            time.sleep(0.05)
        # Quit process2: print final speed and quit
    print(agv.lw_v,agv.rw_v)
    # Quit Pygame
    joy.pygame.quit()
