import time

def control_servo(self, servo):
    # Connect to the client
    servo._state_init()
    servo._state_get(self.t_pos_fastech[servo.id],self.t_vel[servo.id],self.estop_total,self.can_movetop,self.t_action_fastech[servo.id])
    self.t_pos_fastech[servo.id] = servo.pos
    while not self.process_command:
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
        time.sleep(0.16)
    servo.servo_off()

def control_agv(self):
    # Main game loop
    running = True
    temp_time = time.time()
    
    while running:
        self.agv._state_get([self.axes,self.buttons,self.hat],self.is_fastech_on,self.mgresults,self.mg_pos,self.mg_pos_minor)
        self.agv.process_command=self.process_command
        self.agv.was_home_top=self.was_home
        self.agv.t_action_monitoring=self.t_action_fastech
        self.agv.thread_check_all=self.thereadcheck
        
        if (self.buttons[9] and self.buttons[10]) or self.agv.will_terminate: # Robot Off (L_Joy_Btn + R_Joy_Btn)
            running = False
            self.process_command=1
            break
        
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
        
        ############################################### 
        ######### timeout check ######################
        temp_time2 = time.time()
        dt_time=temp_time2-temp_time
        temp_time = temp_time2
        # in case of virtual loop, loop speed is 0.16sec 
        if dt_time<0.01:
            time.sleep(0.16-dt_time)
        ###############################################             