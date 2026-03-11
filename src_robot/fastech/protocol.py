import socket
from fastech import servolist as sl

class fastech:
    commstate=0
    di=0
    do=0
    flag=0
    com_apos=0
    com_rpos=0
    pos=0
    pe=0
    vel=0
    PT_num=0
    servo_state=0
    error=0
    pLimit=0
    nLimit=0
    pos_bias=0
    target_apos=0  
    pSlim=134,217,727
    nSlim=-134,217,728
    error_code=0
    motion_code=0 
    motion_dir=0
    is_alarm=0
    is_origin=0
    is_moving=0
    is_inpause=0
    is_peover=0
    is_overload=0
    is_stop=0
    is_originfinding=0
    is_originfinding_ok=0
    is_inposition=0
    is_costvel=0
    comm_num=0
    id=-1   
    desired_vel=0
    desired_pos=0
    desired_state=0
    desired_action=0
    can_move=True
    can_move_command=True
    estop=False
    will_find_origin=False
    version=0
    internal_state=0
    commbit=0
    origin_find_flag=False
    thread_check_fs=0
    badcomm_count=0
    term_count=0

    def __init__(self,address):
        self.ip=address[0]
        self.port=address[1]
        self.client=FastechClient((self.ip,self.port))
        self.client.connect()
        self.find_id()
        
    def find_id(self):  
        if self.ip==sl.trY:
            self.id=0
            self.version=1
        elif self.ip==sl.trX:
            self.id=1
            self.version=1
        elif self.ip==sl.rtZ:
            self.id=2
            self.version=1
        elif self.ip==sl.Lr:
            self.id=3
            self.version=1
        elif self.ip==sl.Rr:
            self.id=4
            self.version=1
        elif self.ip==sl.Lz:
            self.id=5
            self.version=1
        elif self.ip==sl.Rz:
            self.id=6
            self.version=1
        else:
            self.id=-1
        
    def packet_gen(self,command,data):
        """command packet genenration

        Args:
            command (byte): command address
            data (bytes): data corresponding to command address.
            sync_num (int): sync_byte for distinguishing the packet. it should be changed every command.
        """
        length = len(data)
        self.comm_num+=1
        self.comm_num=self.comm_num%256
        # Method code here
        message = bytearray(4)
        message[0] = 0xAA
        message[1] = 3+length
        message[2] = self.comm_num
        message[3] = 0x00
        message=message+command+data
        
 
        return message
    
    def servo_on(self):
        if self.servo_state!=1:
            self.client.send_data(self.packet_gen(b'\x2A',b'\x01'))
            data = self.client.receive_data(6,self.comm_num)
            comm_result=self.com_state(data[0])
            if comm_result:
                #print("servo on failed  ",self.ip)
                self.internal_state=1
                pass 
            else :
                self.servo_state=1
        else :
            comm_result=0
        return comm_result    
    
    def servo_off(self):
        if self.servo_state!=0:
            self.client.send_data(self.packet_gen(b'\x2A',b'\x00'))
            data = self.client.receive_data(6,self.comm_num)
            comm_result=self.com_state(data[0])
            if comm_result:
                #print("servo off failed  ",self.ip)
                self.internal_state=2
                pass 
            else :
                self.servo_state=0
        else :
            comm_result=0
        return comm_result    
    
    def set_temp_param(self,add,param):
        """temp parameter setting

        Args:
            add (int): param address
            param (int): data

        Returns:
            result: comm_result
        """
        datapacket=add.to_bytes(1,byteorder='little',signed=True)+param.to_bytes(4,byteorder='little',signed=True)
        self.client.send_data(self.packet_gen(b'\x12',datapacket))
        data = self.client.receive_data(6,self.comm_num)
        comm_result=self.com_state(data[0])
        return comm_result
    
    def set_torque_origin(self,dir,offset,vel1=500,vel2=300):
        """torque origin setting 
        Args:
            dir (int): 0 cw, 1 ccw
            offset (int): joint z-pulse to origin
        """
        self.set_temp_param(14,vel1)
        self.set_temp_param(15,vel2)
        self.set_temp_param(17,7)
        self.set_temp_param(18,dir)
        self.set_temp_param(19,offset)
        self.set_temp_param(27,30)
    
    def set_gains(self,gain):
        """gain parameter setting

        Args:
            gain (int): 0~63 , 63 is lowest gain(high inertia ratio)  
        """
        self.set_temp_param(22,gain)
        
    def set_limit_origin(self,dir,offset,vel1=500,vel2=300):
        """limit origin setting 
        Args:
            dir (int): 0 cw, 1 ccw
            offset (int): joint z-pulse to origin
        """
        self.set_temp_param(14,vel1)
        self.set_temp_param(15,vel2)
        self.set_temp_param(17,2)
        self.set_temp_param(18,dir)
        self.set_temp_param(19,offset)
        
    def set_origin_method(self,method,offset,dir=0,vel1=500,vel2=300):
        """homming method setting

        Args:
            method (int): 0 limit sensor, 1 torque sensor with z-pulse
            offset (int): offset value
            dir (int, optional): 0 cw 1 ccw. Defaults to 0.
            vel1 (int, optional): velocity to find origins. Defaults to 500.
            vel2 (int, optional): velocity after touch to origin. Defaults to 300.
        """
        if method==0:
            self.set_limit_origin(dir=dir,offset=offset,vel1=vel1,vel2=vel2)
        elif method==1:
            self.set_torque_origin(dir=dir,offset=offset,vel1=vel1,vel2=vel2)
        else:
            pass
    
    
    def parse_43(self,data):
        self.commstate=self.com_state(data[0])
        self.di=data[1:5]
        self.do=data[5:9]
        self.flag=int.from_bytes(data[9:13],byteorder='little',signed=False)
        self.flag_state()
        self.com_apos=int.from_bytes(data[13:17],byteorder='little',signed=True)
        self.pos=int.from_bytes(data[17:21],byteorder='little',signed=True)
        self.pe=int.from_bytes(data[21:25],byteorder='little',signed=True)
        self.vel=int.from_bytes(data[25:29],byteorder='little',signed=True)
        self.PT_num=int.from_bytes(data[29:33],byteorder='little',signed=True)
    
    def flag_state(self):
        
        flag=self.flag
        self.error=flag&0x01
        self.pLimit=(flag>>1) & 0x01
        self.nLimit=(flag>>2) & 0x01
        self.pSlim=(flag>>3) & 0x01
        self.nSlim=(flag>>4) & 0x01
        self.error_code=(flag>>7) & 0b111111111
        self.is_peover=(flag>>8) & 0x01 or (flag>>10) & 0x01
        self.is_overload=(flag>>11) & 0x01

        self.is_stop=(flag>>16) & 0x03
        self.is_originfinding=(flag>>18) & 1
        self.is_inposition=(flag>>19) & 1
        self.servo_state=(flag>>20) & 1
        self.is_origin=(flag>>23) & 1
        self.is_originfinding_ok=(flag>>25) & 1
        
        self.motion_code=(flag>>26) & 0b111111
        self.motion_dir=(flag>>26) & 1
        self.is_moving=(flag>>27) & 1
        self.is_inpause=(flag>>28) & 1
        self.is_costvel=(flag>>31) & 1
        if self.error: 
            #print(f'Error : {self.ip} Errorall')
            #print(f'Error code for {self.ip} : EC {self.error_code} , 8~16 bit of flag')
            self.internal_state=self.error_code<<5
            pass
    
    def state_embed(self):
        self.client.send_data(self.packet_gen(b'\x43',b''))
        data = self.client.receive_data(5+33,self.comm_num)
        self.parse_43(data)
           
    def servo_rmove(self,position,velocity):
        
        is_success=0
        rmove_data = position.to_bytes(4,byteorder='little',signed=True)+velocity.to_bytes(4,byteorder='little',signed=True)
        #self.state_embed()
        if self.commstate!=0:
            is_success=-1
            pass
        elif self.servo_state==0:
            #print(f" servo is off,... servo on {self.ip}")
            self.internal_state=3
            is_success=-2
        else:
            if self.is_inposition==1:
                self.client.send_data(self.packet_gen(b'\x35',rmove_data))
                data = self.client.receive_data(5+1,self.comm_num)
                self.commstate=self.com_state(data[0])
                if self.commstate==0:
                    self.com_rpos=position
                    is_success=0
                else :
                    is_success=-3              
            else:
                over=position-self.com_rpos
                rover=over.to_bytes(4,byteorder='little',signed=True)
                self.client.send_data(self.packet_gen(b'\x39',rover))
                data = self.client.receive_data(5+1,self.comm_num)
                self.commstate=self.com_state(data[0])
                if self.commstate==0:
                    self.com_rpos=position
                    is_success=0
                else :
                   is_success=-4

        return is_success
    
    def servo_reset(self):
        self.client.send_data(self.packet_gen(b'\x2b',b''))
        data = self.client.receive_data(6,self.comm_num)
        self.commstate=self.com_state(data[0])
        return self.commstate
    
    def servo_alarmcheck(self):
        self.client.send_data(self.packet_gen(b'\x2e',b''))
        data = self.client.receive_data(6+1,self.comm_num)
        self.commstate=self.com_state(data[0])
        if self.commstate:
            pass
        else:
            self.is_alarm=data[1]
        return self.commstate
    
    def servo_amove(self,position,velocity):
        
        is_success=0
        amove_data = position.to_bytes(4,byteorder='little',signed=True)+velocity.to_bytes(4,byteorder='little',signed=True)
        aover=position.to_bytes(4,byteorder='little',signed=True)
        #self.state_embed()
        if self.commstate!=0:
            is_success=-1
            pass
        elif self.servo_state==0:
            #print(f" servo is off,... servo on {self.ip}")
            self.internal_state=3
            is_success=-2
        else:
            if self.is_inposition==1:
                self.client.send_data(self.packet_gen(b'\x34',amove_data))
                data = self.client.receive_data(6,self.comm_num)
                self.commstate=self.com_state(data[0])
                if self.commstate==0:
                    is_success=0
                else :
                    is_success=-3              
            else:
                self.client.send_data(self.packet_gen(b'\x38',aover))
                data = self.client.receive_data(6,self.comm_num)
                self.commstate=self.com_state(data[0])
                if self.commstate==0:
                    is_success=0
                else :
                   is_success=-4
        return is_success
       
    def servo_jog(self,velocity):
        is_success=0
        if velocity>=0:
            direction=b'\x01'
        else:  
            direction=b'\x00'
        
        target_velocity=abs(int(velocity))
        #self.state_embed()
        
        if self.commstate!=0:
            is_success=-1
        elif self.servo_state==0:
            #print(f" servo is off,... servo on {self.ip}")
            self.internal_state=3
            is_success=-2
        else:
            if target_velocity==0: 
                self.servo_jog_stop()
                data = self.client.receive_data(6,self.comm_num)
                self.commstate=self.com_state(data[0])
                if self.commstate==0:
                    is_success=0
                else :
                   is_success=-10

            elif self.is_inposition==1:
                self.client.send_data(self.packet_gen(b'\x37',target_velocity.to_bytes(4,byteorder='little',signed=True)+direction))
                data = self.client.receive_data(6,self.comm_num)
                self.commstate=self.com_state(data[0])
                if self.commstate==0:
                    is_success=0
                else :
                    is_success=-3              
            elif self.is_costvel==1 and self.vel^velocity>=0 :
                self.client.send_data(self.packet_gen(b'\x3A',target_velocity.to_bytes(4,byteorder='little',signed=True)))
                data = self.client.receive_data(6,self.comm_num)
                self.commstate=self.com_state(data[0])
                if self.commstate==0:
                    is_success=0
                else :
                   is_success=-4
            elif self.is_costvel==1 and self.vel^velocity<0 :
                #stop first
                self.client.send_data(self.packet_gen(b'\x31',b''))
                data = self.client.receive_data(6,self.comm_num)
                self.commstate=self.com_state(data[0])
                if self.commstate==0:
                    is_success=0
                else :
                   is_success=-5
            else: 
                #print(f" servo cannot jog for its state {self.ip}")
                self.internal_state=4
                is_success=-9
        
        return is_success
    
    def servo_jog_stop(self):
        self.client.send_data(self.packet_gen(b'\x31',b''))
        data=self.client.receive_data(6,self.comm_num)
        self.commstate=self.com_state(data[0])
        if self.commstate==0:
            return 0
        else :
            return -1
        
    def servo_estop(self):
        self.client.send_data(self.packet_gen(b'\x32',b''))
        data=self.client.receive_data(6,self.comm_num)
        self.commstate=self.com_state(data[0])
        if self.commstate==0:
            return 0
        else :
            return -1
    
    def servo_move_origin(self):
        self.client.send_data(self.packet_gen(b'\x33',b''))
        data=self.client.receive_data(6,self.comm_num)
        self.commstate=self.com_state(data[0])
        if self.commstate==0:
            return 0
        else :
            return -1
    
    def servo_setDO(self, DO):
        """
        Sets the digital output (DO) of the servo motor.

        Args:
            DO (list): A list of 9 bits representing the desired DO state. Each bit should be either 0 or 1.
            e.g. [0,1,1,1,1,1,1,1,1]
        Returns:
            int: Returns 0 if the DO is successfully set, -1 if there is a communication error, -2 if the DO length is not 9 bits, or -3 if the DO format is invalid.
        """
        if len(DO) != 9:
            print("DO should be 9 bits")
            return -2
        else:
            try:
                DO_bits = int(''.join(map(str, DO)), 2)
            except:
                print("DO should be 9 bits like [0,1,1,1,1,1,1,1,1]")
                return -3
            user_do_bits = (self.do >> 15) & 0b111111111
            needtocommand = DO_bits ^ user_do_bits
            setmask = 0
            resetmask = 0

            if needtocommand == 0:
                return 0
            else:
                setmask = needtocommand & DO_bits
                resetmask = needtocommand & (~DO_bits)
                setmask = setmask << 15
                resetmask = resetmask << 15
                setmask = setmask.to_bytes(4, byteorder='little', signed=False)
                resetmask = resetmask.to_bytes(4, byteorder='little', signed=False)
                self.client.send_data(self.packet_gen(b'\x20', setmask + resetmask, self.comm_num))
                data = self.client.receive_data(6 + 1, self.comm_num)
                self.commstate = self.com_state(data[0])
                if self.commstate == 0:
                    return 0
                else:
                    return -1
                
    def servo_pause(self,pause):
        """pause the servo motion

        Args:
            pause (bool): true for pause, false for resume

        """
        if pause:
            self.client.send_data(self.packet_gen(b'\x58',b'\x01'))
        else:
            self.client.send_data(self.packet_gen(b'\x58',b'\x00'))
        data = self.client.receive_data(6,self.comm_num)
        self.commstate=self.com_state(data[0])
        if self.commstate==0:
            return 0
        else :
            return -1

    def com_state(self,data):
        """
        Determines the communication state based on the given data.

        Args:
            data (bytes): The data received from the communication.

        Returns:
            int: The communication state code:
                - 0: Communication successful.
                - 1: Communication error occurred.
        """
        self.combit=data
        if data == 0 or data == 134:
            return 0
        # else:
        #     print("Invalid command.")
        #     return 1
        elif data == 128:
            print(f"servo{self.id} return error: {data} Frame type error")
            #print("Frame type error: Frame type is not feasible.")
        elif data == 129:
            print(f"servo{self.id} return error: {data} Data region error")
            pass
            #print("Data error: data is not in range of feasible region.")
        elif data == 130:
            print(f"servo{self.id} return error: {data} data length error")
            pass
            #print("data length error: data length is not feasible.")
        elif data == 133:
            print(f"servo{self.id} return error: {data} Operation failed: moving, stop, servo off, Z-pulse origin command w/o external encoder and etc.")
            pass
            #print("Operation failed : moving, stop, servo off, Z-pulse origin command w/o external encoder and etc.")
        elif data == 134:
            pass
            #print("Reset failed : already in reset state or in servo on state.")
            return 0
        elif data == 135:
            print(f"servo{self.id} return error: {data} Servo on failed by alarm")
            pass
            #print("Servo on failed by alarm : please check alarm status.")
        elif data == 136:
            pass 
            #print("Servo on failed by eSTOP : please check stop input.")
        elif data == 137:
            pass
            #print("Servo on failed by external control : servo state is controlled by external input.")
        else:
            pass
            #print("Unknown error",data)
        return 1
        
                
    def __del__(self):
        self.client.close()
        print(f"client {self.ip} is closed")

    def _state_init(self):
        #self.set_origin_method(sl.homming_method[self.id],sl.offsets[self.version][self.id],sl.homming_dir[self.id])
        #self.set_gains(sl.gains[self.id])
        try: self.servo_alarmcheck()
        except: print(f"bad comm state for alarm check {self.id}")
        try: self.servo_reset()
        except: print(f"bad comm state for reset {self.id}")
        # self.servo_on()
        
    
    def _state_get(self,desired_pos,desired_vel,estop,can_move,desired_action=0):
        """set servo action

        Args:
            desired_pos (int): position
            desired_vel (int): velocity
            estop (bool): True for emergency stop, False for normal operation
            can_move (bool): True for normal operation, False for pause
            desired_action (int, optional): 1 for finding origin. Defaults to 0.
        """
        try : 
            self.state_embed()
            
        except: 
            #print(f"bad comm state for get data {self.id}")
            self.badcomm_count+=1

        self.term_count+=1

        if self.term_count>20:
            if self.badcomm_count>10:
                self.client.close()
                self.client.connect()
                self.thread_check_fs=0
                print(f"bad comm state for get data {self.id} : {self.badcomm_count}/20")
            else:
                self.thread_check_fs=1    
            self.term_count=0
            self.badcomm_count=0
            
        if self.commstate!=0:
            #print(f"communication error {self.ip}")
            self.internal_state=5
        else:
            pass
        self.desired_pos=desired_pos
        self.desired_vel=desired_vel
        self.estop=estop
        self.can_move=can_move
        self.desired_action=desired_action
        
    
    def _state_check(self):
        
        if self.estop:
            self.will_find_origin=False
            if self.is_stop== 1 or self.is_stop==3:
                pass
                self.can_move_command=False
            else:
                try: self.servo_estop()
                except: print(f"bad comm state set estop {self.id}")
                self.can_move_command=False
        else:    
            if self.error:
                # servo is in error state
                #print(f"error {self.ip}")  
                self.can_move_command=False  
            else: 
                if self.can_move:
                    # if self.is_inpause==1:
                        # self.servo_pause(False)
                        # self.can_move_command=False
                    # elif self.is_moving:
                    #     self.can_move_command=False
                    # else:  
                    #    self.can_move_command=True
                    self.can_move_command=True
                else : 
                    if  self.is_stop==1 or self.is_stop==3:  # self.is_inpause==1 or
                        self.can_move_command=False
                        pass
                    else: 
                        self.can_move_command=False
                        try: self.servo_pause(True) 
                        except: print(f"bad comm state set pause {self.id}")
        if self.desired_action==1 and self.is_originfinding==0 and not self.origin_find_flag:
            self.will_find_origin=True   
        else:
            pass  
                
    def _state_set(self):
        if self.estop:
            pass
        else:
            if self.desired_action==0 or self.desired_action==2 or self.desired_action==1:
                if self.is_alarm or self.error:
                    try: self.servo_reset()
                    except: print(f"bad comm state for reset {self.id}")
        
        if self.can_move_command and self.can_move:
            if self.servo_state==0 and self.desired_action==2:    
                try :self.servo_on()
                except: print(f"bad comm state for set servo on {self.id}")
            if self.will_find_origin:
                try: self.servo_move_origin()
                except: print(f"bad comm state for set origin {self.id}")
                self.will_find_origin=False
                self.origin_find_flag=True
            else:
                if self.desired_action==0:
                    if self.is_originfinding==1 and not self.is_originfinding_ok==1:
                        pass
                    else:
                        if self.com_apos==self.desired_pos:
                            pass
                        else:
                            try: self.servo_amove(self.desired_pos,abs(int(self.desired_vel)))
                            except: print(f"bad comm state for set move {self.id}")
                elif self.desired_action==4:
                    if self.desired_pos==self.com_apos:
                        pass
                else:
                    pass
        else:
            pass
        if self.desired_action==3:
            try : self.servo_off()
            except: print(f"bad comm state servo off {self.id}")
        
class FastechClient:
    def __init__(self, server_address):
        self.server_address = server_address
        #self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    def connect(self):
        temp_flag=1
        while temp_flag!=0:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            temp_flag=self.sock.connect_ex(self.server_address)
            print(f"connection result of {self.server_address} : {temp_flag} ")
            if temp_flag!=0:
                self.sock.close()


    def send_data(self, data):
        for i in range(3):
            try: 
                self.sock.sendall(data)
                break
            except: pass

    def receive_data(self, buffer_size,syncnum):
        self.sock.settimeout(0.15)
        try: raw_data = self.sock.recv(buffer_size)
        except: 
            rdata=b''
            #print(f"timeout")
            return rdata
        if raw_data[0] == 0xaa and raw_data[3] == 0x00:
            len=raw_data[1]
            if len+2==buffer_size:
                if raw_data[2]==syncnum:
                    rdata=raw_data[5:buffer_size]
                    self.commstate=raw_data[5]
                else:
                    rdata=b''
                    #print("packet sync num error")
            else:
                #print("rPacket length error") 
                rdata=b''
        else:
            #print("rPacket header error")
            rdata=b''
        return  rdata
    

    def close(self):
        self.sock.close()
    
    