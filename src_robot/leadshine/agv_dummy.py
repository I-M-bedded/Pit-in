#from pymodbus.client import ModbusTcpClient as mbus
import time

class AGV:
    def __init__(self, server_address, is_real_robot=False):
        #self.client = mbus(host=server_address[0],port=server_address[1])
        self.mbus_is_connected=False
        self.version = 1
        self.initialize_data()
        self.initialize_msg()
        self.initialize_task_attr()
    
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
        self.ros_servo_command=False
        self.process_command=False
        self.drv_srv=False
        self.was_home_top=False
        self.cartype=0 # default car type is niro
        self.t_pos=-1    # target position of AGV 
        self.c_pos=-1    # current position of AGV
        self.lift_was_home=False
        self.master_homming=False
        self.lift_state_ros=4   # lift drive state for agv monitoring
        self.driving_state=0    # driving state
        self.d_command=[0,0,0]  # wheel velocity cnt command 2's complement
        self.lift_height=0.0    # lift height
    
    def initialize_msg(self):
        # reqid= [[current job, previous job],[secondjob, previous], [received job, previous]] 
        self.battery_id=""
        self.op_id=""
        self.pin_state=0
        self.presetnum=0
        
    def initialize_task_attr(self):
        self.can_move=True
        self.can_lift=True
        self.will_terminate=False
        self.task_clear=False
        self.task_clear_top=False
        self.reqid=[[0,0],[0,0],[0,0]]
        self.reqid_ros2=[[0,0],[0,0],[0,0]]
        self.op_state=[0,0,0]
        self.param=[0,0,0.0,0.0,"",""]
        self.lcam_hole_pos=[0.0,0.0,0.0]
        self.rcam_hole_pos=[0.0,0.0,0.0]
        self.is_tracing_srv= False
        self.thread_check_all=[0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.thread_check_total=False
        self.t_action_monitoring=[0,0,0,0,0,0,0]
                
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
            elif case==0x0009:
                # servo on 
                #self.op_state[0]=255
                if self.param[0]==1:
                    self.ros_servo_command=True
                elif self.param[0]==0:
                    self.ros_servo_command=False
                else:
                    self.op_state[0]=255
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
            self.task_clear_top=True
            self.task_clear=False
            self.t_pos=self.c_pos           
            
    def _state_get(self,manual_input,srv_state,mag_data,mg_count,mg_pos_minor):
        """get state from joystick, magnetic sensor, and driver

        Args:
            manual_input ([axes,buttons,hat]): joystick inputs
            srv_state (bool) : motor servo state command 
            mag_data ([magresults,magresults]): magnetic sensor data
        """
        self.drv_srv=srv_state
        if srv_state:
            self.driving_state=self.driving_state|0x0001
            self.lift_state_ros=self.lift_state_ros|0x0001
        else:
            self.driving_state=self.driving_state&0xfffe
            self.lift_state_ros=self.lift_state_ros&0xfffe

        #self.get_state()
        self.incomming_msg()
        
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
    # dummy address
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
        
        # Draw joystick data
        if is_GUI:
            viz.screen_set([agv.lw_qdot,agv.rw_qdot],axes,buttons,hat)        
        # Check for quit events
        if buttons[0] and buttons[1]: 
            if is_GUI:
                viz.quit_event()
        else: pass
        
        if is_GUI:
            running = viz.check_event()
        else: pass
        
        time.sleep(0.05)
    
    # Quit Pygame
    joy.pygame.quit()
