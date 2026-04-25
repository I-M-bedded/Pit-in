import threading
import time
import sys
import os
import platform
import threading
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from fastech import protocol as fs
from fastech import servo_state as ss
from leadshine import agv_dummy as aj
from controller import ik
from config import RobotConfig
from suprevisor import TopPlateSupervisor  # Supervisor import
from robot_test.macro_micro import ControlConfig as PBVSConfig
from robot_test.TCPcontroller import TCPcontroller

if platform.system() == 'Linux':
    is_windows = False
    is_ros_if = True
else:
    is_windows = True
    is_ros_if = False

try: 
    from ros2 import interface as rif
    print("[SYSTEM] ROS2 Interface loaded successfully.")
except ImportError as e: 
    print(f"[WARNING] ROS2 Interface import failed: {e}")
    print("[WARNING] The ROS2 background thread will NOT be started! Check if VSCode is using a venv without rclpy.")
    is_ros_if = False

if is_windows:
    from controller import joy
else:
    pass 

class robot:
    def __init__(self, config: RobotConfig):
        # 1. Config & Hardware Init
        self.config = config
        for key, value in vars(self.config).items():
            setattr(self, key, value)
        
        self.input_manager = self.config.input
        self.servos = [fs.fastech(address) for address in self.config.ip_addresses]
        self.servo_states = ss.ServoState()
        self.agv = aj.AGV("", self.config.is_real_robot)
        
        # 2. Shared Tools
        self.topik = ik.Topik(self.config.version)
        self.pingap = ik.niro_gap
        self.ikmode = True
        
        self.pbvs_tcp_ctrl = TCPcontroller(self.topik, PBVSConfig(version=self.config.version))
        
        # 3. Shared State
        self.c_pos = [0]*7
        self.t_pos = [0]*7
        self.t_vel = list(self.config.t_vel_default)
        self.t_action = [0]*7
        self.t_pos_fastech = [0]*7
        self.t_action_fastech = [0]*7
        self.mg_pos=[0,0]
        self.mg_pos_minor=0
        
        self.process_command = 0
        self.thereadcheck = [0]*13
        self.was_home = False
        
        self.estop_total = False
        self.estop_joy = False
        self.estop_ls2 = False
        self.can_movetop = True
        self.drv_srv = False
        self.laser = False
        self.reset = False
        self.buzzer_stop = False
        self.is_fastech_on = False
        self.manual_servo_flag = False
        self.mgresults = [[0,0,0,0,False],[0,0,0,0,False]]

        # 4. Supervisor Init
        self.supervisor = TopPlateSupervisor(self)

        # 5. Start background threads.
        self.threads = []

        # (1) ROS2 thread
        if is_ros_if: 
            t = threading.Thread(target=self.agv_planning_main, daemon=True)
            t.start()
            self.threads.append(t)

        # (2) Joystick / GUI thread
        if is_windows: 
            t = threading.Thread(target=self.pygame_gui, daemon=True)
            t.start()
            self.threads.append(t)
        else: 
            t = threading.Thread(target=self.joystick, daemon=True)
            t.start()
            self.threads.append(t)

        # (3) Servo control threads
        if self.is_real_robot:
            for servo in self.servos:
                # Pass the servo instance through args.
                t = threading.Thread(target=self.control_servo, args=(servo,), daemon=True)
                t.start()
                self.threads.append(t)
     
        # (4) AGV control thread
        t = threading.Thread(target=self.control_agv, daemon=True)
        t.start()
        self.threads.append(t)

        # (5) Supervisor thread
        t = threading.Thread(target=self.supervisor.run_loop, daemon=True)
        t.start()
        self.threads.append(t)                                                                             

        print("All Threads Started Successfully")
    

    def agv_planning_main(self):
        print("\n[ROS2 THREAD] Starting rclpy initialization...")
        rif.rclpy.init(args=None)
        self.ros = rif.SrvreqSubscriber(self.agv, self.servo_states)
        self.thereadcheck[0] = 1
        print("[ROS2 THREAD] Successfully spinning SrvreqSubscriber node! Subscribed to /cam0 and /cam1")
        try:
            rif.rclpy.spin(self.ros)
        finally:
            self.ros.destroy_node()
            rif.rclpy.shutdown()

    def control_servo(self, servo):
        servo._state_init()
        servo._state_get(self.t_pos_fastech[servo.id], self.t_vel[servo.id], self.estop_total, self.can_movetop, self.t_action_fastech[servo.id])
        self.t_pos_fastech[servo.id] = servo.pos
        
        while not self.process_command:
            if self.is_real_robot:
                servo._state_get(self.t_pos_fastech[servo.id], self.t_vel[servo.id], self.estop_total, self.can_movetop, self.t_action_fastech[servo.id])
                servo._state_check()
                servo._state_set()
                self.c_pos[servo.id] = servo.pos
                self.thereadcheck[servo.id+1] = servo.thread_check_fs
            else:
                if self.t_action_fastech[servo.id] == 2: servo.state = 1
                elif self.t_action_fastech[servo.id] == 3: servo.state = 0
                elif self.t_action_fastech[servo.id] == 0: servo.pos = self.t_pos[servo.id]
                elif self.t_action_fastech[servo.id] == 1: servo.is_originfinding_ok = True
            time.sleep(self.config.loop_sleep_servo)
        servo.servo_off()

    def control_agv(self):
        running = True
        temp_time = time.time()
        while running:
            self.agv._state_get([self.input_manager.raw_axes, self.input_manager.raw_buttons, self.input_manager.raw_hat], 
                                self.is_fastech_on, self.mgresults, self.mg_pos, 0)
            
            self.agv.process_command = self.process_command
            self.agv.was_home_top = self.was_home
            self.agv.t_action_monitoring = self.t_action_fastech
            self.agv.thread_check_all = self.thereadcheck
            
            inputs = self.input_manager.get_state()
            if (inputs['L_JOY_BTN'] and inputs['R_JOY_BTN']) or self.agv.will_terminate:
                running = False
                self.process_command = 1
                break
            
            if self.agv.estop_fin_command_2server:
                self.reset = True
                self.estop_joy = False
                self.agv.estop_fin_command_2server = False
            self.estop_total = self.estop_joy or self.estop_ls2 or self.agv.estop_server2 != 0
            self.estop_2ls = self.estop_joy or self.agv.estop_server2
            self.agv.estop = self.estop_total
            
            if self.buzzer_stop: self.buzzer_stop = False
            if self.reset: self.reset = False

            temp_time2 = time.time()
            dt_time = temp_time2 - temp_time
            temp_time = temp_time2
            if dt_time < 0.01:
                time.sleep(0.16 - dt_time)

    def joystick(self):
            multi_command_flag = True

            # 1. Connect the controller.
            if not is_windows:
                from controller import evdev_controlthread as evct
                controller = evct.controller()
                controller.version = 2
                try:
                    controller.controller_connect()
                    print(">>> Controller Connected via evdev")
                except Exception as e:
                    print(f">>> Controller Connect Fail: {e}")
                    return  # Stop this thread on connection failure.

            # 2. Input polling loop
            while not self.process_command:
                if not is_windows:
                    try:
                        while multi_command_flag:
                            multi_command_flag = controller.joystick_get_data()
                        
                        # Update the shared InputManager instead of storing
                        # controller state on the robot object directly.
                        self.input_manager.update(controller.axes, controller.buttons, controller.hat)
                        
                        multi_command_flag = True
                    except Exception as e:
                        print(f"Joystick Read Error: {e}")
                        time.sleep(1)

                time.sleep(0.05)
            
    def pygame_gui(self):
        viz = joy.joy_gui()
        viz.is_joy()
        while True:
            axes, buttons, hat = viz.get_joy()
            self.input_manager.update(axes, buttons, hat)
            viz.screen_set([axes[0], axes[1]])
            viz.check_event()
            if self.process_command:
                viz.quit_event()
                break
            time.sleep(0.15)
        joy.pygame.quit()
        print("pygame quit")

if __name__ == "__main__":
    my_config = RobotConfig()
    controller = robot(my_config)
    
    try:
        # Wait until shutdown is requested.
        while not controller.process_command:
            time.sleep(1)
    except KeyboardInterrupt:
        # Forward a clean shutdown signal on Ctrl+C.
        controller.process_command = 1
        print("\nStopping robot...")
    if is_ros_if:
        # Confirm that the ROS node was actually created.
        if hasattr(controller, 'ros'):
            controller.ros.destroy_node()
        
        # Shut down rclpy if needed.
        try:
            rif.rclpy.shutdown()
        except Exception:
            pass  # Already shut down.

    print("Program was down successfully")
