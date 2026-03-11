import time
import evdev
from evdev import InputDevice, categorize, ecodes
import sys, os, platform
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import subprocess

controller_address_1 = '28:EA:0B:E8:48:B8'
controller_address_2 = '40:8E:2C:15:51:40'  #'28:EA:0B:CD:7B:A8'   # please set dis address to your controller's mac address
controller_address_3 = '40:8E:2C:43:3E:F7'
class controller: 
    def __init__(self):
        self.plat_name = platform.system()
        self.version = -1
        self.controller = None
        self.connection_type = None
        self.connect_times = 0
        self.version = -1
        self.mac_address = None

        self.buttons = [0,0,0,0,0,0,0,  0,    0,   0,        0]
                     # [A,B,X,Y,L,R,SEL,START,LOGO,L_JOY(s1),R_JOY(s2)]
        self.axes = [0,0,  0,0,  0,0]
                  # [laxes,raxes,triggers]
        self.laxes = [0,0]
        self.raxes = [0,0]
        self.triggers = [0,0]
        self.hat = [0,0]

        self.keyboard=[]

        self.robot_command=[0,0,0,0,0,0,0]
        

        self.left = 0
        self.right = 0
        
        self.vel = 0
        self.ang_vel = 0

        self.ray_viz = False

        self.running = True

    def controller_connect(self):
        devices = None
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if 'Xbox One Wireless Controller' in device.name or 'Microsoft Xbox Controller' in device.name or 'Xbox Wireless Controller'in device.name:
                if device.uniq == '':
                    self.connection_type = 'wireless'
                else:
                    self.connection_type = 'bluetooth'
                    self.mac_address=device.uniq
                path = device.path
                print(device.path, device.name, device.phys, device.uniq)

                gamepad = InputDevice(path)
                self.joy_init()
                connection = 1
                print(gamepad)
                self.controller = gamepad

            elif 'keyboard' in device.name or 'Keyboard' in device.name or device.name == 'Wings Chip Wireless Dongle':
                if connection == 1:
                    break
                path = device.path
                # path = '/dev/input/event3'
                print(device.path, device.name, device.phys)
                keyboard = InputDevice(path)
                self.keyboard_init()
                print(keyboard)
                self.controller = keyboard
                # break
            
        if self.controller == None:
            if self.mac_address==None:
                if self.version == 1:
                    self.mac_address = controller_address_1
                elif self.version == 2:
                    self.mac_address = controller_address_2
                elif self.version == 3:
                    self.mac_address = controller_address_3
            self.connect_times += 1
            # if self.connect_times%10 == 0:
            #     print('controller not connected! Please wait...')
            if self.connect_times == 100:
                try:
                    self.check_paired(self.mac_address)
                    self.connect_times = 0
                except Exception as e:
                    print(e)
                    self.connect_times = 0
            time.sleep(0.01)
            
    
    def check_paired(self,device_mac):
        # bluetoothctl을 사용하여 장치 정보를 가져옵니다.
        result = subprocess.run(['bluetoothctl', 'info', device_mac], capture_output=True, text=True)
        print(result)

        # # 결과 출력
        if 'not available' or 'Paired: no' in result.stdout:
            try:
                # print(f"장치 {device_mac}는 현재 컴퓨터에 페어링되어 있지 않습니다.")
                recon = subprocess.run(['bluetoothctl', 'pair', device_mac],timeout=30, capture_output=True, text=True)
                # print("stdout:", recon.stdout)
                # print("stderr:", recon.stderr)
                return True
            except Exception as e:
                print(e)

        elif 'Connected: no' in result.stdout:
            try:
                # print(f"장치 {device_mac}는 현재 컴퓨터에 연결되어 있지 않습니다.")
                recon = subprocess.run(['bluetoothctl', 'connect', device_mac],timeout=30, capture_output=True, text=True)
                # print("stdout:", recon.stdout)
                # print("stderr:", type(recon.stderr))
                return True
            except Exception as e:
                print(e)

        # else:
        #     print(f"장치 {device_mac}는 현재 컴퓨터에 연결되어 있습니다.")
        #     return True
        
        try:
            os.system('sudo /etc/init.d/bluetooth restart')
        except Exception as e:
                print(e)

    def joy_init(self):
        # Set up the joystick
        # joystick var
        self.aBtn = 304
        self.bBtn = 305
        self.xBtn = 307
        self.yBtn = 308
        self.lBtn = 310
        self.rBtn = 311
        self.selBtn = 314
        self.staBtn = 315
        self.logoBtn = 316
        self.ljoyBtn = 317
        self.rjoyBtn = 318
        if self.connection_type == 'wireless':
            self.R_Joy_X = "ABS_RX"
            self.R_Joy_Y = "ABS_RY"
            self.L_tri = "ABS_Z"
            self.R_tri = "ABS_RZ"
            self.adjustment = 0
        elif self.connection_type == 'bluetooth': 
            self.R_Joy_X = "ABS_Z"
            self.R_Joy_Y = "ABS_RZ"
            self.L_tri = "ABS_BRAKE"
            self.R_tri = "ABS_GAS"
            self.adjustment = 32768

    def joystick_get_data(self):
        # self.joy_init()
        #print("*** get Joystick start ***")
        event = None
        try:
            if self.controller != None:
                event = self.controller.read_one()
            else:
                self.controller_connect()
        except OSError as e:
            print('controller is disconnected! Please wait...? / ',e)
            self.controller = None
            self.controller_connect()
        if event!=None:
            if self.buttons[9] and self.buttons[10]:
                pass
            if event.type == ecodes.EV_KEY:
                if event.code == self.aBtn:
                    self.buttons[0] = event.value
                    # print("A Button Pressed.")
                elif event.code == self.bBtn:
                    self.buttons[1] = event.value
                    # print("B Button Pressed.")
                elif event.code == self.xBtn:
                    self.buttons[2] = event.value
                    # print("X Button Pressed.")
                elif event.code == self.yBtn:
                    self.buttons[3] = event.value
                    # print("Y Button Pressed.")
                elif event.code == self.lBtn:
                    self.buttons[4] = event.value
                    # print("L Button Pressed.")
                elif event.code == self.rBtn:
                    self.buttons[5] = event.value
                    # print("R Button Pressed.")
                elif event.code == self.selBtn:
                    self.buttons[6] = event.value
                    # print("Select Button Pressed.")
                elif event.code == self.staBtn:
                    self.buttons[7] = event.value
                    # print("Start Button Pressed.")
                elif event.code == self.logoBtn:
                    self.buttons[8] = event.value
                    # print("Xbox Button Pressed.")
                elif event.code == self.ljoyBtn:
                    self.buttons[9] = event.value
                    # print("LJoy Button Pressed.")
                elif event.code == self.rjoyBtn:
                    self.buttons[10] = event.value
                    # print("RJoy Button Pressed.")
                # print(self.buttons)

            elif event.type == ecodes.EV_ABS:
                absevent = categorize(event)
                #print(ecodes.bytype[absevent.event.type][absevent.event.code], absevent.event.value)

                # Left_joystick
                if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_X":
                    self.laxes[0] = (absevent.event.value-self.adjustment)/32767
                    self.axes[0] = self.laxes[0]
                elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Y":
                    self.laxes[1] = (absevent.event.value-self.adjustment)/32767
                    self.axes[1] = self.laxes[1]
                # Right_Joystick
                elif ecodes.bytype[absevent.event.type][absevent.event.code] == self.R_Joy_X:
                    self.raxes[0] = (absevent.event.value-self.adjustment)/32767
                    self.axes[2] = self.raxes[0]
                elif ecodes.bytype[absevent.event.type][absevent.event.code] == self.R_Joy_Y:
                    self.raxes[1] = (absevent.event.value-self.adjustment)/32767
                    self.axes[3] = self.raxes[1]

                # L_Trigger
                elif ecodes.bytype[absevent.event.type][absevent.event.code] == self.L_tri:
                    self.triggers[0] = absevent.event.value/1023
                    self.axes[4] = self.triggers[0]
                # R_Trigger
                elif ecodes.bytype[absevent.event.type][absevent.event.code] == self.R_tri:
                    self.triggers[1] = absevent.event.value/1023
                    self.axes[5] = self.triggers[1]
                
                # Hat
                elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_HAT0X":
                    # print(absevent.event.value)
                    self.hat[0] = absevent.event.value
                elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_HAT0Y":
                    self.hat[1] = absevent.event.value  
        else:
            return 0
        return 1
        #print("*** get Joystick Off ***")

    def joystick_check(self):
        print("*** Joystick check start ***")
        while True:
            if self.buttons[9] and self.buttons[10]:
                break
            print('buttons : ',self.buttons)
            print('axes : ',self.axes)
            print('hat : ',self.hat)
            time.sleep(0.5)
        print("*** Joystick check Off ***")

if __name__ == "__main__":
    import threading
    con = controller()
    con.controller_connect()
    con_name = con.controller.name
    threads=[]
    def while_get_joy():
        while True:
            if con.buttons[9] and con.buttons[10]:
                break
            # print(time.time())
            con.joystick_get_data()
    if con_name == 'Xbox Wireless Controller' or 'Microsoft Xbox Controller':
        print("**************************")
        print("*   Joystick Connected   *")
        print("**************************")
        thread = threading.Thread(target=while_get_joy,args=())
        thread.start()
        threads.append(thread)
        thread = threading.Thread(target=con.joystick_check,args=())
        thread.start()
        threads.append(thread)
        for thread in threads:
            thread.join()
