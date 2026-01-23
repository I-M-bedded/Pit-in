import time
from controller import joy

def joystick(self):
    multi_command_flag = True

    # Try to connect to the controller once
    if not self.is_windows: # [수정] self.is_windows 사용
        from controller import evdev_controlthread as evct
        # initialize the top plate controller
        controller = evct.controller()
        controller.version=2
        controller.controller_connect()

    # While the process is not finished, keep getting the data from the controller
    while not self.process_command:

        if not self.is_windows:
            #in Windows, the controller is connected in the pygame_gui function
            #in Linux, the controller is connected here
            while multi_command_flag:
                multi_command_flag = controller.joystick_get_data()
            self.axes=controller.axes
            self.buttons=controller.buttons
            self.hat=controller.hat
            multi_command_flag = True
        time.sleep(0.05)

def pygame_gui(self):
    # initialize joystick if it is connected
    viz=joy.joy_gui()
    
    viz.is_joy()
    
    while True:
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
        
    joy.pygame.quit()   
    print("pygame quit")