import pygame

class joy_gui:
    def __init__(self):
        self.screen_width, self.screen_height = 550, 550
        pygame.init()
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("AGV controller Visualizer")
        self.axes=[0,0,0,0,0,0]
        self.buttons=[0,0,0,0,0,0,0,0,0,0,0]
        self.hat=[0,0]

    def is_joy(self):
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("No joystick detected")
            self.joy_flag=False
            return False
        else:
            print("Joystick detected and 1st joystick is selected")
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.joy_flag=True
            return True
    def check_event(self):
        is_running=True
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                is_running = False
        return is_running
    
    def screen_set(self,v1):
        axes=self.axes
        buttons=self.buttons
        hat=self.hat
        self.screen.fill((0, 0, 0))
        for i in range(2):
            pygame.draw.line(self.screen, (255, 255, 255*i), (150+250*i,150+250*i), (int(axes[i*2] * 100+150+250*i),int(axes[i*2+1] * 100+150+250*i)),width=5)
        for i in range(2):
            pygame.draw.rect(self.screen, (50, 50, 50), (20+300*i, 10, 200, 10))
            pygame.draw.rect(self.screen, (255, 255, 255), (20+300*i, 10, int(axes[i+4] * 100+100), 10))
            pygame.draw.rect(self.screen, (50, 50+200*buttons[4+i], 50), (20+300*i, 30, 200, 10))
        if self.joy_flag:
            pygame.draw.rect(self.screen, (255, 30, 30), (250, 250, 25, 25))
            
        pygame.draw.rect(self.screen, (0, 100+155*buttons[0], 0), (380, 150, 10, 10))
        pygame.draw.rect(self.screen, (100+155*buttons[1], 0, 0), (410, 120, 10, 10))
        pygame.draw.rect(self.screen, (0, 0, 100+155*buttons[2]), (350, 120, 10, 10))
        pygame.draw.rect(self.screen, (100+155*buttons[3], 100+155*buttons[3], 0), (380, 90, 10, 10))    
        pygame.draw.line(self.screen, (255, 255, 255), (150, 400), (100*hat[0]+150, -100*hat[1]+400),width=10)
        
        pygame.draw.rect(self.screen, (0, 100+155*buttons[6], 0), (240, 150, 10, 10))
        pygame.draw.rect(self.screen, (0, 100+155*buttons[7], 0), (300, 150, 10, 10))
        pygame.draw.rect(self.screen, (50+155*buttons[8], 50+155*buttons[8], 50+155*buttons[8]), (270, 100, 10, 10))
        pygame.draw.rect(self.screen, (50+100*buttons[9], 50+100*buttons[9], 50+100*buttons[9]), (145, 145, 10, 10))
        pygame.draw.rect(self.screen, (100+155*buttons[10], 100+155*buttons[10], 100+155*buttons[10]), (395, 395, 10, 10))

        pygame.draw.rect(self.screen, (80, 80, 80), (0, 0, 20, 500))
        pygame.draw.rect(self.screen, (80, 80, 80), (530, 0, 20, 500))
        pygame.draw.line(self.screen, (255, 255, 255), (10, 250), (10, 250+int(v1[0]/10)),width=20)
        pygame.draw.line(self.screen, (255, 255, 255), (540, 250), (540, 250+int(v1[1]/10)),width=20)
        # Update the display
        pygame.display.flip()
    
    def get_joy(self):
        if self.joy_flag:
            self.axes = [self.joystick.get_axis(i) for i in range(4)]
            self.axes.append((self.joystick.get_axis(4)+1.0)*0.5)
            self.axes.append((self.joystick.get_axis(5)+1.0)*0.5)
            self.buttons = [self.joystick.get_button(i) for i in range(8)]
            self.buttons.append(self.joystick.get_button(10))
            self.buttons.append(self.joystick.get_button(8))
            self.buttons.append(self.joystick.get_button(9))
            temphat=self.joystick.get_hat(0)
            self.hat=[temphat[0],-temphat[1]]
        else: 
            self.axes=[0,0,0,0,0,0]
            self.buttons=[0,0,0,0,0,0,0,0,0,0,0]
            self.hat=[0,0]
        #print(self.axes,self.buttons,self.hat)
        
        return self.axes,self.buttons,self.hat
    
    def quit_event(self):
        pygame.event.post(pygame.event.Event(pygame.QUIT))
        
        


if __name__ == "__main__":
    import time
    viz=joy_gui()
    # initialize joystick if it is connected
    is_joy=viz.is_joy()
    
    running = True
    i=1
    while running:
        if is_joy:
            axes,buttons,hat=viz.get_joy()
        else:
            axes=[0,0,0,0,0,0]
            buttons=[0,0,0,0,0,0,0,0,0,0,0]
            hat=[0,0]
        if not is_joy:
            i=i+1
            if i==100:
                i=0
            axes=[i/100.0,-i/100.0,(100-2*i)/100.0,(-100+2*i)/100.0,0,0]    
        viz.screen_set([axes[0],axes[1]])
        if buttons[0] and buttons[1]: 
            viz.quit_event()
        
        running = viz.check_event()
        #time.sleep(0.05)