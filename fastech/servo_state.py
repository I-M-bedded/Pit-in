class ServoState:
    def __init__(self):
        self.topplate_cpos = [0,0,0,0,0,0,0]
        self.topplate_tpos = [0,0,0,0,0,0,0]

    def update_topplate(self,cpos,tpos):
        for i in range(len(self.topplate_cpos)):
            self.topplate_cpos[i] = cpos[i]
        for i in range(len(self.topplate_tpos)):
            self.topplate_tpos[i] = tpos[i]