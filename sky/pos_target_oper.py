from numpy import *
from scipy import *
import math

class pos_target_oper:
    """top plate inverse kinematics class
    """
    def __init__(self,version,reacq_rate):
        if version==1:
            self.d2= [0.024, 0.57763]
            self.d3=[0.0, 0.15859]
            
        elif version==3:
            self.d2=[0.024, 0.54797] 
            self.d3=[0.0, 0.19299] 
            
        elif version==4: #CORA
            self.d2=[0.024, 0.5695] 
            self.d3=[0.0, 0.163] 

        else:
            self.d2=[0.024, 0.58446] 
            self.d3=[0.0, 0.14747]

        self.d20=self.d2[0]*100; self.d21=self.d2[1]*100; self.d31=self.d3[1]*100;
        self.error=0; self.reacq_rate=reacq_rate;
        self.nu=84.2858;
        self.m2cnt = array([2000000.0, 2000000.0, 5000.0*self.nu/pi, 11250.0/pi, 11250.0/pi, 2000000.0, 2000000.0])
        self.target_mode = False   # control mode flag, once pressed
        self.mode = [10,11,12,13,14,15,16,17]   # constant
        self.target_q_data = [0]*7 # [50000, 50000, 4024, 1790, 1790,0,0] 
        self.target_tol_error = [100]*7   # tolerance error to stop
        self.X0=[0,0]; self.X1=[0,0]; self.zloc=[0,0];
        

    def Effectivity(self,Lx, Ly, Rx, Ry, d20, d21, d31, q):
        # Pin distance Effectivity
        dist = math.sqrt((Ry - Ly)**2 + (Rx - Lx)**2)
        if dist > 2*(d21+d31):
            q = [0,0,0,0,0]
            self.error=1;
            raise Exception("Impossible : too far distance")
        elif dist < 2*(d21-d31):
            q = [0,0,0,0,0]
            self.error=2;
            raise Exception("Impossible : too short distance")
        # Motor range limitation
        # elif (q[0]<-0.088*100) or (q[0]>0.0538*100) or (q[1]<-0.063*100) or (q[1]>0.057*100) or (q[2]<-0.32) or (q[2]>0.065) or (q[3]<-1.7) or (q[3]>1.025) or (q[4]<-1.7) or (q[4]>1.318):
        #     q = [0,0,0,0,0]
        #     self.error=3
        #     raise Exception("Range limitation")

        return q
    

    def Error_evaluate(self, X0, X1, Lx, Ly, Rx, Ry):  #cm input, m unit distance error return
        pin_error = [sqrt((X0[0]-Lx)**2+(X0[1]-Ly)**2)/100,sqrt((X1[0]-Rx)**2+(X1[1]-Ry)**2)/100 ]

        return pin_error
    
    def compute_FK(self, q):   #cnt input ,  cm unit location return
        q=q/self.m2cnt
        q[0:2]=q[0:2]*100
        self.X0=[-q[0]+self.d21*cos(q[2])-self.d20*sin(q[2])+self.d31*cos(q[2]+q[3]),-q[1]+self.d21*sin(q[2])+self.d20*cos(q[2])+self.d31*sin(q[2]+q[3])]
        self.X1=[-q[0]-self.d21*cos(q[2])-self.d20*sin(q[2])-self.d31*cos(q[2]+q[4]),-q[1]-self.d21*sin(q[2])+self.d20*cos(q[2])-self.d31*sin(q[2]+q[4])]

        return self.X0, self.X1
    
    def compute_FK_z(self, q):
        self.zloc=q[5:7]
        self.zloc=[self.zloc[0]/self.m2cnt[5]*100,self.zloc[1]/self.m2cnt[6]*100]
        return self.zloc

        
    def compute_IK(self, Lx, Ly, Rx, Ry): #cm단위로, Lx,Ly,Rx,Ry입력
        q2 = math.atan2((Ly - Ry),(Lx - Rx))
        q3 = abs(math.acos((Lx - Rx - 2*self.d21*math.cos(q2)) / (2*self.d31*math.cos(q2))))
        rev = Ry-(Ry-Ly)/(Rx-Lx)*Rx
        if (rev < self.d20*(((Ry-Ly)/(Rx-Lx))**2+1)):
            q3 = -abs(q3)

        q0 = - Lx + self.d21 * math.cos(q2) - self.d20 * math.sin(q2) + self.d31 * math.cos(q2 + q3)
        q1 = - Ly + self.d21 * math.sin(q2) + self.d20 * math.cos(q2) + self.d31 * math.sin(q2 + q3)

        qm = [q0,q1,q2,q3,-q3]
        qm = self.Effectivity(Lx, Ly, Rx, Ry, self.d20, self.d21, self.d31, qm)
        qm = pad(qm, (0, 2), 'constant')
        qm[0:2]=qm[0:2]/100
        self.target_q_data = qm*self.m2cnt

        self.target_q_data = self.target_q_data*(self.reacq_rate/100)  #정지 후 재수집 지점으로 지정
        
        self.target_q_data = self.target_q_data .astype(int)  #소수점 버림
        self.target_q_data = self.target_q_data .tolist()

        return self.target_q_data

