import numpy as np

niro_gap=1.386
i5_gap=1.407#1.407  #1.403 1hogi cnt2334
e6_gap=1.407
e3_gap=1.4
bongo_gap=1.404
reserved_gap=1.4

gap=[niro_gap,i5_gap,e6_gap,bongo_gap,e3_gap,reserved_gap]

class Topik:
    """top plate inverse kinematics class
    """
    def __init__(self,version):
        self.nu=59
        if version == 1:
            self.nu=84.2858
        self.q = np.array([0, 0, 0, 0, 0, 0, 0])
        self.qm = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.qm2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.m2cnt = np.array([2000000.0, 2000000.0, 5000.0*self.nu/np.pi, 11250.0/np.pi, 11250.0/np.pi, 2000000.0, 2000000.0])
        self.cnt2m = np.array([0.0000005, 0.0000005, 0.0002/self.nu*np.pi, 0.000088889*np.pi, 0.000088889*np.pi, 0.0000005, 0.0000005])
        self.xc= np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        self.x = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        

        self.xd = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        self.so3_rcam = np.matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]) #identity
        self.so3_lcam = np.matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.so3_tp = np.matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        
        self.x0=0.0
        self.y0=0.0
        self.rcam_err=np.array([0.0, 0.0, 0.0])
        self.lcam_err=np.array([0.0, 0.0, 0.0])
        # robot version dependent kinmatics parameter 
        if version==1:
            self.d2= [0.024, 0.57763]
            self.d3=[0.0, 0.15859]
            
        elif version==3:
            self.d2=[0.024, 0.54797] 
            self.d3=[0.0, 0.19299]

        elif version==4: #CORA, measured from sky
            self.d2=[0.024, 0.5695] 
            self.d3=[0.0, 0.163]

        else:
            self.d2=[0.024, 0.58446] 
            self.d3=[0.0, 0.14747]
        
        # maximum and minimum distance between pin
        self.dismax=2.0*(self.d2[1]+self.d3[1])-0.01
        self.dismin=2.0*self.d2[1]+0.01
        
        self.qd=[0,0,0,0,0,0,0]
        self.qdm=np.zeros(7)
        self.p2p=0.0
        self.cartype=0 # niro  0, ioniq5 1, ev3 2, bongo 3
        self.xo_arr=[]

        for i in range(len(gap)):
            q3=self.cwa(gap[i])
            if i==3:
                self.get_q([0,0,0,q3,-q3,0,0])
            else:   
                self.get_q([0,0,0,-q3,q3,0,0])
            self.fk()
            self.xo_arr.append(np.array(self.x))

        if version==1:
            self.x0=0.0#-0.008
            self.y0=-0.0078
        elif version==3:
            self.x0=0.0
            self.y0=-0.0006
        else: 
            pass

        
    def get_q(self, q):
        """update motor position to topik class

        Args:
           q(array(7,)): motor position array in cnt. ex) [0,0,0,0,0,0,0]
        """
        self.q = q
        self.qm = self.q*self.cnt2m
        self.qm2[0]=-self.qm[0]
        self.qm2[1]=-self.qm[1]
        self.qm2[2]=-self.qm[2]
        self.qm2[3]=-self.qm[3]
        self.qm2[4]=-self.qm[4]
        self.qm2[5]=self.qm[5]
        self.qm2[6]=self.qm[6]


    def fk(self):
        """calculate forward kinematics with given motor position 
            xc:return and update wing center posion in meter at robot base coordinate
            x: pin position in meter at robot base coordinate
            p2p: pin to pin distance
            so3_tp: rotation matrix of top plate
            so3_lcam: rotation matrix of left camera
            so3_rcam: rotation matrix of right camera

        Returns
            xc(array(2,3)): wing position in meter and robot base coordinate
        """
        # left wing center
        self.xc[0][0] = -self.qm[1]+np.sin(self.qm[2])*self.d2[1]+np.cos(self.qm[2])*self.d2[0]+self.x0 #robot cord shift + plate shift + relative dist (L-shape)
        self.xc[0][1] = -self.qm[0]+np.cos(self.qm[2])*self.d2[1]-np.sin(self.qm[2])*self.d2[0]+self.y0 #x
        self.xc[0][2] = self.qm[5]
        # right wing center
        self.xc[1][0] = -self.qm[1]-np.sin(self.qm[2])*self.d2[1]+np.cos(self.qm[2])*self.d2[0]+self.x0 #y
        self.xc[1][1] = -self.qm[0]-np.cos(self.qm[2])*self.d2[1]-np.sin(self.qm[2])*self.d2[0]+self.y0 #x
        self.xc[1][2] = self.qm[6]

        # left pin 
        self.x[0][0] = self.xc[0][0]+np.sin(self.qm[2]+self.qm[3])*self.d3[1]
        self.x[0][1] = self.xc[0][1]+np.cos(self.qm[2]+self.qm[3])*self.d3[1]
        self.x[0][2] = self.xc[0][2]    
        # right pin
        self.x[1][0] = self.xc[1][0]-np.sin(self.qm[2]+self.qm[4])*self.d3[1]
        self.x[1][1] = self.xc[1][1]-np.cos(self.qm[2]+self.qm[4])*self.d3[1]
        self.x[1][2] = self.xc[1][2]

        self.p2p=np.sqrt((self.x[0][0]-self.x[1][0])**2+(self.x[0][1]-self.x[1][1])**2)

        self.so3_tp=self.rot_Z(self.qm2[2])
        self.so3_lcam=self.rot_Z(self.qm2[2]+self.qm2[3])
        self.so3_rcam=self.rot_Z(self.qm2[2]+self.qm2[4])
        
        return self.xc
    
    def get_J(self):
        """calculate jacobian matrix with given motor position
        J: Jacobian matrix of Lx, Ly, Rx, Ry
        """
        self.J=np.array([[0.0, -1.0, np.cos(selgapf.qm[2])*self.d2[1]-np.sin(self.qm[2])*self.d2[0]+np.cos(self.qm[2]+self.qm[3])*self.d3[1], np.cos(self.qm[2]+self.qm[3])*self.d3[1], 0.0, 0.0,0.0],
                        [-1.0, 0.0, -np.sin(self.qm[2])*self.d2[1]-np.cos(self.qm[2])*self.d2[0]-np.sin(self.qm[2]+self.qm[3])*self.d3[1], -np.sin(self.qm[2]+self.qm[3])*self.d3[1], 0.0, 0.0, 0.0],  
                        [0.0, -1.0, -np.cos(self.qm[2])*self.d2[1]-np.sin(self.qm[2])*self.d2[0]-np.cos(self.qm[2]+self.qm[4])*self.d3[1], 0.0, -np.cos(self.qm[2]+self.qm[4])*self.d3[1], 0.0,0.0], 
                        [-1.0, 0.0, np.sin(self.qm[2])*self.d2[1]-np.cos(self.qm[2])*self.d2[0]+np.sin(self.qm[2]+self.qm[4])*self.d3[1], 0.0, np.sin(self.qm[2]+self.qm[4])*self.d3[1], 0.0,0.0]])

    def cal_wing_angle(self, dis):
        """ calculate wing angle with given pin gap for symmetric condition

        Args:
            dis (float): pin gap distance

        Returns:
            float: wing angle in radian
        """
        if dis>self.dismax:
            dis=self.dismax
        elif dis<self.dismin:
            dis=self.dismin
        
        ly=dis*0.5
        ly_w=ly-self.d2[1]
        cq3=ly_w/self.d3[1]
        q3=np.arccos(cq3)
        return q3 
    
    def cwa(self,dis):
        """calculte wing angle with given pin gap and return in cnt

        Args:
            dis (float): pin gap distance

        Returns:
            int: wing angle in cnt
        """
        return int(np.round(self.cal_wing_angle(dis)*self.m2cnt[3],0))

    def cal_gap(self):
        """calculate pin gap distance from xd

        Returns:
            float : desired pin gap distance in meter
        """
        pingap_v=self.xd[0]-self.xd[1]
        print("pingap_v : ",pingap_v) #sky#sky#sky#sky#sky
        return np.sqrt(pingap_v[0]**2+pingap_v[1]**2)

    def num_ik(self):
        """calculate ik solution with given desired pin gap from xd. 
        qd: desired motor position in cnt
        qdm: desired motor position in meter and radian
        """
        # initialize temporary variable
        qd=np.zeros(7)
        # calculate wing angle with given pin gap from xd and guess the symetric wing angle.
        print("xd : ",self.xd) #sky#sky#sky#sky#sky
        print("dis : ",self.cal_gap())  #sky#sky#sky#sky#sky
        q3=self.cal_wing_angle(self.cal_gap())
        # because of multiple solution, choose the certain solution with cartype
        if self.cartype==3: 
            self.qm[3]=q3
            self.qm[4]=-q3
        else:
            self.qm[3]=-q3
            self.qm[4]=q3
        # calculate q1 and q2 with given desired pin position annd q3 and q4
        sq2=(self.xd[0][0]-self.xd[1][0])/(2*self.d2[1]+2*self.d3[1]*np.cos(self.qm[3]))
        cq2=(self.xd[0][1]-self.xd[1][1])/(2*self.d2[1]+2*self.d3[1]*np.cos(self.qm[3]))
        qd[0]=-self.xd[0][1]-sq2*(self.d2[0]+self.d3[1]*np.sin(self.qm[3]))+cq2*(self.d2[1]+self.d3[1]*np.cos(self.qm[3])) + self.y0
        qd[1]=-self.xd[0][0]+sq2*(self.d2[1]+self.d3[1]*np.cos(self.qm[3]))+cq2*(self.d2[0]+self.d3[1]*np.sin(self.qm[3])) + self.x0
        qd[2]=np.arctan2(sq2,cq2)
        qd[3]=self.qm[3]
        qd[4]=self.qm[4]
        qd[5]=self.xd[0][2]
        qd[6]=self.xd[1][2]
        # update motor position in meter and radian
        self.qdm=qd
        # update motor position in cnt
        for i in range(7):
            self.qd[i]=int(self.qdm[i]*self.m2cnt[i])
        pass
    
    def get_wing_q(self, dis):
        """get wing motor position with given pin gap and cartype

        Args:
            dis (float): pin gap distance

        Returns:
            q(array(7,)): return motor position in cnt for wing
        """
        q3=self.cal_wing_angle(dis)
        q3i=int(q3*self.m2cnt[3])
        if self.cartype==3:
            return [0, 0,0, q3i, -q3i, 0, 0]
        else:
            return [0, 0,0, -q3i, q3i, 0, 0]
    
    def set_xd(self, cartype,xoff,pinh=0):
        """set desired pin position with given cartype and x offset

        Args:
            cartype (int): car type 0 niro, 1 ioniq5, 2 ev3, 3 bongo
            xoff (array(2,3)): x offset in meter [[x,y,z],[x,y,z]]
            pinh (int, optional): pin height in cnt Defaults to 0.

        Returns:
            qd(array(7,)): desired motor position in cnt
        """
        self.cartype=cartype
        #xoffarr=np.array([xoff, xoff])
        xoffarr=xoff  #sky
        pinoff = np.array([[0,0,pinh*self.cnt2m[5]],[0,0,pinh*self.cnt2m[6]]])
        self.xd= self.xo_arr[cartype] +xoffarr +pinoff
        self.num_ik()
        #self.so3_tp=self.rot_Z(self.qdm[2])
        #self.so3_lcam=self.rot_Z(self.qdm[2]+self.qdm[3])
        #self.so3_rcam=self.rot_Z(self.qdm[2]+self.qdm[4])
        return self.qd
    
    def rot_Z(self,angle):
        """rotate matrix in Z axis
        Args:
            angle (rad, float): radian angle

        Returns:
            3X3 matrix: rotation matrix
        """
        return np.matrix([[np.cos(angle), -np.sin(angle), 0.0], [np.sin(angle), np.cos(angle), 0.0], [0.0, 0.0, 1.0]])

    def rcam2robot(self, target):
        so3bs=np.matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        return np.dot(so3bs, target)
    def lcam2robot(self, target):
        so3bs=np.matrix([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
        return np.dot(so3bs, target)
    
    def error2xd(self,cartype,q,lerr,rerr,type=0):
        """return motor position with camera error
        Args:
            cartype (int): 0 niro, 1 ioniq5, 2 ev3, 3 bongo
            q (array(7,)): Current motor position
            lerr (array(3,)): left camera error in meter
            rerr (array(3,)): right camera error in meter
            type (int, optional): Camera dependency: Indirect(camera->adjust(pingap))/Direct(camera info only) Side_selection: L/R/2(both)  0=I2, 1=IL, 2=IR, else=D2. Defaults to 0.

        Returns:
            q(array(7,)): desired motor position in cnt
        """
        self.cartype=cartype
        # get q from motor position and calculate forward kinematics
        self.get_q(q)
        self.fk()

        self.xd[0][0]=self.x[0][0] 
        self.xd[0][1]=self.x[0][1]
        self.xd[0][2]=self.x[0][2]
        self.xd[1][0]=self.x[1][0]
        self.xd[1][1]=self.x[1][1]
        self.xd[1][2]=self.x[1][2]

        # correct desired pin position with camera error
        # calculate error vector of left and right camera
        
        lcam_err=np.dot(self.so3_lcam, self.lcam2robot(lerr).transpose())     
        rcam_err=np.dot(self.so3_rcam, self.rcam2robot(rerr).transpose()) 
        # calculate angle of error vector
        ervec=(lcam_err-rcam_err)*0.5
        average=(lcam_err+rcam_err)*0.5
        # calculate angle of error vector
        r=self.xo_arr[cartype][0][1]
        y=ervec[0,0]
        angle=np.arctan2(-y,r)
        
        average=np.ravel(average)
        lcam_err=np.ravel(lcam_err)
        rcam_err=np.ravel(rcam_err)
        self.rcam_err=rcam_err
        self.lcam_err=lcam_err
        #print("angle in degree",angle*180/np.pi)
        
        if type==0:
            # Indirect method lcam + rcam and translation+rotation
            # calculate desired pin position with error vector
            # translate first
            self.xd[0]=self.xd[0]-average
            self.xd[1]=self.xd[1]-average
            # rotate second
            self.xd[0]=np.dot(self.rot_Z(angle),self.xd[0])
            self.xd[1]=np.array(np.dot(self.rot_Z(angle),self.xd[1]))
            #print(f"lcam_err : {lcam_err}   rcam_err : {rcam_err}")

        elif type==1:
            # Indirect method lcam + translation only
            self.xd[0]=self.xd[0]-lcam_err
            self.xd[1]=self.xd[1]-lcam_err
            # print(f"lcam_err : {lcam_err}")
        
        elif type==2:
            # Indirect method rcam + translation only
            #print(f"rcam_err : {rcam_err}")
            # self.xd[0]=self.xd[0]
            self.xd[1]=self.xd[1]-rcam_err

        else :
            # print(f"lcam_err : {lcam_err}   rcam_err : {rcam_err}")
            
            # direct method
            self.xd[0]=self.xd[0]-lcam_err
            self.xd[1]=self.xd[1]-rcam_err
        # calculate ik solution
        self.num_ik()

        # return angle and average of error vector
        return self.qd

if __name__=="__main__":
    # initialize class with robot version. no1 agv has different reduction ratio at Zrotation
    topik = Topik(1)
    # Easy and simple method
    ### 1. IK simple
    # example  topik.set_xd(cartype, homeoffset) will return desired motor position command of int array
    print("simpel ik for niro",topik.set_xd(0,[0.0,0.0,0.0]))
    print("simpel ik for ioniq5",topik.set_xd(1,[-0.0544,0.0068,0.0])) # -0.0544,0.0064,0.0
    print("simpel ik for bongo",topik.set_xd(3,[0.0,0.0,0.0]))
    # print("simpel ik for ev3",topik.set_xd(2,[0.0,0.0,0.0]))
    # print("simpel ik for ev3 with lateral offset",topik.set_xd(2,[0.0,0.05,0.0]))
    # print("simpel ik for ev3 with x offset",topik.set_xd(2,[0.05,0.0,0.0]))
    # print("simpel ik for ev3 with x offset",topik.set_xd(2,[0.05,0.0,0.0],100000))
    ### 2. FK simple
    # set current angle with motor outputs 
    topik.get_q([0,0,0,-2269,2269,0,0])
    #topik.get_q([0,0,0,-2597,2597,0,0])
    # q will converted to qm(meter, radian unit joint position)
    print("set q:", topik.q)
    print("converted qm:", topik.qm)
    # calcalate forward kinematics
    topik.fk()
    # lpin, rpin position array
    print("fk result",topik.x)    
    
    topik.get_q([0,0,0,-2580,2580,0,0])
    # q will converted to qm(meter, radian unit joint position)
    print("set q:", topik.q)
    print("converted qm:", topik.qm)
    # calcalate forward kinematics
    topik.fk()
    # lpin, rpin position array
    print("fk result",topik.x)    

    ### 3. IK example 2
    # there is some predefined position choose one
    topik.xd=topik.xo_arr[1]
    topik.num_ik()
    print("inverse result",topik.qd)

    ### 4. IK example with camera error
    # set current configuration with default position of niro
    q=topik.set_xd(0,[0.0,0.0,0.0])
    # define camera error vector
    zero=[0.0,0.0,0.0]
    lerr=[-0.1,0.0,0.0]
    rerr=[0.1,0.0,0.0]
    print(q)
    print(topik.xo_arr[0])

    q=[0,0,0,0,0,0,0]
    # calculate motor position with camera zero error 
    qd=topik.error2xd(0,q,zero,zero)
    print("inverse result with camera 0error",qd)
    
    #print("inverse result with camera 0error m",topik.qdm)
    
    # calculate motor position with camera error in indirect method
    #qd=topik.error2xd(0,[0.0,0.0,0.0],q,lerr,rerr)
    #print("inverse result with camera error indirect method",qd)

    # calculate motor position with camera error in Lcam only indirect method
    #qd=topik.error2xd(0,[0.0,0.0,0.0],q,lerr,rerr,1)
    #print("inverse result with left cam",qd)
    # calculate motor position with camera error in Rcam only indirect method
    # qd=topik.error2xd(0,[0.0,0.0,0.0],q,lerr,rerr,2)
    # print("inverse result with right cam",qd)
    #print("inverse result with right cam m",topik.qdm)
    
    # calculate motor position with camera error in direct method
    #qd=topik.error2xd(0,[0.0,0.0,0.0],q,lerr,rerr,3)
    #print("inverse result with camera error direct method",qd)
    

    q=[0,0,0,5625,-5625,0,0]
    qd=topik.error2xd(0,q,lerr,rerr,2)
    # print("inverse result with right cam",qd)
    print(f"rcam_err : {topik.rcam_err}")
    # print(f"rcam_rotation : {topik.so3_rcam}")
    
    
    q=[0,0,0,0,0,0,0]
    qd=topik.error2xd(0,q,lerr,rerr,2)
    # print("inverse result with right cam",qd)
    print(f"rcam_err : {topik.rcam_err}")
    # print(f"rcam_rotation : {topik.so3_rcam}")
    
    q=[0,0,0,-5625,5625,0,0]
    qd=topik.error2xd(0,q,lerr,rerr,2)
    # print("inverse result with right cam",qd)
    print(f"rcam_err : {topik.rcam_err}")
    # print(f"rcam_rotation : {topik.so3_rcam}")

    q=[0,0,0,-2708,2708,0,0]
    qd=topik.error2xd(0,q,lerr,rerr,2)
    # print("inverse result with right cam",qd)
    print(f"rcam_err : {topik.rcam_err}")
    # print(f"rcam_rotation : {topik.so3_rcam}")

