#  Walking engine for Starkit Kondo OpenMV
#  Copyright STARKIT Soccer team of MIPT

from compute_Alpha import compute_Alpha_v3
from ball_Approach_calc import ball_Approach_Calc
from ball_Approach_Steps_Seq import steps
import math, time, sys

#import pyb
HEIGHT_OF_CAMERA = 412.5
HEIGHT_OF_NECK = 44
DIAMETER_OF_BALL = 80

thresholds = [(30, 100, 15, 127, 15, 127), # generic_red_thresholds
              (30, 100, -64, -8, -32, 32), # generic_green_thresholds
              (0, 30, 0, 64, -128, 0)] # generic_blue_thresholds

ACTIVESERVOS = [(10,2),(9,2),(8,2),(7,2),(6,2),(5,2),(4,2),
                (3,2),(2,2),(1,2),(0,2),(10,1),(9,1),(8,1),
                (7,1),(6,1),(5,1),(4,1),(3,1),(2,1),(1,1)]
# (10,2) Прав Стопа Вбок, (9,2) Прав Стопа Вперед,(8,2) Прав Голень, (7,2) Прав Колено,
# (6,2)  Прав Бедро,      (5,2) Прав Таз,         (1,2) Прав Ключица,(4,2) Прав Локоть, (0,2) Торс
# (10,1) Лев Стопа Вбок,  (9,1) Лев Стопа Вперед, (8,1) Лев Голень,  (7,1) Лев Колено
# (6,1)  Лев Бедро,       (5,1) Лев Таз,          (1,1) Лев Ключица, (4,1) Лев Локоть


ACTIVEJOINTS = ['Leg_right_10','Leg_right_9','Leg_right_8','Leg_right_7','Leg_right_6','Leg_right_5','hand_right_4',
                'hand_right_3','hand_right_2','hand_right_1','Tors_0','Leg_left_10','Leg_left_9','Leg_left_8',
                'Leg_left_7','Leg_left_6','Leg_left_5','hand_left_4','hand_left_3','hand_left_2','hand_left_1','head0','head12']

FACTOR = [1,-1,-1,1,-1,-1,-1,1,1,-1,1,1,-1,-1,1,-1,-1,-1,1,1,-1,-1,1]

a5 = 21.5  # мм расстояние от оси симметрии до оси сервы 5
b5 = 18.5  # мм расстояние от оси сервы 5 до оси сервы 6 по горизонтали
c5 = 0     # мм расстояние от оси сервы 6 до нуля Z по вертикали
a6 = 42    # мм расстояние от оси сервы 6 до оси сервы 7
a7 = 65.5  # мм расстояние от оси сервы 7 до оси сервы 8
a8 = 63.8  # мм расстояние от оси сервы 8 до оси сервы 9
a9 = 35.5  # мм расстояние от оси сервы 9 до оси сервы 10
a10= 25.4  # мм расстояние от оси сервы 10 до центра стопы по горизонтали
b10= 16.4  # мм расстояние от оси сервы 10 до низа стопы
c10 = 12   # мм расстояние от оси сервы 6 до оси сервы 10 по горизонтали
SIZES = [ a5, b5, c5, a6, a7, a8, a9, a10, b10, c10 ]

limAlpha5 = [-2667, 2667]
limAlpha6 = [-3000,  740]
limAlpha7 = [-3555, 3260]
limAlpha8 = [-4150, 1777]
limAlpha9 = [-4000, 2960]
limAlpha10 =[-2815,   600]
LIMALPHA = [limAlpha5, limAlpha6, limAlpha7, limAlpha8, limAlpha9, limAlpha10]

RIGHT = True
LEFT = False


class Motion:

    SIMULATION = 1          # 0 - Simulation without physics, 1 - Simulation with physics, 2 - live on openMV
    slowTime = 0.0          # seconds
    simThreadCycleInMs = 20
    stepLength = 0.0          # -50 - +70. Best choise 64 for forward yield 46+78.2*(n-1). Maximum safe value for backward step -50.
    sideLength = 0.0         # -20 - +20. Side step length to right (+) and to left (-). Recommended +/-20 yield 22.5
    rotation = 0               # -45 - +45 degrees Centigrade per step + CW, - CCW.Recommended +/-12 yield 13.6


    # Following paramenetrs Not recommended for change
    amplitude = 32          # mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2
    fr1 =8                  # frame number for 1-st phase of gait ( two legs on floor)
    fr2 = 12                # frame number for 2-nd phase of gait ( one leg in air)
    gaitHeight= 180         # Distance between Center of mass and floor in walk pose
    stepHeight = 32.0       # elevation of sole over floor
    first_Leg_Is_Right_Leg = True
    jointHandle =[]
    limAlpha1 =LIMALPHA
    limAlpha1[3][1]=0
    exitFlag = 0
    Dummy_HData =[]
    BallData =[]
    timeElapsed = 0
    trims = []
    xtr = 0
    ytr = -53.4
    ztr = -gaitHeight
    xr = 0
    yr = 0
    zr = -1
    wr = 0
    xtl = 0
    ytl = 53.4
    ztl = -gaitHeight
    xl = 0
    yl = 0
    zl = -1
    wl = 0
    #------------------------------------------------------------------------------------------------------------------------------------
    def __init__(self, simulation):
        self.SIMULATION = simulation    # 0 - Simulation without physics, 1 - Simulation with physics, 2 - live on openMV
        self.slowTime               # seconds
        self.simThreadCycleInMs
        self.stepLength          # -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
        self.sideLength          # -20 - +20. Side step length to right (+) and to left (-)
        self.rotation            # -45 - +45 degrees Centigrade per step + CW, - CCW.
        self.first_Leg_Is_Right_Leg
        self.initPoses = 400//self.simThreadCycleInMs
        self.Vision_Sensor_Display_On = True
        self.falling_Flag = 0
        self.neck_pan = 0
        with open("calibr.txt", 'r') as f:
            self.neck_calibr = int(f.read())
        self.neck_tilt = self.neck_calibr
        self.direction_To_Attack = 0
        if self.SIMULATION == 1 or self.SIMULATION == 0:
            import sim as vr
            self.vrep = vr
            import numpy as np
            self.np = np
            import matplotlib.pyplot as plt
            self.plt = plt
            import cv2 as cv2
            self.cv2 = cv2
        elif self.SIMULATION == 2 :
            from kondo import Kondo
            from pyb import Pin, UART, LED
            import sensor as s
            self.green_led = LED(2)
            self.pin2 = Pin('P2', Pin.IN, Pin.PULL_UP)
            uart = UART(1, 1250000, parity=0)
            #uart = pyb.UART(1, 1250000, parity=0)
            #k = kondo.Kondo()
            k = Kondo()
            k.init(uart)
            self.kondo = k
            self.clock = time.clock()
            s.reset()
            s.set_pixformat(sensor.RGB565)
            s.set_framesize(sensor.QVGA)
            s.skip_frames(time = 2000)
            s.set_auto_exposure(False)
            s.set_auto_whitebal(False)
            s.skip_frames(time = 1000)
            s.set_auto_gain(False, gain_db=0 ) # 9.172951)
            s.skip_frames(time = 1000)
            s.set_auto_exposure(False, exposure_us=2000) #6576)
            s.skip_frames(time = 1000)
            s.set_auto_whitebal(False, rgb_gain_db = (-6.0, -3.0, 3.0))
            self.sensor = s
    #-------------------------------------------------------------------------------------------------------------------------------



    def quaternion_to_euler_angle(self, quaternion):
        w,x,y,z = quaternion
        ysqr = y*y
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0,t1))
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3,t4))

        return X, Y, Z

    def push_Button(self):
        ala = 0
        while(ala==0):
            if (self.pin2.value()== 0):   # нажатие на кнопку 2 на голове
                ala = 1
                print("нажато")

    def falling_Test(self):
        if self.SIMULATION == 0 or self.SIMULATION == 1:
            returnCode, Camera_quaternion= self.vrep.simxGetObjectQuaternion(self.clientID, self.VisionHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
            self.camera_euler_angle = self.quaternion_to_euler_angle(Camera_quaternion)
        if (self.euler_angle[1]- self.neck_tilt*0.03375) < -45: self.falling_Flag = 1
        if (self.euler_angle[1]- self.neck_tilt*0.03375) >  45: self.falling_Flag = -1
        if 45< (self.euler_angle[2]) < 135: self.falling_Flag = -2
        if -135< (self.euler_angle[2]) < 45: self.falling_Flag = 2

        return self.falling_Flag

    def computeAlphaForWalk(self,sizes, limAlpha ):
        if not self.falling_Test() == 0:  return[]
        angles =[]
        anglesR=[]
        anglesL=[]
        #clock1 = time.clock()
        #clock1.tick()
        anglesR = compute_Alpha_v3(self.xtr,self.ytr,self.ztr,self.xr,self.yr,self.zr,self.wr, sizes, limAlpha)
        anglesL = compute_Alpha_v3(self.xtl,-self.ytl,self.ztl,self.xl,-self.yl,self.zl,self.wl, sizes, limAlpha)
        #print('time elapsed in compute_Alpha:', clock1.avg())
        if len(anglesR)>1:
            for i in range(len(anglesR)):
                if len(anglesR)==1: break
                if anglesR[0][2]<anglesR[1][2]: anglesR.pop(1)
                else: anglesR.pop(0)
        elif len(anglesR)==0:
            return[]
        if len(anglesL)>1:
            for i in range(len(anglesL)):
                if len(anglesL)==1: break
                if anglesL[0][2]<anglesL[1][2]: anglesL.pop(1)
                else: anglesL.pop(0)
        elif len(anglesL)==0:
            return[]

        if self.first_Leg_Is_Right_Leg == True:
            for j in range(6): angles.append(anglesR[0][j])
            angles.append(-1.745)
            angles.append(0.0)
            angles.append(0.0)
            angles.append(0.524 - self.xtl/57.3)
            angles.append(0.0)
            #for j in range(5): angles.append(0.0)
            for j in range(6): angles.append(-anglesL[0][j])
            #for j in range(4): angles.append(0.0)
            angles.append(1.745)
            angles.append(0.0)
            angles.append(0.0)
            angles.append(-0.524 + self.xtr/57.3)
        else:
            for j in range(6): angles.append(anglesL[0][j])
            angles.append(-1.745)
            angles.append(0.0)
            angles.append(0.0)
            angles.append(0.524 - self.xtr/57.3)
            angles.append(0.0)                                  # Tors
            for j in range(6): angles.append(-anglesR[0][j])
            angles.append(1.745)
            angles.append(0.0)
            angles.append(0.0)
            angles.append(-0.524 + self.xtl/57.3)

        return angles

    def vision_Sensor_Get_Image(self):
        #self.vrep.simxSynchronousTrigger(self.clientID)
        returnCode, resolution, image_Data = self.vrep.simxGetVisionSensorImage(self.clientID, self.VisionHandle, 0 ,self.vrep.simx_opmode_buffer)
        nuimg = self.np.array(image_Data, dtype=self.np.uint8)
        nuimg.shape = (resolution[1],resolution[0],3)
        nuimg1 = self.cv2.cvtColor(nuimg, self.cv2.COLOR_RGB2BGR)
        img = self.np.flip(nuimg1, 1)
        return img

    def vision_Sensor_Display(self, img):
        if self.Vision_Sensor_Display_On:
            self.cv2.imshow('Vision Sensor', img)
            self.cv2.waitKey(10) & 0xFF


    def sim_Enable(self):
        print ('Simulation started')
        if self.SIMULATION == 1 or self.SIMULATION  == 0:
            t_start = time.process_time()
            self.vrep.simxFinish(-1) # just in case, close all opened connections
            self.clientID=self.vrep.simxStart('127.0.0.1',19997,True,True,5000,self.simThreadCycleInMs) # Connect to V-REP
            if self.clientID!=-1:
                print ('Connected to remote API server')
            else:
                print ('Failed connecting to remote API server')
                print ('Program ended')
                exit(0)
            # Collect Joint Handles and trims from model

            returnCode, self.Dummy_HHandle = self.vrep.simxGetObjectHandle(self.clientID, 'Dummy_H', self.vrep.simx_opmode_blocking)
            returnCode, self.BallHandle = self.vrep.simxGetObjectHandle(self.clientID, 'Ball', self.vrep.simx_opmode_blocking)
            returnCode, self.ResizableFloorHandle = self.vrep.simxGetObjectHandle(self.clientID, 'ResizableFloor_5_25', self.vrep.simx_opmode_blocking)
            returnCode, self.VisionHandle = self.vrep.simxGetObjectHandle(self.clientID, 'Vision_sensor', self.vrep.simx_opmode_blocking)
            returnCode, self.Ballposition= self.vrep.simxGetObjectPosition(self.clientID, self.BallHandle ,self.ResizableFloorHandle, self.vrep.simx_opmode_streaming)
            returnCode, Dummy_Hposition= self.vrep.simxGetObjectPosition(self.clientID, self.Dummy_HHandle ,self.ResizableFloorHandle, self.vrep.simx_opmode_streaming)
            returnCode, Dummy_Hquaternion= self.vrep.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle ,self.ResizableFloorHandle, self.vrep.simx_opmode_streaming)
            returnCode, resolution, image_Data = self.vrep.simxGetVisionSensorImage(self.clientID, self.VisionHandle, 0 ,self.vrep.simx_opmode_streaming)
            returnCode, Camera_quaternion= self.vrep.simxGetObjectQuaternion(self.clientID, self.VisionHandle ,self.ResizableFloorHandle, self.vrep.simx_opmode_streaming)

            #print(Dummy_Hquaternion)

            for i in range(len(ACTIVEJOINTS)):
                returnCode, a= self.vrep.simxGetObjectHandle(self.clientID, ACTIVEJOINTS[i], self.vrep.simx_opmode_blocking)
                self.jointHandle.append(a)
                returnCode, position= self.vrep.simxGetJointPosition(self.clientID, a, self.vrep.simx_opmode_blocking)
                self.trims.append(position)
            if self.SIMULATION == 1:
                self.vrep.simxSynchronous(self.clientID,True)

    def sim_Start(self):
        if self.SIMULATION == 1:
            time.sleep(0.1)
            self.vrep.simxStartSimulation(self.clientID,self.vrep.simx_opmode_oneshot)
            returnCode, Dummy_Hquaternion= self.vrep.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
            self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)
            self.direction_To_Attack = self.euler_angle[0]
            returnCode, Camera_quaternion= self.vrep.simxGetObjectQuaternion(self.clientID, self.VisionHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
            self.camera_euler_angle = self.quaternion_to_euler_angle(Camera_quaternion)
            print('euler_angle = ', self.euler_angle)
            returnCode, Dummy_Hposition= self.vrep.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
            self.Dummy_HData.append(Dummy_Hposition)
            returnCode, self.Ballposition= self.vrep.simxGetObjectPosition(self.clientID, self.BallHandle ,self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)

    def walk_Initial_Pose(self):
        #tmp1 = self.first_Leg_Is_Right_Leg
        #if self.rotation>0 or self.sideLength>0:  self.first_Leg_Is_Right_Leg = False
        #else: self.first_Leg_Is_Right_Leg = True
        for j in range (self.initPoses):
            self.ztr = -223.0 + j*(223.0-self.gaitHeight)/self.initPoses
            self.ztl = -223.0 + j*(223.0-self.gaitHeight)/self.initPoses
            self.ytr = -53.4 - j*self.amplitude/2 /self.initPoses
            self.ytl =  53.4 - j*self.amplitude/2 /self.initPoses
            angles = self.computeAlphaForWalk(SIZES, self.limAlpha1 )
            if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.SIMULATION == 1 or self.SIMULATION  == 0:
                    for i in range(len(angles)):
                        if self.SIMULATION == 1: returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                                            self.jointHandle[i] , angles[i]*FACTOR[i], self.vrep.simx_opmode_oneshot)
                        elif self.SIMULATION == 0: returnCode = self.vrep.simxSetJointPosition(self.clientID,
                                            self.jointHandle[i] , angles[i]*FACTOR[i], self.vrep.simx_opmode_oneshot)
                    if self.SIMULATION == 1 or self.SIMULATION  == 0:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.vrep.simxGetObjectPosition(self.clientID,
                                              self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
                        #self.Dummy_HData.append(Dummy_Hposition)
                        #self.timeElapsed = self.timeElapsed +1
                        # print(Dummy_Hposition)
                        self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                    if self.SIMULATION == 1: self.vrep.simxSynchronousTrigger(self.clientID)
                elif self.SIMULATION == 2:
                    servoDatas = []
                    for i in range(len(angles)):
                        pos = int(angles[i]*1698 + 7500)
                        servoDatas.append( self.kondo.rcb4.ServoData(ACTIVESERVOS[i][0],ACTIVESERVOS[i][1],pos))

                    a=self.kondo.rcb4.setServoPos (servoDatas,2)
                    #print(servoDatas)
                    #print(clock.avg())
                    pyb.delay(3)
        #self.first_Leg_Is_Right_Leg = tmp1

    def walk_Cycle(self, stepLength,sideLength, rotation,cycle, number_Of_Cycles):
        self.stepLength = stepLength
        self.sideLength = sideLength
        self.rotation = rotation
        #tmp1 = self.first_Leg_Is_Right_Leg
        #if rotation>0 or sideLength>0:  self.first_Leg_Is_Right_Leg = False
        #else: self.first_Leg_Is_Right_Leg = True
        rotation = -self.rotation/200
        alpha = 0
        alpha01 = math.pi/self.fr1*2
        frameNumberPerCycle = 2*self.fr1+2*self.fr2
        for iii in range(0,frameNumberPerCycle,2):
            if self.SIMULATION == 2: self.clock.tick()
            if 0<= iii <self.fr1 :
                alpha = alpha01 * (iii/2+1)
                S = (self.amplitude/2 + self.sideLength/2 )*math.cos(alpha)
                self.ytr = S - 53.4 + self.sideLength/2
                self.ytl = S + 53.4 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                if cycle ==0: continue
                    #dx0 = 0
                else: dx0 = self.stepLength/(2*self.fr1+self.fr2+4)*2
                #self.xtl = self.stepLength/4 - dx0*(iii/2+1)
                xtl0 = - (self.fr2 + self.fr1 + 4) * self.stepLength/(2*self.fr1+self.fr2+4) + self.stepLength
                xtr0 = self.stepLength/2 - (self.fr2 + self.fr1 +4 ) * self.stepLength/(2*self.fr1+self.fr2+4)
                self.xtl = xtl0 - dx0*(iii/2+1)
                self.xtr = xtr0 - dx0*(iii/2+1)
            if self.fr1+self.fr2<=iii<2*self.fr1+self.fr2 :
                alpha = alpha01 * ((iii-self.fr2)/2+1)
                S = (self.amplitude/2 + self.sideLength/2)*math.cos(alpha)
                self.ytr = S - 53.4 - self.sideLength/2
                self.ytl = S + 53.4 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                dx0 = self.stepLength/(2*self.fr1+self.fr2+4)*2
                self.xtl = self.xtl - dx0
                self.xtr = self.xtr - dx0
            if self.fr1<= iii <self.fr1+self.fr2:
                #print('ytr = ',self.ytr, 'ytl = ', self.ytl )
                self.ztr = -self.gaitHeight + self.stepHeight
                #self.ytr = S - 64
                if cycle ==0:
                    dx = self.stepLength/2/self.fr2*2
                    dx0 = self.stepLength/(2*self.fr1+self.fr2+4)*2
                    dy = self.sideLength/self.fr2*2
                    dy0 = self.sideLength/(2*self.fr1+self.fr2+4)*2

                else:
                    dx = self.stepLength/self.fr2*2
                    dx0 = self.stepLength/(2*self.fr1+self.fr2+4)*2
                    dy = self.sideLength/self.fr2*2
                    dy0 = self.sideLength/(2*self.fr1+self.fr2+4)*2
                if iii==self.fr1:
                    self.xtr = self.xtr - dx0
                    self.ytr = S - 64 + dy0
                elif iii == (self.fr1 +self.fr2 - 2):
                    self.xtr = self.xtr - dx0
                    self.ytr = S - 64 + 2*dy0 - self.sideLength
                else:
                    self.xtr = self.xtr + dx*self.fr2/(self.fr2-4)
                    self.ytr = S - 64 + dy0 - dy*self.fr2/(self.fr2-4)*((iii - self.fr1)/2)
                    self.wl = rotation -(iii-self.fr1)* rotation/(self.fr2-4)*2
                    self.wr = rotation -(iii-self.fr1)* rotation/(self.fr2-4)*2
                self.xtl = self.xtl - dx0
                self.ytl = self.ytl + dy0
            if 2*self.fr1+self.fr2<= iii :
                #print('ytr = ',self.ytr, 'ytl = ', self.ytl )
                self.ztl = -self.gaitHeight + self.stepHeight
                #self.ytl = S + 64
                if cycle == number_Of_Cycles - 1:
                    dx0 =self.stepLength/(2*self.fr1+self.fr2+4)*4 / self.fr2 * 2          # 8.75/6
                    dx = (self.stepLength*(self.fr1+self.fr2)/(4*self.fr1)+2*dx0)/(self.fr2-4) *2
                    if iii== (2*self.fr1 + 2*self.fr2 - 2):
                        self.ztl = -self.gaitHeight
                        self.ytl = S + 53.4
                else:
                    dx = self.stepLength/(self.fr2-4) *2
                    dx0 = self.stepLength/(2*self.fr1+self.fr2+4)*2
                    dy = self.sideLength/(self.fr2-4) *2
                    dy0 = self.sideLength/(2*self.fr1+self.fr2+4)*2
                if iii== (2*self.fr1 + self.fr2 ):
                    self.xtl = self.xtl - dx0
                    self.ytl = S + 64 + dy0
                elif iii== (2*self.fr1 + 2*self.fr2 - 2):
                    self.xtl = self.xtl - dx0
                    self.ytl = S + 64 + 2*dy0 - self.sideLength
                else:
                    self.xtl = self.xtl + dx
                    self.ytl = S + 64 + dy0 - dy * (iii -(2*self.fr1+self.fr2) )/2
                    self.wl = (iii-(2*self.fr1+self.fr2))* rotation/(self.fr2-4) *2 - rotation
                    self.wr = (iii-(2*self.fr1+self.fr2))* rotation/(self.fr2-4) *2 - rotation
                self.xtr = self.xtr - dx0
                self.ytr = self.ytr + dy0
            #print('xtl = ',self.xtl, 'xtr = ',self.xtr)
            #print(angles)
            angles = self.computeAlphaForWalk(SIZES, self.limAlpha1 )
            if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.SIMULATION == 1 or self.SIMULATION  == 0:
                    for i in range(len(angles)):
                        if self.SIMULATION == 1: returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                                                          self.jointHandle[i] , angles[i]*FACTOR[i], self.vrep.simx_opmode_oneshot)
                        elif self.SIMULATION == 0: returnCode = self.vrep.simxSetJointPosition(self.clientID,
                                                          self.jointHandle[i] , angles[i]*FACTOR[i], self.vrep.simx_opmode_oneshot)

                    if self.SIMULATION == 1 or self.SIMULATION  == 0:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.vrep.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
                        self.Dummy_HData.append(Dummy_Hposition)
                        returnCode, self.Ballposition= self.vrep.simxGetObjectPosition(self.clientID, self.BallHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
                        self.BallData.append(self.Ballposition)
                        returnCode, Dummy_Hquaternion= self.vrep.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
                        self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)
                        #print(self.euler_angle)
                        self.timeElapsed = self.timeElapsed +1
                        #print(Dummy_Hposition)
                        self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                        if self.SIMULATION == 1: self.vrep.simxSynchronousTrigger(self.clientID)
                elif self.SIMULATION == 2:
                    #print(' elapsed time =',clock.avg())
                    servoDatas = []
                    disp = []
                    for i in range(len(angles)):
                        pos = int(angles[i]*1698 + 7500)
                        disp.append((ACTIVEJOINTS[i],ACTIVESERVOS[i][0],ACTIVESERVOS[i][1],pos -7500))
                        servoDatas.append( self.kondo.rcb4.ServoData(ACTIVESERVOS[i][0],ACTIVESERVOS[i][1],pos))

                    a=self.kondo.rcb4.setServoPos (servoDatas,2)
                    #print(disp)
                    print(' elapsed time =', self.clock.avg())
                    pyb.delay(3)
        #self.first_Leg_Is_Right_Leg = tmp1

    def walk_Final_Pose(self):
        for j in range (self.initPoses):
            self.ztr = -self.gaitHeight - (j+1)*(223.0-self.gaitHeight)/self.initPoses
            self.ztl = -self.gaitHeight - (j+1)*(223.0-self.gaitHeight)/self.initPoses
            self.ytr = -53.4 - (self.initPoses-(j+1))*self.amplitude/2 /self.initPoses
            self.ytl =  53.4 - (self.initPoses-(j+1))*self.amplitude/2 /self.initPoses
            angles = self.computeAlphaForWalk(SIZES, self.limAlpha1 )
            if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.SIMULATION == 1 or self.SIMULATION  == 0:
                    for i in range(len(angles)):
                        if self.SIMULATION == 1: returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                                            self.jointHandle[i] , angles[i]*FACTOR[i], self.vrep.simx_opmode_oneshot)
                        elif self.SIMULATION == 0: returnCode = self.vrep.simxSetJointPosition(self.clientID,
                                            self.jointHandle[i] , angles[i]*FACTOR[i], self.vrep.simx_opmode_oneshot)
                    if self.SIMULATION == 1 or self.SIMULATION  == 0:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.vrep.simxGetObjectPosition(self.clientID,
                                              self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
                        #self.Dummy_HData.append(Dummy_Hposition)
                        #self.timeElapsed = self.timeElapsed +1
                        # print(Dummy_Hposition)
                        self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                    if self.SIMULATION == 1: self.vrep.simxSynchronousTrigger(self.clientID)
                elif self.SIMULATION == 2:
                    servoDatas = []
                    for i in range(len(angles)):
                        pos = int(angles[i]*1698 + 7500)
                        servoDatas.append( self.kondo.rcb4.ServoData(ACTIVESERVOS[i][0],ACTIVESERVOS[i][1],pos))

                    a=self.kondo.rcb4.setServoPos (servoDatas,2)
                    #print(servoDatas)
                    #print(clock.avg())
                    pyb.delay(3)
    def sim_Progress(self,simTime):  # simTime in seconds
        if self.SIMULATION == 1 or self.SIMULATION  == 0:
            for i in range(simTime*1000//self.simThreadCycleInMs):
                returnCode, Dummy_Hposition= self.vrep.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
                self.Dummy_HData.append(Dummy_Hposition)
                returnCode, self.Ballposition= self.vrep.simxGetObjectPosition(self.clientID, self.BallHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
                self.BallData.append(self.Ballposition)
                returnCode, Dummy_Hquaternion= self.vrep.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
                #print(quaternion_to_euler_angle(Dummy_Hquaternion))
                self.timeElapsed = self.timeElapsed +1
                self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                self.vrep.simxSynchronousTrigger(self.clientID)

    def sim_Stop(self):
        if self.SIMULATION == 1 or self.SIMULATION  == 0:
            self.vrep.simxStopSimulation(self.clientID,self.vrep.simx_opmode_oneshot)
                #a = input('Press "e" for end or <enter> zt to continue ')
                #if a=='e': break
                #zt = float(a)
                #a = input('Press "e" for end or <enter> y to continue ')
                #if a=='e': break
                #y = float(a)
                #a = input('Press "e" for end or <enter> z to continue ')
                #if a=='e': break
                #z = float(a)
                    # return to initial pose
            for j in range(len(ACTIVEJOINTS)):
                if self.SIMULATION == 1: returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                                     self.jointHandle[j] , self.trims[j], self.vrep.simx_opmode_oneshot)
                else: returnCode = self.vrep.simxSetJointPosition(self.clientID,
                                     self.jointHandle[j] , self.trims[j], self.vrep.simx_opmode_oneshot)
    def print_Diagnostics(self):
        Dummy_HDataX =[]
        Dummy_HDataY =[]
        Dummy_HDataZ =[]
        for i in range (self.timeElapsed):
            Dummy_HDataX.append( self.Dummy_HData[i][0])
            Dummy_HDataY.append(self.Dummy_HData[i][1])
            Dummy_HDataZ.append(self.Dummy_HData[i][2])
        BallDataX =[]
        BallDataY =[]
        BallDataZ =[]
        for i in range (self.timeElapsed):
            BallDataX.append( self.BallData[i][0])
            BallDataY.append(self.BallData[i][1])
            BallDataZ.append(self.BallData[i][2])
        print('exitFlag' ,self.exitFlag)
        rng = self.np.arange(self.timeElapsed)
        fig,ax = self.plt.subplots(figsize=(10,6))
        self.plt.plot(rng,Dummy_HDataX, label = 'Body X')
        self.plt.plot(rng,Dummy_HDataY, label = 'Body Y')
        self.plt.plot(rng,Dummy_HDataZ, label = 'Body Z')
        self.plt.plot(rng,BallDataX, label = 'Ball X')
        self.plt.plot(rng,BallDataY, label = 'Ball Y')
        self.plt.plot(rng,BallDataZ, label = 'Ball Z')
        ax.legend(loc='upper left')
        ax.grid(True)
        if self.SIMULATION == 1:
            self.plt.show()
            #break
        print('exitFlag' ,self.exitFlag)

    def sim_Disable(self):            # Now close the connection to V-REP:
        time.sleep(0.2)
        self.vrep.simxFinish(self.clientID)

    def kick(self, first_Leg_Is_Right_Leg):
        gaitHeight = 210 #210
        stepHeight = 55 #55
        stepLength = 64 #64
        tmp1 = self.first_Leg_Is_Right_Leg
        self.first_Leg_Is_Right_Leg = first_Leg_Is_Right_Leg
        tmp = self.gaitHeight
        self.gaitHeight = gaitHeight
        self.walk_Initial_Pose()
        alpha = 0
        alpha01 = math.pi/self.fr1*2
        frameNumberPerCycle = 2*self.fr1+2*self.fr2
        for iii in range(0,frameNumberPerCycle,2):
            if self.SIMULATION == 2: self.clock.tick()
            if 0<= iii <self.fr1 :
                alpha = alpha01 * (iii/2+1)
                S = (self.amplitude/2 )*math.cos(alpha)
                self.ytr = S - 53.4 
                self.ytl = S + 53.4 
                self.ztl = -gaitHeight
                self.ztr = -gaitHeight
                continue

            if self.fr1+self.fr2<=iii<2*self.fr1+self.fr2 :
                alpha = alpha01 * ((iii-self.fr2)/2+1)
                S = (self.amplitude/2)*math.cos(alpha)
                self.ytr = S - 53.4 
                self.ytl = S + 53.4 
                self.ztl = -gaitHeight
                self.ztr = -gaitHeight
                dx0 = stepLength/(2*self.fr1+self.fr2+4)*2
                self.xtl = self.xtl - dx0
                self.xtr = self.xtr - dx0
            if self.fr1<= iii <self.fr1+self.fr2:
                #print('ytr = ',self.ytr, 'ytl = ', self.ytl )
                self.ztr = -gaitHeight + stepHeight
                #self.ytr = S - 64
                dx = stepLength/2/self.fr2*2
                dx0 = stepLength/(2*self.fr1+self.fr2+4)*2
                if iii==self.fr1:
                    self.xtr = self.xtr - dx0
                    self.ytr = S - 64
                elif iii == (self.fr1 +self.fr2 - 2):
                    self.xtr = self.xtr - dx0
                    self.ytr = S - 64 
                else:
                    self.xtr = self.xtr + dx*self.fr2/(self.fr2-4)
                    self.ytr = S - 64 
                if iii == self.fr1 +self.fr2 - 10: self.xtr = self.xtr + 70
                if iii == self.fr1 +self.fr2 - 4: self.xtr = self.xtr - 70
                self.xtl = self.xtl - dx0
            if 2*self.fr1+self.fr2<= iii :
                #print('ytr = ',self.ytr, 'ytl = ', self.ytl )
                self.ztl = -gaitHeight + stepHeight
                #self.ytl = S + 64
                dx0 = stepLength/(2*self.fr1+self.fr2+4)*4 / self.fr2 * 2          # 8.75/6
                dx = (stepLength*(self.fr1+self.fr2)/(4*self.fr1)+2*dx0)/(self.fr2-4) *2
                if iii== (2*self.fr1 + 2*self.fr2 - 2):
                    self.ztl = -gaitHeight
                    self.ytl = S + 53.4

                if iii== (2*self.fr1 + self.fr2 ):
                    self.xtl = self.xtl - dx0
                    self.ytl = S + 64
                elif iii== (2*self.fr1 + 2*self.fr2 - 2):
                    self.xtl = self.xtl - dx0
                    self.ytl = S + 64 
                else:
                    self.xtl = self.xtl + dx
                    self.ytl = S + 64
                self.xtr = self.xtr - dx0
            #print('xtl = ',self.xtl, 'xtr = ',self.xtr)
            #print(angles)
            angles = self.computeAlphaForWalk(SIZES, self.limAlpha1 )
            if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.SIMULATION == 1 or self.SIMULATION  == 0:
                    for i in range(len(angles)):
                        if self.SIMULATION == 1: returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                                                          self.jointHandle[i] , angles[i]*FACTOR[i], self.vrep.simx_opmode_oneshot)
                        elif self.SIMULATION == 0: returnCode = self.vrep.simxSetJointPosition(self.clientID,
                                                          self.jointHandle[i] , angles[i]*FACTOR[i], self.vrep.simx_opmode_oneshot)

                    if self.SIMULATION == 1 or self.SIMULATION  == 0:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.vrep.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
                        returnCode, Dummy_Hquaternion= self.vrep.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
                        self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)
                        #print(self.euler_angle)
                        self.Dummy_HData.append(Dummy_Hposition)
                        returnCode, self.Ballposition= self.vrep.simxGetObjectPosition(self.clientID, self.BallHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
                        self.BallData.append(self.Ballposition)
                        self.timeElapsed = self.timeElapsed +1
                        #print(Dummy_Hposition)
                        self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                        if self.SIMULATION == 1: self.vrep.simxSynchronousTrigger(self.clientID)
                elif self.SIMULATION == 2:
                    #print(' elapsed time =',clock.avg())
                    servoDatas = []
                    disp = []
                    for i in range(len(angles)):
                        pos = int(angles[i]*1698 + 7500)
                        disp.append((ACTIVEJOINTS[i],ACTIVESERVOS[i][0],ACTIVESERVOS[i][1],pos -7500))
                        servoDatas.append( self.kondo.rcb4.ServoData(ACTIVESERVOS[i][0],ACTIVESERVOS[i][1],pos))

                    a=self.kondo.rcb4.setServoPos (servoDatas,2)
                    #print(disp)
                    print(' elapsed time =', self.clock.avg())
                    pyb.delay(3)
        self.walk_Final_Pose()
        self.gaitHeight = tmp
        self.first_Leg_Is_Right_Leg = tmp1

    def head_Tilt_Calibration(self):
            # Калибрация Наклона камеры. Установить мяч на расстоянии 77 см (100 см)по низу мяча.
            # Полученная величина наклона камеры эквивалентна 62 (69.4) градуса от вертикали.
            # Вторая позиция головы 23 градуса к вертикали отличается от первой на 1155
        threshold_index = 0 # 0 for red, 1 for green, 2 for blue

            # Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
            # The below thresholds track in general red/green/blue things. You may wish to tune them...
        thresholds = [(30, 100, 15, 127, 15, 127), # generic_red_thresholds
                          (30, 100, -64, -8, -32, 32), # generic_green_thresholds
                          (0, 30, 0, 64, -128, 0)] # generic_blue_thresholds
        if self.SIMULATION == 2:
            import sensor, image, time, math, kondo,pyb

            sensor.reset()
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.QVGA)
            sensor.skip_frames(time = 2000)
            sensor.set_auto_gain(False) # must be turned off for color tracking
            sensor.set_auto_whitebal(False) # must be turned off for color tracking
            clock = time.clock()

            kondo = kondo.Kondo()
            uart = pyb.UART(1, 115200, parity=0)
            kondo.init(uart)

            i= -200
            a = True
            kondo.rcb4.motionPlay(2)
            #kondo.rcb4.setSingleServo( 0, 1 , 0, 10)
            kondo.rcb4.setUserParameter(19,0)
            while(a):
                if (i < -1500) : a=False
                clock.tick()
                i=i-1
                #kondo.rcb4.setSingleServo( 12, 2 , i, 10)
                a=kondo.rcb4.setUserParameter(20,i)
                img = sensor.snapshot()
                for blob in img.find_blobs([thresholds[threshold_index]], pixels_threshold=200, area_threshold=200, merge=True):
                    if blob.roundness() > 0.9:
                        img.draw_rectangle(blob.rect())
                        img.draw_cross(blob.cx(), blob.cy())
                        #print(blob.cy(),blob.w(), blob.h())
                        if (blob.y()+ blob.h())==120 : a=False
            #print(i)
            with open("calibr.txt", 'w') as f:
                f.write(str(i))
            return
        else:
            import reload as re
            import cv2
            i= -200
            a = True
            returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[21] , 0*FACTOR[21], self.vrep.simx_opmode_oneshot) #head pan to zero
            returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , 0*FACTOR[22], self.vrep.simx_opmode_oneshot) #head tilt to zero
            while(a):
                if (i < -1500) : a=False
                #clock.tick()
                i=i-1
                returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , i*0.000589*FACTOR[22], self.vrep.simx_opmode_oneshot)
                self.vrep.simxSynchronousTrigger(self.clientID)
                img1 = self.vision_Sensor_Get_Image()
                img = re.Image(img1)
                #for blob in img.find_blobs((45, 60, 75, 85, 60, 75), pixels_threshold=200, area_threshold=200, merge=True):
                #for blob in img.find_blobs((18, 55, 20, 77, 28, 57), pixels_threshold=200, area_threshold=200, merge=True):
                for blob in img.find_blobs((13, 58, 27, 98, 8, 95), pixels_threshold=100, area_threshold=100, merge=True):
                #for blob in img.find_blobs(thresholds [0], pixels_threshold=100, area_threshold=200, merge=True):
                    #if blob.roundness() > 0.9:
                    #print(blob.cx(), blob.cy(),blob.w(), blob.h())
                    img.draw_rectangle(blob.rect())
                    #img.draw_cross(blob.cx(), blob.cy())
                
                    if blob.cy()==120 : a=False
                self.vision_Sensor_Display(img.img)
            with open("calibr.txt", 'w') as f:
                f.write(str(i))
            print( self.get_course_and_distance(blob))
        
    def seek_Ball(self, fast_Reaction_On):                  # seeks ball in 360 dergree one time
        a, course, distance = self.seek_Ball_In_Pose(fast_Reaction_On)
        if a == True: return a, course, distance
        else:
            target_course = self.euler_angle[0] +160
            self.turn_To_Course(target_course)
            a, course, distance = self.seek_Ball_In_Pose(fast_Reaction_On)
            if a == True: return a, course, distance
            else:
                target_course = self.euler_angle[0] -160
                self.turn_To_Course(target_course)
                return False, 0, 0

    def seek_Ball_In_Pose(self, fast_Reaction_On):
        variants = []
        # U19 - Шея поворот
        # U20 - Шея Наклон
        c = self.neck_calibr
        #head_pose = [(-1333, c), (-2667,c), (-2667, c-593),(-1333, c-593), (0, c-593), (1333,c-593),(2667, c-593), (2667, c-1186), (1333, c-1186), ( 0, c-1186), (-1333, c-1186), (-2667, c-1186), (2667,c), (1333, c), ( 0, c) ]
        #head_pose = [(-2667,c), (-1333, c) , ( 0, c) , (1333, c) , (2667,c),  
        #             (-2667, c-593),(-1333, c-593), (0, c-593), (1333,c-593),(2667, c-593),
        #            (-2667, c-1186), (-1333, c-1186), ( 0, c-1186), (1333, c-1186), (2667, c-1186)]
        head_pose = [(-2667,c), (-1333, c) , ( 0, c) , (1333, c) , (2667,c),  
                     (-2667, c-700),(-1333, c-700), (0, c-700), (1333,c-700),(2667, c-700),
                    (-2667, c-1400), (-1333, c-1400), ( 0, c-1400), (1333, c-1400), (2667, c-1400)]
        #head_pose_seq = [2,7,12,13,8,3,1,6,11,10,5,0,4,9,14,2]
        head_pose_seq = [2,7,12,11,6,8,13,14,9,4,3,10,5,0,1,2]
        for i in range(len(head_pose_seq)):
            if not(fast_Reaction_On == True and i == 0): 
                x = head_pose[head_pose_seq[i]]
                self.neck_pan = x[0]
                self.neck_tilt = x[1]
            if self.SIMULATION == 2:
                self.kondo.rcb4.setUserParameter(19,self.neck_pan)
                pyb.delay(200)
                self.kondo.rcb4.setUserParameter(20,self.neck_tilt)
                pyb.delay(200)
            else:
                returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[21] , self.neck_pan*0.000589*FACTOR[21], self.vrep.simx_opmode_oneshot)   # Шея поворот
                returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , self.neck_tilt*0.000589*FACTOR[22], self.vrep.simx_opmode_oneshot)  # Шея Наклон
                for i in range(16): self.vrep.simxSynchronousTrigger(self.clientID)
            a, course, distance, blob = self.seek_Ball_In_Frame()
            if a == True: variants.append ((course, distance))
            if fast_Reaction_On == True and a== True: break
        course = 0
        distance = 0
        if len(variants)>0:
            for i in range (len(variants)):
                course = course + variants[i][0]
                distance = distance + variants[i][1]
            course  = course /len(variants)
            distance = distance /len(variants)
            self.neck_pan = - course/ 0.03375
            D = HEIGHT_OF_CAMERA - HEIGHT_OF_NECK- DIAMETER_OF_BALL/2
            E = (2*distance*D - math.sqrt(4*distance**2*D**2 - 4*(distance**2-HEIGHT_OF_NECK**2)*(D**2 -HEIGHT_OF_NECK**2)))/(2*(D**2-HEIGHT_OF_NECK**2))
            alpha = math.atan(E)
            alpha_d = math.degrees(alpha)
            self.neck_tilt = (alpha_d - 69.4)/0.03375 + c
            returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[21] , self.neck_pan*0.000589*FACTOR[21], self.vrep.simx_opmode_oneshot)   # Шея поворот
            returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , self.neck_tilt*0.000589*FACTOR[22], self.vrep.simx_opmode_oneshot)  # Шея Наклон
            for i in range(16): self.vrep.simxSynchronousTrigger(self.clientID)
            a, course, distance, blob = self.seek_Ball_In_Frame()
            returnCode, Dummy_Hquaternion= self.vrep.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
            self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)
            course = self.euler_angle[0]
            if a == True: return(True, course, distance)
        return False, 0, 0

    def seek_Ball_In_Frame(self):
        tra = 0
        for number in range (10):
            if self.SIMULATION == 2:
                img = self.sensor.snapshot().lens_corr(strength = 1.45, zoom = 1.0)
                blobs = img.find_blobs([thresholds[0]], pixels_threshold=50, area_threshold=50, merge=True, margin=10)
                if (len (blobs) == 1):
                    blob = blobs [0]
                    self.green_led.on()  # подмигивание зеленым светодиодом
                    time.sleep(50)  #
                    self.green_led.off() #
                    tra = tra + 1
                    img.draw_rectangle(blob.rect(), color = (255, 0, 0))
                    course, distance = get_course_and_distance(blob)
                    return True, course, distance, blob
            else:
                import reload as re
                import cv2
                self.vrep.simxSynchronousTrigger(self.clientID)
                img1 = self.vision_Sensor_Get_Image()
                img = re.Image(img1)
                self.vision_Sensor_Display(img.img)
                blobs = img.find_blobs((13, 58, 27, 98, 8, 95), pixels_threshold=100, area_threshold=100, merge=True)
                if (len (blobs) == 1):
                    blob = blobs [0]
                    tra = tra + 1
                    img.draw_rectangle(blob.rect())
                    self.vision_Sensor_Display(img.img)
                    course, distance = self.get_course_and_distance(blob)
                    return True, course, distance, blob
            #print("number =   ", number)
        if tra == 0: return False, 0, 0, 0

    def get_course_and_distance(self, blob):   # returns course in degrees and distance in cm
        c = self.neck_calibr
        if self.SIMULATION == 2:
            z,a = self.kondo.rcb4.getUserParameter(19)
            z,b = self.kondo.rcb4.getUserParameter(20)
            # U19 - Шея поворот
            # U20 - Шея Наклон
            course = a*0.03375 + (160 - blob.cx())*0.1875
            x = 62 + (b-c)*0.03375 +(120-blob.y() - blob.h())*0.1875
            y=math.radians(x)
            distance = 41* math.tan(y)
        else: 
            returnCode, position21= self.vrep.simxGetJointPosition(self.clientID, self.jointHandle[21], self.vrep.simx_opmode_blocking)
            a = position21*FACTOR[21]*1698        # Шея поворот
            returnCode, position22= self.vrep.simxGetJointPosition(self.clientID, self.jointHandle[22], self.vrep.simx_opmode_blocking)
            b = position22*FACTOR[22]*1698        # Шея Наклон
            course = -a*0.03375 + (160 - blob.cx())*0.14375
            x = 69.4 + (b-c)*0.03375 +(120-blob.cy())*0.14375
            y=math.radians(x)
            distance_in_mm = (HEIGHT_OF_CAMERA - HEIGHT_OF_NECK + HEIGHT_OF_NECK*math.sin(y)- DIAMETER_OF_BALL/2) * math.tan(y) + HEIGHT_OF_NECK*math.cos(y)
        return course, distance_in_mm


    #def turn_To_Course(self, course):
    #    stepLength = 0
    #    sideLength = 0
    #    rotation = 0
    #    target = self.direction_To_Attack + course
    #    returnCode, Dummy_Hquaternion= self.vrep.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
    #    self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)
    #    rotation1 = target - self.euler_angle[0]
    #    if rotation1 > 180 : rotation1 = rotation1 - 360
    #    if rotation1 < -180 : rotation1 = rotation1 + 360
    #    if abs(rotation1)> 2:
    #        self.walk_Initial_Pose()
    #        if abs(rotation1)> 12:
    #            cycleNumber= int(math.floor(abs(rotation1)/13.2))
    #            rotation = math.copysign(12, rotation1)
    #            for cycle in range (cycleNumber):
    #                self.walk_Cycle(stepLength, sideLength,rotation,cycle,cycleNumber)
    #        returnCode, Dummy_Hquaternion= self.vrep.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
    #        self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)
    #        rotation1 = target - self.euler_angle[0]
    #        if rotation1 > 180 : rotation1 = rotation1 - 360
    #        if rotation1 < -180 : rotation1 = rotation1 + 360
    #        rotation = rotation1*12/13.2
    #        print('self.euler_angle[0]=', self.euler_angle[0],'rotation =', rotation )
    #        cycleNumber = 1
    #        cycle = 0
    #        self.walk_Cycle(stepLength, sideLength,rotation,cycle,cycleNumber)
    #        self.walk_Final_Pose()
    def turn_To_Course(self, course):
        stepLength = 0
        sideLength = 0
        rotation = 0
        cycleNumber = 1
        cycle = 0
        target = self.direction_To_Attack + course
        self.head_Up()
        returnCode, Dummy_Hquaternion= self.vrep.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
        self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)
        rotation1 = target - self.euler_angle[0]
        if rotation1 > 180 : rotation1 = rotation1 - 360
        if rotation1 < -180 : rotation1 = rotation1 + 360
        if abs(rotation1)> 2:
            cycleNumber = int(math.floor(abs(rotation1)/13.2))+1
            self.walk_Initial_Pose()
            for cycle in range (cycleNumber):
                returnCode, Dummy_Hquaternion= self.vrep.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , self.ResizableFloorHandle, self.vrep.simx_opmode_buffer)
                self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)
                rotation1 = target - self.euler_angle[0]
                if rotation1 > 180 : rotation1 = rotation1 - 360
                if rotation1 < -180 : rotation1 = rotation1 + 360
                if abs(rotation1)< 2: break
                rotation = rotation1*12/13.2/(cycleNumber - cycle)
                #print('self.euler_angle[0]=', self.euler_angle[0],'rotation =', rotation )
                self.walk_Cycle(stepLength, sideLength,rotation,cycle,cycleNumber)
            self.walk_Final_Pose()
        self.head_Return()
    
    def head_Up(self):
        if self.SIMULATION == 2:
            self.kondo.rcb4.setUserParameter(19,0)
            pyb.delay(200)
            self.kondo.rcb4.setUserParameter(20,self.neck_calibr)
            pyb.delay(200)
        else:
             returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[21] , 0, self.vrep.simx_opmode_oneshot)   # Шея поворот
             returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , self.neck_calibr*0.000589*FACTOR[22], self.vrep.simx_opmode_oneshot)  # Шея Наклон

    def head_Return(self):
        if self.SIMULATION == 2:
            self.kondo.rcb4.setUserParameter(19,self.neck_pan)
            pyb.delay(200)
            self.kondo.rcb4.setUserParameter(20,self.neck_tilt)
            pyb.delay(200)
        else:
             returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[21] , self.neck_pan*0.000589*FACTOR[21], self.vrep.simx_opmode_oneshot)   # Шея поворот
             returnCode = self.vrep.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , self.neck_tilt*0.000589*FACTOR[22], self.vrep.simx_opmode_oneshot)  # Шея Наклон




if __name__=="__main__":
    SIMULATION = 1
    motion = Motion(SIMULATION)
    motion.slowTime = 0.0
    motion.sim_Enable()
    motion.sim_Start()
    motion.Vision_Sensor_Display_On = False
    x1, y1, z1 = motion.Dummy_HData[len(motion.Dummy_HData)-1]
    player_X_m, player_Y_m = x1, y1
    x1 = 1000* x1
    y1 = 1000* y1
    u1 = motion.euler_angle[0]
    print('u1 = ', u1)
    ball_X_m, ball_Y_m, ball_Z_m = motion.Ballposition
    print('ballposition= ', motion.Ballposition)
    destination = ball_Approach_Calc(ball_X_m, ball_Y_m, player_X_m, player_Y_m, 3.6, 2.6 )
    x2 = -0.65 *1000
    y2 = 0.13 *1000
    u2 = 90
    for stop_point in range (len(destination)):
        x1, y1, z1 = motion.Dummy_HData[len(motion.Dummy_HData)-1]
        x1 = 1000* x1
        y1 = 1000* y1
        u1 = motion.euler_angle[0]
        x2,y2,u2 = destination[stop_point]
        x2 = 1000* x2
        y2 = 1000* y2
        step_Seq, walk_Direction = steps( x1 ,y1,u1,x2,y2,u2)
        for i in range (len(step_Seq)):
            motion.walk_Initial_Pose()
            stepLength, sideLength, rotation, cycleNumber = step_Seq[i]
            for cycle in range(cycleNumber):
                rotation1 = rotation
                if rotation == 0: 
                    rotation1 = (walk_Direction - motion.euler_angle[0])*1 
                    if rotation1 > 180 : rotation1 = rotation1 - 360
                    if rotation1 < -180 : rotation1 = rotation1 + 360
                    if rotation1 > 12 : rotation1 = 12
                    if rotation1 < -12 : rotation1 = -12
                    
                motion.walk_Cycle(stepLength, sideLength,rotation1,cycle,cycleNumber)
            motion.walk_Final_Pose()
        motion.turn_To_Course(u2)
    #motion.kick(LEFT)
    motion.sim_Progress(1)
    motion.sim_Stop()
    #motion.print_Diagnostics()
    motion.sim_Disable()


