
from class_Motion import Motion
import time, math
#import pyb

SIMULATION = 1
motion = Motion(SIMULATION)
motion.slowTime = 0.0
if SIMULATION ==0: motion.slowTime = 0.1
if SIMULATION == 1 or SIMULATION == 0:
    motion.simThreadCycleInMs = 20
    motion.Vision_Sensor_Display_On = False
    motion.sim_Enable()
    motion.sim_Start()
else:
    motion.push_Button()
    pyb.delay(2000)

#motion.first_Leg_Is_Right_Leg = True
#motion.walk_Initial_Pose()
#stepLength, sideLength, rotation, cycle, cycleNumber = 0, 20 , 0 , 0 , 20
#for cycle in range(cycleNumber):
#    rotation1 = rotation
#    if rotation == 0: 
#        returnCode, position21= motion.vrep.simxGetJointPosition(motion.clientID, motion.jointHandle[21], motion.vrep.simx_opmode_blocking)
#        rotation1 = (0 - motion.euler_angle[0] + position21*180/math.pi)*1 
#        if rotation1 > 180 : rotation1 = rotation1 - 360
#        if rotation1 < -180 : rotation1 = rotation1 + 360
#        if rotation1 > 12 : rotation1 = 12
#        if rotation1 < -12 : rotation1 = -12
#    motion.walk_Cycle(stepLength, sideLength,rotation1,cycle,cycleNumber)
#motion.walk_Final_Pose()
#motion.turn_To_Course(0)
#for i in range(0,360,5):
#    motion.turn_To_Course(i)
#    print(i, '  motion.euler_angle =', motion.euler_angle)
#print(motion.seek_Ball(True))


if SIMULATION == 1 or SIMULATION == 0:
    motion.sim_Progress(1)
    motion.sim_Stop()
    motion.print_Diagnostics()
    motion.sim_Disable()
