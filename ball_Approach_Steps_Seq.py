
# Module is designed by Vladimir Tkach

import math
#from class_Motion import *
from ball_Approach_calc import ball_Approach_Calc



def steps(x1,y1,u1,x2,y2,u2): #returns list of steps [[step forward/back,step left/right,degrees of rotation,the number of steps]] 
    stepLength = 64
    first_step_yield = 46
    cycle_step_yield = 78.2
    rotation_angle = 12
    rotation_angle_yield = 13.2
    a=[]
    b=[]
    Dx=x2-x1 #calculating the x length
    Dy=y2-y1 #calculating the y length
    S=(Dx**2+Dy**2)**0.5 #calculating the distance to destination
    n=(S-first_step_yield)/cycle_step_yield+1 #calculating the number of steps
    n1=math.floor(n) #calculating the number of full steps
    Ds=S-(first_step_yield+cycle_step_yield*(n1-1)) #calculating the remains of the distance
    steplenght_T=Ds/(first_step_yield+cycle_step_yield) * stepLength #calculating the length of the little steps
    if Dx == 0:
        if Dy > 0: uB = 90 
        if Dy < 0: uB = -90
        if Dy == 0: uB = u1
    else:
        uB=math.atan(Dy/Dx)  #calculating B angle in radians
        uB=uB*180/math.pi #makeing degrees from radians
    if Dx < 0: uB = uB + 180
    U=uB-u1 #calculating the rotation angle before starting to go
    if U > 180 : U = U - 360
    print ('U =', U)
    U2=u2-uB #calculating the rotation angle when the robot is on the destinatiob point
    if U2 > 180 : U2 = U2 - 360
    if U2 < -180 : U2 = U2 + 360
    print(U2)
    Rotates=int(U//rotation_angle_yield) #calculating the full number of rotates (45 degrees)
    if abs(Rotates)>0:
        if Rotates>0:
            b=[0,0,rotation_angle,Rotates] 
        else:
            b=[0,0,-rotation_angle,-Rotates]
        a.append(b) #add the number of full rotates and degrees of rotation
    Rotates1=U % rotation_angle_yield #calculating the number of remained degrees
    if abs(Rotates1)>0:
        b=[0,0,Rotates1,1] 
        #a.append(b) #add the number of remained rotates and degrees of rotation

    
    if S!=0:
        b=[64,0,0,n1]
        a.append(b)
        b=[steplenght_T,0,0,2]
        a.append(b)
    
    Rotates=int(U2//rotation_angle_yield) #calculating the angle at which the robot must look when he is near the ball
    if abs(Rotates)>0:
        if Rotates>0:
            b=[0,0,rotation_angle,Rotates] #adds the number of rotations
        else:
            b=[0,0,-rotation_angle,-Rotates] #adds the number of rotations
        a.append(b)
    Rotates1=U2 % rotation_angle_yield #calculating the remain degrees to finish the robot rotation
    if abs(Rotates1)>0:
        b=[0,0,Rotates1,1] #adds the degrees of the final rotation
        #a.append(b)
    return a, uB         #returns a-list value and walk direction

def ball_Approach(motion, ball_X_m, ball_Y_m, player_X_m, player_Y_m):
    x1 = player_X_m
    y1 = player_Y_m
    #u1 = motion.euler_angle[0]
    #print('u1 = ', u1)
    destination = ball_Approach_Calc(ball_X_m, ball_Y_m, player_X_m, player_Y_m, 3.6, 2.6 )
    for stop_point in range (len(destination)):
        #x1, y1, z1 = motion.Dummy_HData[len(motion.Dummy_HData)-1]
        if stop_point > 0: 
            x1,y1,u1 = destination[stop_point-1]
        x1 = 1000* x1
        y1 = 1000* y1
        #u1 = motion.euler_angle[0]
        returnCode, position21= motion.vrep.simxGetJointPosition(motion.clientID, motion.jointHandle[21], motion.vrep.simx_opmode_blocking)
        u1 = motion.euler_angle[0] - position21*180/math.pi
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
                    returnCode, position21= motion.vrep.simxGetJointPosition(motion.clientID, motion.jointHandle[21], motion.vrep.simx_opmode_blocking)

                    rotation1 = (walk_Direction - motion.euler_angle[0] + position21*180/math.pi)*1 
                    if rotation1 > 180 : rotation1 = rotation1 - 360
                    if rotation1 < -180 : rotation1 = rotation1 + 360
                    if rotation1 > 12 : rotation1 = 12
                    if rotation1 < -12 : rotation1 = -12
                    
                motion.walk_Cycle(stepLength, sideLength,rotation1,cycle,cycleNumber)
            motion.walk_Final_Pose()
        motion.turn_To_Course(u2)


if __name__=="__main__":
    SIMULATION = 1
    motion = Motion(SIMULATION)
    motion.slowTime = 0.0
    motion.sim_Enable()
    motion.sim_Start()
    motion.Vision_Sensor_Display_On = False
    x1, y1, z1 = motion.Dummy_HData[len(motion.Dummy_HData)-1]
    ball_X_m, ball_Y_m, ball_Z_m = motion.Ballposition
    player_X_m, player_Y_m = x1, y1
    ball_Approach(motion, ball_X_m, ball_Y_m, player_X_m, player_Y_m)

    #motion.kick(LEFT)
    motion.sim_Progress(1)
    motion.sim_Stop()
    #motion.print_Diagnostics()
    motion.sim_Disable()