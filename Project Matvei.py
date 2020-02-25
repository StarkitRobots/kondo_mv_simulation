import class_Motion as mt
from ball_Approach_Steps_Seq import *
import math, random

def goto_Center():#Function for reterning to center position
    print('Function for reterning to center position')
    x1, y1, z1 = motion.Dummy_HData[len(motion.Dummy_HData)-1]
    ball_X_m, ball_Y_m = -1.6, 0
    player_X_m, player_Y_m = x1, y1
    ball_Approach(motion, ball_X_m, ball_Y_m, player_X_m, player_Y_m)

def find_Ball(motion):
    #dist = float(input('dist ='))
    #napravl = float(input('napravl ='))
    a, napravl, distance = motion.seek_Ball_In_Pose(fast_Reaction_On=True)
    dist = distance/1000
    print ( 'dist = ', dist, 'napravl =', napravl)
    return dist, napravl
def ball_Speed_Dangerous():
    pass
def fall_to_Defence():
    print('fall to defence')
def get_Up_from_defence():
    print('up from defence')
def scenario_A1(motion, dist, napravl):#The robot knock out the ball to the side of the enemy
    print('The robot knock out the ball to the side of the enemy')
    napravl_rad = math.radians(napravl)
    ball_X_m = dist* math.cos(napravl_rad)-1.8 
    ball_Y_m = dist* math.sin(napravl_rad)
    player_X_m, player_Y_m = -1.8 , 0
    ball_Approach(motion, ball_X_m, ball_Y_m, player_X_m, player_Y_m)
    for i in range(5):
        motion.turn_To_Course(0)
        a, napravl, dist = motion.seek_Ball_In_Pose(fast_Reaction_On = True)
        if dist > 500: break
        else:
            #n=(dist-80-46)/78.2+1 #calculating the number of steps
            #n1=math.floor(n) #calculating the number of full steps
            n = int(math.floor((dist-90-46)/78.2)+1)+1 #calculating the number of potential full steps forward
            displacement = dist*math.sin(math.radians(napravl))
            m = int(math.floor(abs(abs(displacement)-53.4)/22.5)+1)
            if n >= m : 
                if napravl > 0: sideLength = (displacement - 53.4)/n*20/22.5
                else:           sideLength = (displacement + 53.4)/n*20/22.5
                stepLength = (dist-90)/(46+78.2*(n-1))*64
                cycleNumber = n
            else:
                if napravl > 0: sideLength = (displacement - 53.4)/m*20/22.5
                else:           sideLength = (displacement + 53.4)/m*20/22.5
                stepLength = (dist-90)/(46+78.2*(m-1))*64
                cycleNumber = m
            #print('displacement =', displacement )
            #if napravl > 0: sideLength = -(displacement - 53.4)/(n1+1)
            #else:           sideLength = -(displacement + 53.4)/(n1+1)
            motion.head_Up()
            motion.walk_Initial_Pose()
            #stepLength, sideLength, rotation, cycle, cycleNumber = 64, sideLength , 0 , 0 , n1
            rotation = 0
            for cycle in range(cycleNumber): motion.walk_Cycle(stepLength, sideLength,rotation,cycle,cycleNumber)
            motion.walk_Final_Pose()
            motion.head_Return()
            a, napravl, dist = motion.seek_Ball_In_Pose(fast_Reaction_On = True)
            #motion.head_Up()
            if napravl > 0: motion.kick(first_Leg_Is_Right_Leg=False)
            else: motion.kick(first_Leg_Is_Right_Leg=True)
            #motion.head_Return()
    #motion.head_Up()
    goto_Center()
    #motion.head_Return()
    
        
def scenario_A2(motion, dist, napravl):#The robot knock out the ball to the side of the enemy
    print('The robot knock out the ball to the side of the enemy')
    scenario_A1(motion, dist, napravl)

def scenario_A3(motion, dist, napravl):#The robot knock out the ball to the side of the enemy
    print('The robot knock out the ball to the side of the enemy')
    scenario_A1(motion, dist, napravl)

def scenario_A4(motion, dist, napravl):#The robot knock out the ball to the side of the enemy
    print('The robot knock out the ball to the side of the enemy')
    scenario_A1(motion, dist, napravl)

def scenario_B1():#the robot moves to the left and stands on the same axis as the ball and the opponents' goal
    print('the robot moves to the left 2 steps')
    motion.first_Leg_Is_Right_Leg = True
    motion.walk_Initial_Pose()
    stepLength, sideLength, rotation, cycle, cycleNumber = 0, -20 , 0 , 0 , 4
    for cycle in range(cycleNumber): motion.walk_Cycle(stepLength, sideLength,rotation,cycle,cycleNumber)
    motion.walk_Final_Pose()
    motion.turn_To_Course(0)

def scenario_B2():#the robot moves to the left and stands on the same axis as the ball and the opponents' goal
    print('the robot moves to the left 2 steps')
    scenario_B1()

def scenario_B3():#the robot moves to the right and stands on the same axis as the ball and the opponents' goal
    print('the robot moves to the right 2 steps')
    motion.first_Leg_Is_Right_Leg = True
    motion.walk_Initial_Pose()
    stepLength, sideLength, rotation, cycle, cycleNumber = 0, 20 , 0 , 0 , 4
    for cycle in range(cycleNumber): motion.walk_Cycle(stepLength, sideLength,rotation,cycle,cycleNumber)
    motion.walk_Final_Pose()
    motion.turn_To_Course(0)

def scenario_B4():#the robot moves to the right and stands on the same axis as the ball and the opponents' goal
    print('the robot moves to the right 2 steps')
    scenario_B3()
 

if __name__=="__main__":
    
    SIMULATION = 1
    motion = mt.Motion(SIMULATION)
    motion.sim_Enable()
    motion.sim_Start()

    falling_Flag = 0
    while (True):
        Ballposition = motion.Ballposition
        Ballposition[0] = random.uniform(-1.7, 0)
        Ballposition[1] = random.uniform(-1.3, 1.3)
        if input('continue? (y/n)') == 'n': break
        #returnCode = motion.vrep.simxSetObjectPosition(motion.clientID, motion.BallHandle , motion.ResizableFloorHandle,Ballposition, motion.vrep.simx_opmode_oneshot)
        dist = -1.0
        if falling_Flag == 1:
            goto_Center()
            falling_Flag = 0
        while(dist < 0):
            dist,napravl = find_Ball(motion)
            if ball_Speed_Dangerous():
                fall_to_Defence()
                time.sleep(3.0)
                get_Up_from_defence()
            motion.head_Up()
            if (dist <= 0.9 and 0 <= napravl <= 45): scenario_A1(motion, dist, napravl)
            if (dist <= 0.9 and 45 < napravl <= 90): scenario_A2(motion, dist, napravl)
            if (dist <= 0.9 and 0 >= napravl >= -45): scenario_A3(motion, dist, napravl)
            if (dist <= 0.9 and -45 > napravl >= -90): scenario_A4(motion, dist, napravl)
            if ((1.8 >= dist > 0.9) and (10 <= napravl <= 45)): scenario_B1()
            if ((1.8 >= dist > 0.9) and (45 < napravl <= 90)): scenario_B2()
            if ((1.8 >= dist > 0.9) and (-10 >= napravl >= -45)): scenario_B3()
            if ((1.8 >= dist > 0.9) and (-45 > napravl >= -90)): scenario_B4()
            motion.head_Return()


    motion.sim_Progress(2)
    motion.sim_Stop()
    #motion.print_Diagnostics()
    motion.sim_Disable()
    

    
        
