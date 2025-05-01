import pybullet as p
import time
import math

from walkEngine.GenrateWalkingTrajectories import GenerateWalkingTrajectories
from walkEngine.RobotControl import RobotControl
from Keyboard import keyboardMonitoring


robot_control = RobotControl()
keyboard_monitoring = keyboardMonitoring()
keyboard_monitoring.start()


sampling_time= robot_control.sampling_time
foot_heel_toe=0
distance_between_feet = 0.18
z_leg = 0.40
number_of_step=5

step_x= 0.0
step_y=0.0
step_theta = 0.
step_z = 0.033
step_time= 0.3
stop_for_one_step =  1
new_step_time = step_time
first_step_is_right = 0


if (step_y<0.001 or step_theta>0.01):
    first_step_is_right = 1

if (first_step_is_right):
    FR0X,FR0Y  = step_x/2, -distance_between_feet/2
    FL0X, FL0Y = 0 , distance_between_feet/2 

else:
    FL0X, FL0Y = step_x/2 , distance_between_feet/2   
    FR0X, FR0Y = 0, -distance_between_feet/2
    
            
walking = GenerateWalkingTrajectories(FR0X,FR0Y,FL0X,FL0Y,
                                      step_x,step_y,step_z,
                                      step_time,sampling_time,
                                      first_step_is_right,
                                      z_leg,z_offset=0.0)
walking.generate()


t0 = 0
t_walk0 = 0
t_walk =0
g_time = 0


Startpush = 0
arms_joint_pos = []

while (not keyboard_monitoring.exit):       

    p.stepSimulation()
    g_time = g_time + sampling_time
    
    if (g_time-t_walk0 >= 2*step_time - sampling_time):
        t_walk0 = g_time
        if (keyboard_monitoring.new_command): 
            step_x = keyboard_monitoring.new_step_x
            step_y = keyboard_monitoring.new_step_y
            step_theta = keyboard_monitoring.new_step_theta
            step_time = keyboard_monitoring.new_step_time
            keyboard_monitoring.new_command = 0

        if (keyboard_monitoring.stop == 1):
            stop_for_one_step = 1
            
        elif(keyboard_monitoring.stop == 100): # reset
            p.resetSimulation()
            robot_control.init_robot()
            arms_joint_pos = []

            stop_for_one_step = 1
            keyboard_monitoring.stop = 1
        else:
            stop_for_one_step = 0

        if (step_y<0 or step_theta>0):
            if (first_step_is_right==0): # robot should stop
                stop_for_one_step =1
            first_step_is_right = 1
        else:
            if (first_step_is_right==1): # robot should stop
                stop_for_one_step =1
            first_step_is_right = 0
        
        if (first_step_is_right):
            FR0X = -step_x/2
            FR0Y = -distance_between_feet/2
            FL0X = 0
            FL0Y = distance_between_feet/2
        else:
            FL0X = -step_x/2    
            FL0Y = distance_between_feet/2
            FR0X = 0
            FR0Y = -distance_between_feet/2    
                    
        
        
        step_x_rotated = step_x * math.cos(math.radians(step_theta)) - step_y* math.sin(math.radians(step_theta))
        step_y_rotated = step_x * math.sin(math.radians(step_theta)) + step_y * math.cos(math.radians(step_theta))


        walking.init_walking(FR0X,FR0Y,FL0X,FL0Y,
                                step_x_rotated,step_y_rotated,step_z,
                                step_time,sampling_time,
                                first_step_is_right,z_leg,z_offset=0.0)
        walking.generate()  
        

    else:
        t_walk = g_time - t_walk0
        idx = int(math.floor(t_walk/sampling_time))
        
        if (stop_for_one_step == 1):
            
            right_foot_pos = [0, 0, robot_control.Zleg, 0]
            left_foot_pos = [0, 0, robot_control.Zleg, 0]
            legs_joint_pos = robot_control.update_joint_pos(right_foot_pos, left_foot_pos)
            arms_joint_pos = []
            
        elif (idx >= 0 and idx < len(walking.left_leg_x)):
            
            arm_amp = math.radians(5)
            arm_offset = 20 
            arms_joint_pos,right_theta,left_theta = robot_control.generate_arm_motions(first_step_is_right,
                                                                arm_amp,arm_offset,
                                                                step_theta,
                                                                t_walk,
                                                                step_time)
            
            waste_balance_ctrl_cmd, ankle_x_balance_ctrl_cmd, ankle_y_balance_ctrl_cmd =robot_control.balance_control(ori,CoP_L,CoP_R)
            
            right_foot_pos = [walking.right_leg_x[idx], walking.right_leg_y[idx]+distance_between_feet/2,walking.right_leg_z[idx],0*right_theta]
            left_foot_pos = [walking.left_leg_x[idx],walking.left_leg_y[idx]-distance_between_feet/2,walking.left_leg_z[idx],0*left_theta]
            legs_joint_pos = robot_control.update_joint_pos(right_foot_pos, left_foot_pos,ankle_x_balance_ctrl_cmd,ankle_y_balance_ctrl_cmd)
            

        robot_control.update_legs_motors(legs_joint_pos)
        if arms_joint_pos:
            robot_control.update_upper_body_motors(arms_joint_pos,waste_balance_ctrl_cmd)
        
        
    pos, ori, CoP_L, CoP_R = robot_control.update_sensors()    

    if (keyboard_monitoring.F != [0,0,0]):
        Startpush = g_time
        Force = keyboard_monitoring.F
        keyboard_monitoring.F = [0,0,0]

    if (g_time>Startpush and g_time<Startpush+keyboard_monitoring.Duration):
       print("Pushed at t =",str(g_time),"\n")   
       p.applyExternalForce(robot_control.Coman_ID, -1, Force, pos, p.WORLD_FRAME)     
    
    time.sleep(sampling_time)    


p.disconnect()
keyboard_monitoring.inputThread.stop()

