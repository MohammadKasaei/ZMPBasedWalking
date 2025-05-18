#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 21:56:03 2020

@author: mohammad
"""

import matplotlib.pyplot as plt


from walkEngine.FootStepPlanner import Foot
from walkEngine.FootStepPlanner import FootStepPlanner as FP
from walkEngine.ZMPGenerator import ZMPGenerator as ZG
from walkEngine.COMGenerator import COMGenerator as CG
from walkEngine.FeetGenerator import FootPositionGenerator as FG

# from FootStepPlanner as FP
# from test2 import ZMPGenerator as ZG
# from test2 import COMGenerator as CG
# from test2 import FootPositionGenerator as FG
# from test2 import Foot

class GenerateWalkingTrajectories:
   
    def __init__(
        self,
        FR0X, FR0Y, FL0X, FL0Y,
        step_x = 0.0, step_y= 0.0, swing_step_z= 0.0,
        step_time=0.5, sampling_time = 0.05,
        first_step_is_right = 0,
        distance_between_feet = 0,
        z_leg = 0.39,
        z_offset=0.0,
        num_steps=3,
        ds_ratio=0.,
        foot_heel_toe=0,
        smooth_height_index=0,
        com_height_amp=0.0
    ):
        
        self.init_walking( FR0X, FR0Y, FL0X, FL0Y,
                        step_x, step_y, swing_step_z,
                        step_time, sampling_time,
                        first_step_is_right,
                        distance_between_feet,
                        z_leg,
                        z_offset,
                        num_steps,
                        ds_ratio,
                        foot_heel_toe,
                        smooth_height_index,
                        com_height_amp)
       
    def init_walking(
        self,
        FR0X, FR0Y, FL0X, FL0Y,
        step_x = 0.0, step_y= 0.0, swing_step_z= 0.0,
        step_time=0.5, sampling_time = 0.05,
        first_step_is_right = 0,
        distance_between_feet = 0.05,
        z_leg = 0.39,
        z_offset=0.0,
        num_steps=5,
        ds_ratio=0.0,
        foot_heel_toe=0,
        smooth_height_index=0,
        com_height_amp=0.0
    ):
        # Initial feet positions
        self.FR0X = FR0X
        self.FR0Y = FR0Y
        self.FL0X = FL0X
        self.FL0Y = FL0Y

        # CoM start position
        self.com_x0 = (FR0X + FL0X) / 2
        self.com_y0 = (FR0Y + FL0Y) / 2

        # Trajectory generation parameters
        self.step_x = step_x
        self.step_y = step_y
        self.step_z = swing_step_z
        self.step_time = step_time
        self.sampling_time = sampling_time
        self.first_step_is_right = first_step_is_right
        self.DS_ratio = ds_ratio
        self.foot_heel_Toe = foot_heel_toe
        self.smooth_height_index = smooth_height_index
        self.number_of_step = num_steps
        self.distance_between_feet = distance_between_feet
        self.com_height_amp = com_height_amp
        self.z0 = 0.5
        self.z_leg = z_leg + z_offset

        # Double and single support times
        self.DS_time = self.DS_ratio * self.step_time
        self.SS_time = self.step_time - self.DS_time

        # Output trajectories
        self.right_leg_x = []
        self.right_leg_y = []
        self.right_leg_z = []

        self.left_leg_x = []
        self.left_leg_y = []
        self.left_leg_z = []
    
    def generate(self):
        
        planner = FP(fr0 = (self.FR0X, self.FR0Y),
                     fl0 =(self.FL0X , self.FL0Y))
        planner.plan_steps(number_of_steps=self.number_of_step,
                        step_x=self.step_x, 
                        step_y=self.step_y,
                        first_step_is_right=self.first_step_is_right)
 
        support_x = [f.x for f in planner.support_positions]
        support_y = [f.y for f in planner.support_positions]

        zmp_gen = ZG()
        zmp_gen.generate(
            support_pos_x=support_x,
            support_pos_y=support_y,
            foot_heel_toe=0.0,
            step_time=self.step_time,
            ds_ratio=0.,
            sampling_time=self.sampling_time,
        )
    
    
        com_gen = CG()
        com_gen.generate(
            com_x0=(self.FR0X+self.FL0X)/2, com_y0=(self.FR0Y+self.FL0Y)/2,
            support_pos_x=support_x, support_pos_y=support_y,
            zmp_x=zmp_gen.zmp_x, zmp_y=zmp_gen.zmp_y,
            step_time=self.step_time, sampling_time=self.sampling_time,
            number_of_steps=self.number_of_step, ds_ratio=0.,
            com_height_amp=0.0, z_0=self.z0
        )

        paths = planner.get_paths()
        right = [Foot(x, y, 0.0) for x, y in paths["right_foot"]]
        left = [Foot(x, y, 0.0) for x, y in paths["left_foot"]]
        support = [Foot(x, y, 0.0) for x, y in paths["support"]]

        foot_gen = FG(
            right_foot_steps=right,
            left_foot_steps=left,
            support_steps=support,
            step_time=self.step_time,
            sampling_time=self.sampling_time,
            foot_clearance=self.step_z
        )
        foot_gen.generate()
        
        rx, ry, rz = zip(*foot_gen.right_foot_traj)
        lx, ly, lz = zip(*foot_gen.left_foot_traj)


        self.right_leg_x = []
        self.right_leg_y = []
        self.right_leg_z = []
        
        self.left_leg_x = []
        self.left_leg_y = []
        self.left_leg_z = []
        
        
        # Relative trajectories
        for i in range(len(com_gen.com_x) - 1):
            self.right_leg_x.append(rx[i] - com_gen.com_x[i])
            self.right_leg_y.append(ry[i] - com_gen.com_y[i])
            self.right_leg_z.append(self.z_leg    - rz[i] )

            self.left_leg_x.append(lx[i] - com_gen.com_x[i])
            self.left_leg_y.append(ly[i] - com_gen.com_y[i] )
            self.left_leg_z.append(self.z_leg   - lz[i] )
            
            
        