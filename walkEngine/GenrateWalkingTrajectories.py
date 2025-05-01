#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 21:56:03 2020

@author: mohammad
"""

import matplotlib.pyplot as plt

from walkEngine.FootStepPlanner import FootStepPlanner
from walkEngine.ZMPGenerator import ZMPGenerator
from walkEngine.COMGenerator import COMGenerator
from walkEngine.FeetGenerator import FeetGenerator


class GenerateWalkingTrajectories:
   
    def __init__(
        self,
        FR0X, FR0Y, FL0X, FL0Y,
        step_x = 0.0, step_y= 0.0, swing_step_z= 0.0,
        step_time=0.5, sampling_time = 0.05,
        first_step_is_right = 0,
        z_leg = 0.39,
        z_offset=0.0,
        num_steps=3,
        ds_ratio=0.0,
        foot_heel_toe=0,
        smooth_height_index=0,
        com_height_amp=0.0
    ):
        
        self.init_walking( FR0X, FR0Y, FL0X, FL0Y,
                        step_x, step_y, swing_step_z,
                        step_time, sampling_time,
                        first_step_is_right,
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
        z_leg = 0.39,
        z_offset=0.0,
        num_steps=3,
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
        self.distance_between_feet = 0.17
        self.com_height_amp = com_height_amp
        self.z0 = 0.8
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
        # Step Planning
        fp = FootStepPlanner(self.FR0X, self.FR0Y, self.FL0X, self.FL0Y, self.distance_between_feet)
        fp.plan_steps(self.number_of_step, self.step_x, self.step_y, self.first_step_is_right)
        
        # plt.figure()
        # plt.plot(fp.support_pos_x, fp.support_pos_y,'ro')
        # plt.show()

        # ZMP Generation
        zmp = ZMPGenerator()
        zmp.generate(
            fp.support_pos_x, fp.support_pos_y,
            self.foot_heel_Toe, self.step_time,
            self.DS_ratio, self.sampling_time
        )

        # CoM Generation
        com = COMGenerator()
        com.generate(
            self.com_x0, self.com_y0,
            fp.support_pos_x, fp.support_pos_y,
            zmp.zmp_x, zmp.zmp_y,
            self.step_time, self.sampling_time,
            self.number_of_step, self.DS_ratio,
            self.com_height_amp, self.z0
        )

        # Feet Trajectories
        ft = FeetGenerator()
        ft.generate(
            self.step_x, self.step_y, self.step_z,
            self.step_time, self.sampling_time,
            self.number_of_step, zmp.zmp_x, zmp.zmp_y,
            self.FR0X, self.FR0Y, self.FL0X, self.FL0Y,
            self.first_step_is_right, self.smooth_height_index
        )

        # Relative trajectories
        for i in range(len(com.com_x) - 1):
            self.right_leg_x.append(ft.right_x[i] - com.com_x[i])
            self.right_leg_y.append(ft.right_y[i] - com.com_y[i])
            self.right_leg_z.append(ft.right_z[i] - self.z_leg)

            self.left_leg_x.append(ft.left_x[i] - com.com_x[i])
            self.left_leg_y.append(ft.left_y[i] - com.com_y[i])
            self.left_leg_z.append(ft.left_z[i] - self.z_leg)

