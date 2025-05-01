#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 14:07:46 2020

@author: mohammad
"""

import matplotlib.pyplot as plt
import numpy
import math


class ZMPGenerator:

    def __init__(self):
        self.zmp_x = []
        self.zmp_y = []
        
    def generate(self,support_pos_x,support_pos_y,foot_heel_toe,step_time,DS_ratio,sampling_time):
                
        number_of_Step = len(support_pos_y)-2
        
        DS_time = (DS_ratio * step_time)
        SS_time =  step_time - DS_time
        
        zmp_x = []
        zmp_y = []
        
        index=0
        t=0
        for global_time in numpy.arange(0,step_time*(number_of_Step),sampling_time):
            if ( t <= SS_time): #SS        
                if (index==1):                        
                    ex0 = support_pos_x[index] +(foot_heel_toe*t/SS_time)           
                else:
                    ex0 = support_pos_x[index] - (foot_heel_toe)+(2*foot_heel_toe*t/SS_time)           
                
                zmp_x.append(ex0)
                zmp_y.append(support_pos_y[index]);        
                LastZmpX =  ex0
                LastZmpY =  support_pos_y[index];    
            else: #%%DS
                dx =( support_pos_x[1+math.floor(global_time/step_time)] - support_pos_x[math.floor(global_time/step_time)]) - (2*foot_heel_toe)
                
                dy = support_pos_y[1+math.floor(global_time/step_time)] - support_pos_y[math.floor(global_time/step_time)]
                
                zmp_x.append(LastZmpX+(t-SS_time)*dx/(DS_time))
                zmp_y.append(LastZmpY+(t-SS_time)*dy/(DS_time))    
        
            if (t >= step_time-sampling_time):    
                index=index+1
                t=0
            else:
                t = t+sampling_time          
            
        
        self.zmp_x = zmp_x
        self.zmp_y = zmp_y


