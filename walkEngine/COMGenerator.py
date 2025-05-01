#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 14:51:52 2020

@author: mohammad
"""

import matplotlib.pyplot as plt
import numpy
import math


class COMGenerator:
  
    def __init__(self):
        self.com_x = []
        self.com_y = []
        self.com_z = []
        
        
    def generate(self,com_x0,com_y0,support_pos_x,support_pos_y,zmp_x,zmp_y,step_time,sampling_time,number_of_step,DS_ratio,com_height_amp,z_0):
            DS_time = (DS_ratio * step_time)
           
            com_x = []
            com_y = []
            com_z = []
            
            
            gravity = 9.81
            
            z_c = z_0
            ddot_zc = 0
            a = (gravity + ddot_zc) / z_c
            w = math.sqrt(a)
            
            i=0
            
            A = com_height_amp
            
            for GlobalTime in numpy.arange(0,step_time*(number_of_step-1),sampling_time):
                t=GlobalTime
                com_z.append(z_c)
                w = math.sqrt(z_c/(gravity))
                   
                #%%%%%%%%%%%%%%%%%% Search for Initial Condition
                k = number_of_step+1
                while( k>0):    
                    if (t <= (k*step_time) - DS_time/2 and t >= ((k-1)*step_time) -DS_time/2):
                        break
                    k=k-1
                #print("k is :"+ str(k)+"\n")
                #%*****************************************************************************************************
                if(k>1):
                    t0=(k-1)*step_time - DS_time/2
                    tf=k*step_time - DS_time/2
                else:
                    t0=0
                    tf=k*step_time - DS_time/2    
                
                #%*****************************************************************************************************
                z=zmp_y[i]
                if (i<len(zmp_y)):
                    i=i+1;           
                
                if (t0==0):
                 y0 = com_y0
                 yf = (support_pos_y[k-1] + support_pos_y[k])/ 2           
                else:
                 y0 = (support_pos_y[k-2] + support_pos_y[k-1])/ 2                   
                 yf = (support_pos_y[k-1] + support_pos_y[k])/ 2                   

                y = z + (1/math.sinh((t0 - tf)/w))*((yf - z)*math.sinh((t0 - t)/w) + (-y0 + z)*math.sinh((tf - t)/w))
                com_y.append(y)
            
                #%*****************************************************************************************************
                z=zmp_x[i]
            
                if (t0==0):
                     x0 = com_x0
                     xf = (support_pos_x[k-1] + support_pos_x[k])/ 2                     
                else:
                     x0 = (support_pos_x[k-2] + support_pos_x[k-1])/ 2                   
                     xf = (support_pos_x[k-1] + support_pos_x[k])/ 2                 
                
            
                x = z + (1/math.sinh((t0 - tf)/w))*((xf - z)*math.sinh((t0 - t)/w) + (-x0 + z)*math.sinh((tf - t)/w))
                com_x.append(x);    
            
            self.com_x = com_x
            self.com_y = com_y
            self.com_z = com_z
    
    