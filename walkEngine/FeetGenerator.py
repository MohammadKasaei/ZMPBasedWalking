#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 14:51:52 2020

@author: mohammad
"""
import numpy
import math
import statistics


class FeetGenerator:

    def __init__(self):
        self.right_x  = []
        self.right_y  = []
        self.right_z  = []
    
        self.left_x  = []
        self.left_y  = []
        self.left_z  = []

        
    def generate(self,step_x,step_y,step_z,step_time,sampling_time,number_of_step,zmp_x,zmp_y,FR0X,FR0Y,FL0X,FL0Y,first_step_is_right,smooth_height_index):

            t = 0
            index =0
            right_is_moving = first_step_is_right
            
            tf_simulation = (number_of_step)*step_time    
            for global_time in numpy.arange(0,tf_simulation,sampling_time):
                if (global_time==0 or t > step_time-(sampling_time) ):           
                    if (index==0):
                        right_is_moving = first_step_is_right
                        if (first_step_is_right):
                            x_p = FR0X 
                            y_p = FR0Y
                        else:
                            x_p = FL0X 
                            y_p = FL0Y
                        
                    else:
                        if (right_is_moving):
                            right_is_moving = 0
                        else:
                            right_is_moving = 1
                        
                        x_p = ZMP_X
                        y_p = ZMP_Y
                    
                    l = int(index*round(step_time/sampling_time))
                    ZMP_X = statistics.mean(zmp_x[ l:l+int(round(0.1*step_time/sampling_time))]); 
                    ZMP_Y = statistics.mean(zmp_y[l:l+int(round(0.1*step_time/sampling_time))]); 
                    
                    index=index+1
                    t = 0    
                else:
                    t = t + sampling_time
                
                
                #ZMP = [ZMPX ,ZMPY , 0];    
                zmp_z = 0
                t0 = 0.
                if (smooth_height_index>1 and index<smooth_height_index):
                    if (t<t0):
                        swing_x = x_p+(step_x*(t/(step_time)))
                        swing_y = y_p+(step_y*(t/step_time))
                        swing_z = 0*(index/smooth_height_index)*step_z*(math.sin(math.pi*t/step_time))        
                    else:
                        swing_x = x_p+(step_x*(t/(step_time)))
                        swing_y = y_p+(step_y*(t/step_time))
                        swing_z = (index/smooth_height_index)*step_z*(math.sin(math.pi*(t-t0)/(step_time-t0)))        
                else:
                    if (t<t0):            
                        swing_x = x_p+(step_x*(t/(step_time)))
                        swing_y = y_p+(step_y*(t/step_time))
                        swing_z = 0        
                    else:
                         swing_x = x_p+(step_x*(t/(step_time)))
                         swing_y = y_p+(step_y*(t/step_time))
                         swing_z = step_z*(math.sin(math.pi*(t-t0)/(step_time-t0)));
                    
                
                
                
                #swing = [swing; SWING ];
                
                if (right_is_moving):    
                   self.right_x.append(swing_x)
                   self.right_y.append(swing_y)
                   self.right_z.append(swing_z)
                   
                   self.left_x.append(ZMP_X)
                   self.left_y.append(ZMP_Y)
                   self.left_z.append(zmp_z)
                else:
                   self.left_x.append(swing_x)
                   self.left_y.append(swing_y)
                   self.left_z.append(swing_z)
                   
                   self.right_x.append(ZMP_X)
                   self.right_y.append(ZMP_Y)
                   self.right_z.append(zmp_z)
            
            
            
#
#
#SamplingTime=0.01
#StepTime=1
#
#FootHeelToe=0
#DistanceBetweenFeet=0.2
#
#NumberOfStep=10
#StepX=0.1
#StepY=0.1
#SwingStepZ =0.04
#
#FirstStepIsRight=1
#DSRatio=0.
#CoMHeightAmp = 0
#Z0 = 1
#NewZMPGenerator=0
#SmoothHeightIndex=0
#
###
#FR0X=-StepX/2
#FR0Y=DistanceBetweenFeet/2
#FL0X=0
#FL0Y=-DistanceBetweenFeet/2
#
#COM_X0 = (FR0X+FL0X)/2;
#COM_Y0 = (FR0Y+FL0Y)/2;
#
##%% FootStep planner
#FP = FootStepPlanner(FR0X,FR0Y,FL0X,FL0Y,DistanceBetweenFeet)
#FP.PlanSteps(NumberOfStep,StepX,StepY,FirstStepIsRight)
#
##%% ZMP Generator     
#ZMP = ZMPGenerator()
#ZMP.Generate(FP.SupportPositionsX,FP.SupportPositionsY,FootHeelToe,StepTime,DSRatio,SamplingTime)
#
##%% COM Generator
#COM = COMGenerator()
#COM.Generate(COM_X0,COM_Y0,FP.SupportPositionsX,FP.SupportPositionsY,ZMP.zmp_x,ZMP.zmp_y,StepTime,SamplingTime,NumberOfStep,DSRatio,CoMHeightAmp,Z0)
#
#
##%% FeetGenerator
#FT = FeetGenerator()
#FT.Generate(StepX,StepY,SwingStepZ,StepTime,SamplingTime,NumberOfStep,ZMP.zmp_x,ZMP.zmp_y,FR0X,FR0Y,FL0X,FL0Y,FirstStepIsRight,SmoothHeightIndex)
##%% plotting
#time = numpy.arange(0,NumberOfStep*StepTime,SamplingTime)
#time2 = numpy.arange(0,(NumberOfStep-1)*StepTime,SamplingTime)
#
#fig = plt.figure()
#plt.plot(FP.SupportPositionsX,FP.SupportPositionsY,'o--',label='FootSteps')
#plt.legend()
#plt.grid(True)
#
#fig = plt.figure()
#ax1 = fig.add_subplot(211)
#ax1.plot(time,ZMP.zmp_x,linestyle='dashed',color = 'darkgreen',linewidth=2,label='ZMP_X')
#ax1.hold(True)
#ax1.grid(True)
#ax1.plot(time2,COM.COM_X,color = 'red',linewidth=1,label='COM_X')
#ax1.legend()      
#
#ax2 = fig.add_subplot(212)
#ax2.plot(time,ZMP.zmp_y,linestyle='dashed',color = 'darkgreen',linewidth=2,label='ZMP_Y')
#ax2.hold(True)
#ax2.grid(True)
#ax2.plot(time2,COM.COM_Y,color = 'red',linewidth=1,label='COM_Y')
#ax2.legend()
#
#fig = plt.figure()
#plt.plot(time,FT.LeftX)
#plt.hold(True)
#plt.grid(True)
#plt.plot(time,FT.RightX)
#
#fig = plt.figure()
#plt.plot(time,FT.LeftY)
#plt.hold(True)
#plt.grid(True)
#plt.plot(time,FT.RightY)
#
#fig = plt.figure()
#plt.plot(time,FT.LeftZ)
#plt.hold(True)
#plt.grid(True)
#plt.plot(time,FT.RightZ)
