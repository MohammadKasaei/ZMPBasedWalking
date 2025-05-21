import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple
import math
import numpy as np


from walkEngine.FootStepPlanner import Foot
from walkEngine.FootStepPlanner import FootStepPlanner
from walkEngine.ZMPGenerator import ZMPGenerator 
from walkEngine.COMGenerator import COMGenerator 
from walkEngine.FeetGenerator import FootPositionGenerator 
from walkEngine.DCMGenerator import DCMGenerator

if __name__ == "__main__":

    planner = FootStepPlanner(fl0 = (-0.,0),
                            fr0 =(-0.015,0.1))
    planner.plan_steps(number_of_steps=6,
                    step_x=0.03, 
                    step_y=0.05,
                    first_step_is_right=True)
    planner.plot_steps()

    support_x = [f.x for f in planner.support_positions]
    support_y = [f.y for f in planner.support_positions]

    zmp_gen = ZMPGenerator()
    zmp_gen.generate(
        support_pos_x=support_x,
        support_pos_y=support_y,
        foot_heel_toe=0.0,
        step_time=1,
        ds_ratio=0.,
        sampling_time=0.005,
    )
    zmp_gen.plot_zmp()

    com_gen = COMGenerator()
    com_gen.generate(
        com_x0=support_x[0], com_y0=support_y[0],
        support_pos_x=support_x, support_pos_y=support_y,
        zmp_x=zmp_gen.zmp_x, zmp_y=zmp_gen.zmp_y,
        step_time=1, sampling_time=0.005,
        number_of_steps=6, ds_ratio=0.,
        com_height_amp=0.0, z_0=0.8
    )
    com_gen.plot()
    
    dcm_gen = DCMGenerator(z_0=0.8, dt=0.005)
    dcm_x, dcm_y = dcm_gen.generate(com_gen.com_x,com_gen.com_y)
    

    paths = planner.get_paths()
    right = [Foot(x, y, 0.0) for x, y in paths["right_foot"]]
    left = [Foot(x, y, 0.0) for x, y in paths["left_foot"]]
    support = [Foot(x, y, 0.0) for x, y in paths["support"]]

    foot_gen = FootPositionGenerator(
        right_foot_steps=right,
        left_foot_steps=left,
        support_steps=support,
        step_time=1,
        sampling_time=0.005,
        foot_clearance=0.05
    )
    foot_gen.generate()
    
    rx, ry, rz = zip(*foot_gen.right_foot_traj)
    lx, ly, lz = zip(*foot_gen.left_foot_traj)

    tf = len(com_gen.com_x)
    t = np.linspace(0,tf*0.005,len(com_gen.com_x))
    
    fig, ax = plt.subplots(4, 1, figsize=(8, 12))
    ax[0].plot(t, rx[:tf], 'r-', label='Right Foot Trajectory')
    ax[0].plot(t, lx[:tf], 'r--', label='Left Foot Trajectory')
    ax[0].plot(t, zmp_gen.zmp_x[:tf], 'm-', label='ZMP - X')
    ax[0].plot(t, com_gen.com_x[:tf], 'c-', label='COM - X')
    ax[0].plot(t, dcm_x[:tf], 'k--', label='DCM - X')
    ax[0].set_ylabel("X [m]")
    
    
    ax[1].plot(t, ry[:tf], 'g-', label='Right Foot Trajectory')
    ax[1].plot(t, ly[:tf], 'g--', label='Left Foot Trajectory')
    ax[1].plot(t, zmp_gen.zmp_y[:tf], 'c-', label='ZMP-Y')
    ax[1].plot(t, com_gen.com_y[:tf], 'm-', label='COM - Y')
    ax[1].plot(t, dcm_y[:tf], 'y--', label='DCM - Y')
    ax[1].set_ylabel("Y [m]")
    
    ax[2].plot(t, rz[:tf], 'b-', label='Right Foot Trajectory')
    ax[2].plot(t, lz[:tf], 'r-', label='Left Foot Trajectory')
    ax[2].set_ylabel("Z [m]")
    
    ax[3].plot(rx,ry, 'b-', label='Right Foot Trajectory')
    ax[3].plot(lx,ly, 'r-', label='Left Foot Trajectory')
    ax[3].plot(support_x , support_y, 'm*', linewidth = 5, label='Support Feet')
        
    
    ax[3].plot(zmp_gen.zmp_x, zmp_gen.zmp_y,'g-', label='ZMP')
    ax[3].set_xlabel("X [m]")
    ax[3].set_ylabel("Y [m]")
    
    
    
    ax[0].legend()
    ax[1].legend()
    ax[2].legend()
    ax[3].legend()
    ax[0].grid()
    ax[1].grid()
    ax[2].grid()
    ax[3].grid()
    plt.show()
    
    
