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


if __name__ == "__main__":

    planner = FootStepPlanner(fl0 = (-0.,0),
                            fr0 =(-0.015,0.1))
    planner.plan_steps(number_of_steps=6,
                    step_x=0.03, 
                    step_y=0.0,
                    first_step_is_right=True)
    planner.plot_steps()

    support_x = [f.x for f in planner.support_positions]
    support_y = [f.y for f in planner.support_positions]

    zmp_gen = ZMPGenerator()
    zmp_gen.generate(
        support_pos_x=support_x,
        support_pos_y=support_y,
        foot_heel_toe=0.0,
        step_time=0.35,
        ds_ratio=0.,
        sampling_time=1/240.,
    )
    zmp_gen.plot_zmp()

    com_gen = COMGenerator()
    com_gen.generate(
        com_x0=support_x[0], com_y0=support_y[0],
        support_pos_x=support_x, support_pos_y=support_y,
        zmp_x=zmp_gen.zmp_x, zmp_y=zmp_gen.zmp_y,
        step_time=0.35, sampling_time=1/240.,
        number_of_steps=6, ds_ratio=0.,
        com_height_amp=0.0, z_0=0.8
    )
    com_gen.plot()

    paths = planner.get_paths()
    right = [Foot(x, y, 0.0) for x, y in paths["right_foot"]]
    left = [Foot(x, y, 0.0) for x, y in paths["left_foot"]]
    support = [Foot(x, y, 0.0) for x, y in paths["support"]]

    foot_gen = FootPositionGenerator(
        right_foot_steps=right,
        left_foot_steps=left,
        support_steps=support,
        step_time=0.35,
        sampling_time=1/240.,
        foot_clearance=0.05
    )
    foot_gen.generate()
    foot_gen.plot()
