from .FootStepPlanner import Foot
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple
import numpy as np


class ZMPGenerator:
    def __init__(self):
        self.zmp_x: List[float] = []
        self.zmp_y: List[float] = []

    def generate(
        self,
        support_pos_x: List[float],
        support_pos_y: List[float],
        foot_heel_toe: float,
        step_time: float,
        ds_ratio: float,
        sampling_time: float,
    ) -> None:
        """
        Generates a ZMP trajectory based on support foot positions.

        Parameters:
            support_pos_x (List[float]): X positions of support foot.
            support_pos_y (List[float]): Y positions of support foot.
            foot_heel_toe (float): Step extension toward heel/toe.
            step_time (float): Duration of one step (seconds).
            ds_ratio (float): Ratio of time in double support phase.
            sampling_time (float): Time interval for ZMP sampling (seconds).
        """

        if len(support_pos_x) != len(support_pos_y):
            raise ValueError("Support position lists must be the same length.")

        num_steps = len(support_pos_x) - 2
        ds_time = ds_ratio * step_time
        ss_time = step_time - ds_time

        zmp_x: List[float] = []
        zmp_y: List[float] = []

        index = 0
        local_time = 0.0

        for global_time in np.arange(0, step_time * num_steps, sampling_time):
            current_step = int(global_time // step_time)

            if local_time <= ss_time:
                # Single Support Phase
                if index == 1:
                    zmp_pos_x = support_pos_x[index] + (foot_heel_toe * local_time / ss_time)
                else:
                    zmp_pos_x = support_pos_x[index] - foot_heel_toe + (2 * foot_heel_toe * local_time / ss_time)

                zmp_pos_y = support_pos_y[index]

                last_zmp_x = zmp_pos_x
                last_zmp_y = zmp_pos_y
            else:
                # Double Support Phase
                dx = (support_pos_x[current_step + 1] - support_pos_x[current_step]) - 2 * foot_heel_toe
                dy = support_pos_y[current_step + 1] - support_pos_y[current_step]

                zmp_pos_x = last_zmp_x + (local_time - ss_time) * dx / ds_time
                zmp_pos_y = last_zmp_y + (local_time - ss_time) * dy / ds_time

            zmp_x.append(zmp_pos_x)
            zmp_y.append(zmp_pos_y)

            # Update time
            if local_time >= step_time - sampling_time:
                index += 1
                local_time = 0.0
            else:
                local_time += sampling_time

        self.zmp_x = zmp_x
        self.zmp_y = zmp_y

    def plot_zmp(self) -> None:
        """Visualizes the generated ZMP trajectory."""
        if not self.zmp_x or not self.zmp_y:
            raise ValueError("No ZMP trajectory available. Please run generate() first.")

        plt.plot(self.zmp_x, self.zmp_y, 'm-', label='ZMP Trajectory')
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Zero Moment Point (ZMP) Trajectory")
        plt.grid(True)
        plt.axis("equal")
        plt.legend()
        plt.show()

