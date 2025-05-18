from .FootStepPlanner import Foot
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple
import numpy as np


class COMGenerator:
    def __init__(self):
        self.com_x: List[float] = []
        self.com_y: List[float] = []
        self.com_z: List[float] = []

    def generate(
        self,
        com_x0: float,
        com_y0: float,
        support_pos_x: List[float],
        support_pos_y: List[float],
        zmp_x: List[float],
        zmp_y: List[float],
        step_time: float,
        sampling_time: float,
        number_of_steps: int,
        ds_ratio: float,
        com_height_amp: float,
        z_0: float,
    ):
        gravity = 9.81
        ds_time = ds_ratio * step_time
        self.zmp_x =zmp_x
        self.zmp_y =zmp_y   

        com_x = []
        com_y = []
        com_z = []

        z_c = z_0
        ddot_zc = 0.0
        a = (gravity + ddot_zc) / z_c
        w = np.sqrt(a)

        zmp_index = 0
        total_time = step_time * (number_of_steps - 1)

        for global_time in np.arange(0, total_time, sampling_time):
            t = global_time
            com_z.append(z_c)
            w = np.sqrt(z_c / gravity)  # Matching original line

            # --- Determine step interval (k) where current time falls
            k = number_of_steps + 1
            while k > 0:
                t0 = (k - 1) * step_time - ds_time / 2
                tf = k * step_time - ds_time / 2
                if t0 <= t <= tf:
                    break
                k -= 1

            if k > 1:
                t0 = (k - 1) * step_time - ds_time / 2
                tf = k * step_time - ds_time / 2
            else:
                t0 = 0.0
                tf = k * step_time - ds_time / 2

            # ---------------- Y DIRECTION ----------------
            z_y = zmp_y[min(zmp_index, len(zmp_y) - 1)]
            if zmp_index < len(zmp_y):
                zmp_index += 1

            if t0 == 0:
                y0 = com_y0
                yf = (support_pos_y[k - 1] + support_pos_y[k]) / 2
            else:
                y0 = (support_pos_y[k - 2] + support_pos_y[k - 1]) / 2
                yf = (support_pos_y[k - 1] + support_pos_y[k]) / 2

            denom = np.sinh((t0 - tf) / w)
            if abs(denom) < 1e-6:
                denom = 1e-6  # avoid division by zero

            y = z_y + (1 / denom) * (
                (yf - z_y) * np.sinh((t0 - t) / w)
                + (z_y - y0) * np.sinh((tf - t) / w)
            )
            com_y.append(y)

            # ---------------- X DIRECTION ----------------
            z_x = zmp_x[min(zmp_index, len(zmp_x) - 1)]

            if t0 == 0:
                x0 = com_x0
                xf = (support_pos_x[k - 1] + support_pos_x[k]) / 2
            else:
                x0 = (support_pos_x[k - 2] + support_pos_x[k - 1]) / 2
                xf = (support_pos_x[k - 1] + support_pos_x[k]) / 2

            x = z_x + (1 / denom) * (
                (xf - z_x) * np.sinh((t0 - t) / w)
                + (z_x - x0) * np.sinh((tf - t) / w)
            )
            com_x.append(x)

        self.com_x = com_x
        self.com_y = com_y
        self.com_z = com_z

    def get_trajectory(self):
        return {
            "com_x": self.com_x,
            "com_y": self.com_y,
            "com_z": self.com_z,
        }
    def plot(self):            
        plt.plot(self.com_x, self.com_y,'g', label="COM Trajectory")
        plt.plot(self.zmp_x, self.zmp_y, 'r--', label="ZMP Trajectory")
        plt.legend()
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("COM and ZMP Trajectory")
        plt.axis("equal")
        plt.grid(True)
        plt.show()
        