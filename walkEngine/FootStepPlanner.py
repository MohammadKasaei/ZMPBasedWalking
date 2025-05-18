import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class Foot:
    x: float
    y: float
    z: float = 0.0  # default on the ground


class FootStepPlanner:
    def __init__(self, fr0: Tuple[float, float] = (0.15, 0.), fl0: Tuple[float, float] = (0., 0.1)):
        self.initial_right = Foot(*fr0)
        self.initial_left = Foot(*fl0)

        self.right_foot_steps: List[Foot] = [self.initial_right]
        self.left_foot_steps: List[Foot] = [self.initial_left]
        self.support_positions: List[Foot] = []

    def plan_steps(self, number_of_steps: int, step_x: float, step_y: float, first_step_is_right: bool):
        right_is_support = not first_step_is_right
        left_is_support = first_step_is_right

        for i in range(number_of_steps):
            if right_is_support:
                self.support_positions.append(self.right_foot_steps[-1])
                last_left = self.left_foot_steps[-1]
                new_left = Foot(last_left.x + step_x, last_left.y + step_y)
                self.left_foot_steps.append(new_left)
                right_is_support = False
                left_is_support = True
            else:
                self.support_positions.append(self.left_foot_steps[-1])
                last_right = self.right_foot_steps[-1]
                new_right = Foot(last_right.x + step_x, last_right.y + step_y)
                self.right_foot_steps.append(new_right)
                right_is_support = True
                left_is_support = False

        # Handle final double support phase
        if right_is_support:
            self.support_positions.append(self.right_foot_steps[-1])
            last_left = self.left_foot_steps[-1]
            final_left = Foot(last_left.x + step_x / 2, last_left.y + step_y / 2)
            self.left_foot_steps.append(final_left)
            self.support_positions.append(final_left)
        elif left_is_support:
            self.support_positions.append(self.left_foot_steps[-1])
            last_right = self.right_foot_steps[-1]
            final_right = Foot(last_right.x + step_x / 2, last_right.y + step_y / 2)
            self.right_foot_steps.append(final_right)
            self.support_positions.append(final_right)

    def get_paths(self):
        return {
            "right_foot": [(f.x, f.y) for f in self.right_foot_steps],
            "left_foot": [(f.x, f.y) for f in self.left_foot_steps],
            "support": [(f.x, f.y) for f in self.support_positions],
        }

    def plot_steps(self):
        paths = self.get_paths()
        rx, ry = zip(*paths["right_foot"])
        lx, ly = zip(*paths["left_foot"])
        sx, sy = zip(*paths["support"])
        


        plt.plot(rx, ry, 'ro-', label='Right Foot')
        plt.plot(lx, ly, 'bo-', label='Left Foot')
        plt.plot(sx, sy, 'go--', label='Support')
        plt.legend()
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Footstep Plan")
        plt.grid(True)
        plt.axis("equal")
        plt.show()

