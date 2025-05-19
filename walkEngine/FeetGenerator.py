import matplotlib.pyplot as plt
from typing import List, Tuple
        
class FootPositionGenerator:
    def __init__(self, right_foot_steps, left_foot_steps, support_steps, step_time, sampling_time, foot_clearance=0.05):
        self.right_foot_steps = right_foot_steps
        self.left_foot_steps = left_foot_steps
        self.support_steps = support_steps
        self.step_time = step_time
        self.sampling_time = sampling_time
        self.foot_clearance = foot_clearance

        self.right_foot_traj: List[Tuple[float, float, float]] = []
        self.left_foot_traj: List[Tuple[float, float, float]] = []

    def parabolic_z(self, t, T):
        """Parabolic swing trajectory: peak at T/2."""
        return 4 * self.foot_clearance * (t / T) * (1 - t / T)

    def generate(self):
        num_phases = len(self.support_steps) - 1
        total_time = num_phases * self.step_time
        num_samples = int(total_time / self.sampling_time)
        

        right_index = 0
        left_index = 0

        for i in range(num_samples):
            current_time = i * self.sampling_time
            phase_index = int(current_time // self.step_time)
            phase_time = current_time % self.step_time

            # Stop safely if we're out of bounds
            if phase_index >= len(self.support_steps) - 1:
                break

            # Determine support foot
            support = self.support_steps[phase_index]
            is_right_support = support == self.right_foot_steps[right_index]

            if is_right_support:
                # Right is stance, left is swing
                right = self.right_foot_steps[right_index]
                next_left_index = left_index + 1

                if next_left_index >= len(self.left_foot_steps):
                    next_left_index = left_index  # no more swing

                start = self.left_foot_steps[left_index]
                end = self.left_foot_steps[next_left_index]

                ratio = min(phase_time / self.step_time, 1.0)
                x = start.x + ratio * (end.x - start.x)
                y = start.y + ratio * (end.y - start.y)
                z = self.parabolic_z(phase_time, self.step_time)

                self.right_foot_traj.append((right.x, right.y, 0.0))
                self.left_foot_traj.append((x, y, z))
                
                if phase_time >= self.step_time - self.sampling_time and next_left_index != left_index:
                    left_index = next_left_index

            else:
                # Left is stance, right is swing
                left = self.left_foot_steps[left_index]
                next_right_index = right_index + 1

                if next_right_index >= len(self.right_foot_steps):
                    next_right_index = right_index  # no more swing

                start = self.right_foot_steps[right_index]
                end = self.right_foot_steps[next_right_index]

                ratio = min(phase_time / self.step_time, 1.0)
                x = start.x + ratio * (end.x - start.x)
                y = start.y + ratio * (end.y - start.y)
                z = self.parabolic_z(phase_time, self.step_time)

                self.left_foot_traj.append((left.x, left.y, 0.0))
                self.right_foot_traj.append((x, y, z))

                if phase_time>= self.step_time-self.sampling_time and next_right_index != right_index:
                    right_index = next_right_index

    def get_trajectories(self):
        return {
            "right_foot": self.right_foot_traj,
            "left_foot": self.left_foot_traj,
        }

    def plot(self):

        rx, ry, rz = zip(*self.right_foot_traj)
        lx, ly, lz = zip(*self.left_foot_traj)

        fig, ax = plt.subplots(3, 1, figsize=(8, 12))
        ax[0].plot(rx, 'r-', label='Right Foot Trajectory')
        ax[0].plot(lx, 'r--', label='Left Foot Trajectory')
        ax[1].plot(ry, 'g-', label='Right Foot Trajectory')
        ax[1].plot(ly, 'g--', label='Left Foot Trajectory')
        ax[2].plot(rz, 'b-', label='Right Foot Trajectory')
        ax[2].plot(lz, 'b--', label='Left Foot Trajectory')
        
        ax[0].legend()
        ax[1].legend()
        ax[2].legend()
        ax[0].grid()
        ax[1].grid()
        ax[2].grid()
        
        
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(rx, ry, rz, 'r-', label='Right Foot Trajectory')
        ax.plot(lx, ly, lz, 'b-', label='Left Foot Trajectory')
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("3D Foot Trajectories")
        ax.legend()
        plt.show()
        
        
