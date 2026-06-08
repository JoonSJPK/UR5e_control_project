import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class PIDController:

  def __init__(self, Kp, Ki, Kd):
    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd
    self.integral = 0.0

  def reset(self):
    self.integral = 0.0

  def compute(self, dt, target_qpos, curr_qpos, curr_qvel):
    error = target_qpos - curr_qpos
    self.integral += error * dt
    torques = (self.Kp * error) + (self.Ki * self.integral) - (self.Kd * curr_qvel)

    return torques
  
  def plot_graph(self, dt, y_values, target_value, joint_num=6, out_path='matplotlib_graph.png'):
    steps = int(10 / dt)
    x_values = [i * dt for i in range(steps)]

    fig, ax = plt.subplots()
    ax.plot(x_values, y_values, label=f'Joint {joint_num} angle')
    ax.axhline(y=target_value, color='r', linestyle='--', label='Target')

    ax.set_xlim(0, 10)
    ax.set_xlabel('Time (sec)')
    ax.set_ylabel('Angle (rad)')
    ax.set_title(f'Joint {joint_num} Tracking')
    ax.legend()

    plt.tight_layout()
    plt.savefig(out_path)
    plt.close(fig)