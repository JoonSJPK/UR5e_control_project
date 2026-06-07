import numpy as np

class PIDController:

  def __init__(self, Kp, Ki, Kd):
    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd
    self.integral = 0.0

  def compute(self, dt, target_qpos, curr_qpos, curr_qvel):
    error = target_qpos - curr_qpos
    self.integral += error * dt
    torques = (self.Kp * error) + (self.Ki * self.integral) - (self.Kd * curr_qvel)

    return error, torques