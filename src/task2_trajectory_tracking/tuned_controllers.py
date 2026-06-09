import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class PIDController:

  def __init__(self, joint):

    match joint:
      case 0:
          self.Kp = 220.0
          self.Ki = 280.0
          self.Kd = 50.0
      case 1:
          self.Kp = 220.0
          self.Ki = 180.0
          self.Kd = 50.0
      case 2:
          self.Kp = 220.0
          self.Ki = 210.0
          self.Kd = 50.0
      case 3:
          self.Kp = 220.0
          self.Ki = 70.0
          self.Kd = 50.0
      case 4:
          self.Kp = 220.0
          self.Ki = 70.0
          self.Kd = 50.0
      case 5:
          self.Kp = 220.0
          self.Ki = 70.0
          self.Kd = 50.0
      case _:
          self.Kp = 0.0
          self.Ki = 0.0
          self.Kd = 0.0
    
    self.integral = 0.0

  def reset(self):
    self.integral = 0.0

  def compute(self, dt, target_qpos, curr_qpos, curr_qvel):
    error = target_qpos - curr_qpos
    self.integral += error * dt
    torques = (self.Kp * error) + (self.Ki * self.integral) - (self.Kd * curr_qvel)

    return torques