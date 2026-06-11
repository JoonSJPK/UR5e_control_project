import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class PIDController:

  def __init__(self, joint, target, init):

    match joint:
      case 0:
          self.Kp = 220.0
          self.Ki = 0.0
          self.Kd = 50.0
          self.Kv = 50.0
      case 1:
          self.Kp = 220.0
          self.Ki = 0.0
          self.Kd = 50.0
          self.Kv = 50.0
      case 2:
          self.Kp = 220.0
          self.Ki = 0.0
          self.Kd = 50.0
          self.Kv = 50.0
      case 3:
          self.Kp = 220.0
          self.Ki = 0.0
          self.Kd = 50.0
          self.Kv = 50.0
      case 4:
          self.Kp = 220.0
          self.Ki = 0.0
          self.Kd = 50.0
          self.Kv = 50.0
      case 5:
          self.Kp = 220.0
          self.Ki = 0.0
          self.Kd = 50.0
          self.Kv = 50.0
      case _:
          self.Kp = 0.0
          self.Ki = 0.0
          self.Kd = 0.0
          self.Kv = 0.0
    
    self.integral = 0.0
    self.t1 = 2
    self.t2 = 6
    self.t3 = 8

    self.target = target
    self.init = init

    # Peak velocity: integral of the trapezoid must equal total displacement
    total_area = self.t1 / 2 + (self.t2 - self.t1) + (self.t3 - self.t2) / 2
    self.v_peak = (target - init) / total_area

  def reset(self):
    self.integral = 0.0

  def compute(self, dt, target_qpos, curr_qpos, curr_qvel, target_qvel=0.0):
    pos_error = target_qpos - curr_qpos
    self.integral += pos_error * dt
    torques = (self.Kp * pos_error) + (self.Ki * self.integral) - (self.Kd * curr_qvel) + (self.Kv * target_qvel)

    return torques

  def compute_tgt_vel(self, t):
      if t >= 0 and t < self.t1:
          return self.v_peak * t / self.t1
      elif t >= self.t1 and t < self.t2:
          return self.v_peak
      elif t >= self.t2 and t < self.t3:
          return self.v_peak * (self.t3 - t) / (self.t3 - self.t2)
      else:
          return 0.0

  def compute_tgt_pos(self, t, curr_pos):
      accel = self.v_peak / self.t1
      decel = self.v_peak / (self.t3 - self.t2)
      pos_at_t1 = self.init + accel * self.t1 ** 2 / 2

      if t >= 0 and t < self.t1:
          return self.init + accel * t ** 2 / 2

      elif t >= self.t1 and t < self.t2:
          return pos_at_t1 + self.v_peak * (t - self.t1)

      elif t >= self.t2 and t < self.t3:
          pos_at_t2 = pos_at_t1 + self.v_peak * (self.t2 - self.t1)
          dt = t - self.t2
          return pos_at_t2 + self.v_peak * dt - decel * dt ** 2 / 2

      #else:
         #return curr_pos