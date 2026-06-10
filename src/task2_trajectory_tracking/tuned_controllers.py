import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class PIDController:

  def __init__(self, joint, target, init):

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
    self.t1 = 1
    self.t2 = 3
    self.t3 = 4
    
    self.target = target
    self.init = init
    self.a = (target - init) / 3
    self.b = self.a
    self.c = self.a * self.t3

  def reset(self):
    self.integral = 0.0

  def compute(self, dt, target_qpos, curr_qpos, curr_qvel, target_qvel=0.0):
    error = target_qpos - curr_qpos
    self.integral += error * dt
    torques = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * (target_qvel - curr_qvel))

    return torques

  def compute_tgt_vel(self, t):
      if t >= 0 and t < self.t1:
          return self.a * t
      elif t >= self.t1 and t < self.t2:
          return self.a
      elif t >= self.t2 and t < self.t3:
          return self.a * (self.t3 - t)
      else:
          return 0.0
  
  def compute_tgt_pos(self, t, curr_pos):
      if( t >= 0 and t < self.t1 ):
        tgt_pos = ((self.a * t * t) / 2) + self.init

        return tgt_pos

      elif( t >= self.t1 and t < self.t2 ):
        tgt_pos = (self.b * t) + ((self.a * self.t1 * self.t1) / 2) + (self.init) - (self.b * self.t1)

        return tgt_pos

      elif( t >= self.t2 and t < self.t3 ):
        tgt_pos = -((self.a * t * t) / 2) + (self.c * t) + (self.b * self.t2) + ((self.a * self.t1 * self.t1) / 2) + (self.init) - (self.b * self.t1) + ((self.a * self.t2 * self.t2) / 2) - (self.c * self.t2)

        return tgt_pos
      
      #else:
         #return curr_pos