import numpy as np


class PIDController:

  def __init__(self, Kp, Ki, Kd, integral_limit=50.0, Kv=0.0):
    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd
    self.Kv = Kv
    self.integral_limit = integral_limit
    self.integral = 0.0

  def reset(self):
    self.integral = 0.0

  def compute(self, dt, target_qpos, curr_qpos, curr_qvel, target_qvel=0.0):
    pos_error = target_qpos - curr_qpos
    self.integral += pos_error * dt
    self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
    torques = (self.Kp * pos_error) + (self.Ki * self.integral) - (self.Kd * curr_qvel) + (self.Kv * target_qvel)

    return torques


def make_joint_controller(joint):
  match joint:
    case 0:
        Kp = 221.0
        Ki = 300.0
        Kd = 38.0
        integral_limit = 30.0
    case 1:
        Kp = 1575.0
        Ki = 900.0
        Kd = 45.0
        integral_limit = 30.0
    case 2:
        Kp = 567.0
        Ki = 300.0
        Kd = 63.0
        integral_limit = 30.0
    case 3:
        Kp = 300.0
        Ki = 100.0
        Kd = 28.0
        integral_limit = 5.0
    case 4:
        Kp = 67.0
        Ki = 100.0
        Kd = 3.0
        integral_limit = 5.0
    case 5:
        Kp = 78.0
        Ki = 100.0
        Kd = 3.0
        integral_limit = 5.0
    case _:
        Kp = 0.0
        Ki = 0.0
        Kd = 0.0
        integral_limit = 0.0

  return PIDController(Kp, Ki, Kd, integral_limit, Kv=Kd)
