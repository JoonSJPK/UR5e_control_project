import numpy as np

class PIDController:
  
  def __init__(self, joint, theta):
    match joint:

      case 0:
        self.theta = theta
        self.d = 0.1625
        self.a = 0.0
        self.alpha = np.pi / 2
        self.Kp = 220.0
        self.Ki = 0.0
        self.Kd = 50.0
      
      case 1:
        self.theta = theta
        self.d = 0.0
        self.a = -0.425
        self.alpha = 0.0
        self.Kp = 220.0
        self.Ki = 0.0
        self.Kd = 50.0

      case 2:
        self.theta = theta
        self.d = 0.0
        self.a = -0.3922
        self.alpha = 0.0
        self.Kp = 220.0
        self.Ki = 0.0
        self.Kd = 50.0

      case 3:
        self.theta = theta
        self.d = 0.1333
        self.a = 0.0
        self.alpha = np.pi / 2
        self.Kp = 220.0
        self.Ki = 0.0
        self.Kd = 50.0

      case 4:
        self.theta = theta
        self.d = 0.0997
        self.a = 0.0
        self.alpha = -np.pi / 2
        self.Kp = 220.0
        self.Ki = 0.0
        self.Kd = 50.0

      case 5:
        self.theta = theta
        self.d = 0.0996
        self.a = 0.0
        self.alpha = 0.0
        self.Kp = 220.0
        self.Ki = 0.0
        self.Kd = 50.0

    self.integral = 0.0
    #self.target = target
    #self.init = init


  def compute(self, dt, target_qpos, curr_qpos, curr_qvel):
    pos_error = target_qpos - curr_qpos
    self.integral += pos_error * dt
    torques = (self.Kp * pos_error) + (self.Ki * self.integral) - (self.Kd * curr_qvel)

    return torques

  def theta_getter(self, theta):
    return self.theta

  def theta_setter(self, theta):
    self.theta = theta

  def fk_elem_step(self):
    ct = np.cos(self.theta)
    st = np.sin(self.theta)
    cal = np.cos(self.alpha)
    sal = np.sin(self.alpha)
    a = self.a
    d = self.d
    elem_step = np.array([[ ct, -st * cal,  st * sal, a * ct ],
                          [ st,  ct * cal, -ct * sal, a * st ],
                          [  0,       sal,       cal,      d ],
                          [  0,          0,        0,      1 ]])
    
    return elem_step
  
  def fk_trans_matrix(elem):
    trans_matrix = np.array([[1,0,0,0],
                             [0,1,0,0],
                             [0,0,1,0],
                             [0,0,0,1]])
    for idx in range(6):
      trans_matrix = trans_matrix @ elem[idx]

    return trans_matrix

    