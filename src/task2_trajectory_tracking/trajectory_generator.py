class TrajectoryGenerator:

  def __init__(self, target, init):
    self.t1 = 2
    self.t2 = 6
    self.t3 = 8

    self.target = target
    self.init = init

    # Peak velocity: integral of the trapezoid must equal total displacement
    time_factor = self.t1 / 2 + (self.t2 - self.t1) + (self.t3 - self.t2) / 2
    self.v_peak = (target - init) / time_factor

  def compute_tgt_vel(self, t):
      if t >= 0 and t < self.t1:
          return self.v_peak * t / self.t1
      elif t >= self.t1 and t < self.t2:
          return self.v_peak
      elif t >= self.t2 and t < self.t3:
          return self.v_peak * (self.t3 - t) / (self.t3 - self.t2)
      else:
          return 0.0

  def compute_tgt_pos(self, t):
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
