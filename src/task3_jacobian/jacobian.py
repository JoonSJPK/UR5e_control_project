import os

import mujoco
import mujoco.viewer
import numpy as np
import math

from jacobian_controllers import PIDController

SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "..", "models", "universal_robots_ur5e", "scene.xml")

def main():
    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data = mujoco.MjData(model)
    dt = model.opt.timestep

    # initial joint position in rad
    init_theta  = [0.3491, -1.7453, 1.3963, -1.5708, 1.2217, 0.5236]

    # Cartesian target position
    target = [0.40, 0.1, 0.5]
    e_mag = 100
    while (e_mag > 0.001):
      controllers = []
      for idx in range(6):
        controllers.append(PIDController(joint = idx, theta = init_theta[idx]))

      elem_steps = []
      for idx in range(6):
        elem_steps.append(controllers[idx].fk_elem_step())



      trans_cum_matrix, trans_matrix, z, p = fk_trans_matrix(elem_steps)
      #print(trans_cum_matrix)
      #print(trans_matrix)
      #print(p)
      #print(z)

      e = calc_error(target, trans_cum_matrix)
      e_mag = calc_e_mag(e)
      print(e_mag)
      #print(e)


      pe = p[-1]
      jv_transpose = []
      for idx in range(6):
        jv_transpose.append(calc_jv_column(z[idx], pe - p[idx]))
      #print(jv_transpose)

      jv = np.array(jv_transpose).T
      #print(jv)

      step_size = 0.01
      delta_p = [step_size * e[0], step_size * e[1], step_size * e[2]]
      identity = np.array([[1,0,0],
                           [0,1,0],
                           [0,0,1]])
      lambda_squared = 0.0025 * 0.0025
      #print(delta_p)

      delta_theta = (jv_transpose @ np.linalg.inv((jv @ jv_transpose) + (lambda_squared * identity))) @ delta_p

      #print(delta_theta)

      init_theta = np.add(init_theta, delta_theta)







def fk_trans_matrix(elem):

  trans_cum_matrix = np.array([[1,0,0,0],
                               [0,1,0,0],
                               [0,0,1,0],
                               [0,0,0,1]])
  trans_matrix = []
  z = [[0, 0, 1]]
  p = [[0, 0, 0]]

  for idx in range(6):
    trans_cum_matrix = trans_cum_matrix @ elem[idx]

    trans_matrix.append(trans_cum_matrix)
    p.append(calc_p(trans_cum_matrix))

  for idx in range(5):
     z.append(calc_z_axis(trans_matrix[idx]))

  return trans_cum_matrix, trans_matrix, z, p

def calc_p(matrix):
   
   px = matrix[0][3]
   py = matrix[1][3]
   pz = matrix[2][3]

   pi = np.array([px, py, pz])
   return pi

def calc_error(tgt, matrix):
   
   pe = calc_p(matrix)

   ex = tgt[0] - pe[0]
   ey = tgt[1] - pe[1]
   ez = tgt[2] - pe[2]

   e = np.array([ex, ey, ez])

   return e

def calc_z_axis(matrix):
   
   z1 = matrix[0][2]
   z2 = matrix[1][2]
   z3 = matrix[2][2]

   zi = np.array([z1, z2, z3])

   return zi

def calc_jv_column(z, p):
   
   cross = np.cross(z, p)
   #print(cross)
   return cross

def calc_e_mag(e):
   e_mag = math.sqrt((e[0] * e[0] + e[1] * e[1] + e[2] * e[2]))

   return e_mag

    






if __name__ == "__main__":
    main()