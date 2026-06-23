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

    # initial joint position in rad — standard UR5e ready pose
    init_theta  = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0]
    controllers = []
    for idx in range(6):
      controllers.append(PIDController(joint = idx, theta = init_theta[idx]))

    mujoco.mj_forward(model, data)
    validate_fk(model, data, controllers)
    target = bottle_grasp_target(model, data, height_frac=1.0)
    e_mag = 100

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():

          mujoco_move(dt, viewer, controllers, model, data, init_theta)

          if (data.time > 6):
            while (e_mag > 0.001):

              for _ in range(10):
                mujoco_move(dt, viewer, controllers, model, data, init_theta)

              elem_steps = []
              for idx in range(6):
                controllers[idx].theta = data.qpos[idx]
                elem_steps.append(controllers[idx].fk_elem_step())

              trans_cum_matrix, trans_matrix, z, p = fk_trans_matrix(elem_steps)

              e = calc_error(target, trans_cum_matrix)
              e_mag = calc_e_mag(e)
              print(e_mag)

              pe = p[-1]
              jv_transpose = []
              for idx in range(6):
                jv_transpose.append(calc_jv_column(z[idx], pe - p[idx]))

              jv = np.array(jv_transpose).T

              step_size = 0.01
              delta_p = [step_size * e[0], step_size * e[1], step_size * e[2]]
              identity = np.array([[1,0,0],
                                  [0,1,0],
                                  [0,0,1]])
              lambda_squared = 0.0025 ** 2

              delta_theta = (jv_transpose @ np.linalg.inv((jv @ jv_transpose) + (lambda_squared * identity))) @ delta_p

              init_theta = np.add(init_theta, delta_theta)

def bottle_grasp_target(model, data, height_frac=1.0):
  body_id = model.body("bottle_target").id
  geom_id = None
  for g in range(model.ngeom):
    if model.geom_bodyid[g] == body_id:
      geom_id = g
      break

  # local AABB: center (cx,cy,cz) + half-sizes (_,_,hz)
  cx, cy, cz, _, _, hz = model.geom_aabb[geom_id]
  bottom_local = np.array([cx, cy, cz - hz])
  top_local    = np.array([cx, cy, cz + hz])
  grasp_local  = bottom_local + height_frac * (top_local - bottom_local)

  # transform from geom frame to world frame
  geom_pos = data.geom_xpos[geom_id]
  geom_mat = data.geom_xmat[geom_id].reshape(3, 3)
  return geom_pos + geom_mat @ grasp_local

def fk_world_pos(controllers, q):
  elem_steps = []
  for idx in range(6):
    controllers[idx].theta = q[idx]
    elem_steps.append(controllers[idx].fk_elem_step())

  trans_cum_matrix, _, _, _ = fk_trans_matrix(elem_steps)
  return calc_p(trans_cum_matrix)

def validate_fk(model, data, controllers, tol=2e-3):
  site_id = model.site("attachment_site").id
  wrist_id = model.body("wrist_3_link").id

  rng = np.random.default_rng(0)
  saved = np.array(data.qpos[:6]).copy()
  samples = [saved] + [rng.uniform(-2.0, 2.0, 6) for _ in range(4)]

  worst = 0.0
  print("FK validation (DH flange vs MuJoCo attachment_site):")
  for q in samples:
    data.qpos[:6] = q
    mujoco.mj_forward(model, data)

    p_dh = fk_world_pos(controllers, q)
    r_site = np.linalg.norm(p_dh - data.site_xpos[site_id])
    r_wrist = np.linalg.norm(p_dh - data.xpos[wrist_id])
    worst = max(worst, r_site)
    print(f"  q={np.round(q, 3)}  site_resid={r_site:.2e} m"
          f"  (wrist_3_resid={r_wrist:.2e} m)")

  # restore the state
  data.qpos[:6] = saved
  mujoco.mj_forward(model, data)

  assert worst < tol, (
      f"FK/MuJoCo mismatch {worst:.4e} m exceeds tol {tol:.1e} m ")
  print(f"  PASS: max flange residual {worst:.2e} m < tol {tol:.1e} m\n")

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

   return cross

def calc_e_mag(e):
   e_mag = math.sqrt((e[0] * e[0] + e[1] * e[1] + e[2] * e[2]))

   return e_mag

def mujoco_move(dt, viewer, controllers, model, data, init_theta):
  for idx, controller in enumerate(controllers):
    #data.ctrl[idx] = data.qpos[idx]
    data.qfrc_applied[idx] = controller.compute(
        dt, init_theta[idx], data.qpos[idx], data.qvel[idx]
    ) #+ data.qfrc_bias[idx]

  #update simulation
  mujoco.mj_step(model, data)
  viewer.sync()

    






if __name__ == "__main__":
    main()