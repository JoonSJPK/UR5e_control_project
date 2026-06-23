import os

import mujoco
import mujoco.viewer
import numpy as np
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d projection)

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

    site_id = model.site("attachment_site").id
    path_log = []
    path_saved = False

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():

          mujoco_move(dt, viewer, controllers, model, data, init_theta, site_id, path_log)

          if (data.time > 6) and not path_saved:
            path_log.clear()
            while (e_mag > 0.001):

              for _ in range(10):
                mujoco_move(dt, viewer, controllers, model, data, init_theta, site_id, path_log)

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

            # IK converged (e_mag <= 0.001) -- save the path once
            plot_ee_path(path_log, target)
            path_saved = True

def bottle_grasp_target(model, data, height_frac=1.0):
  body_id = model.body("bottle_target").id
  geom_id = None
  for g in range(model.ngeom):
    if model.geom_bodyid[g] == body_id:
      geom_id = g
      break

  # local
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

def mujoco_move(dt, viewer, controllers, model, data, init_theta, site_id=None, path_log=None):
  for idx, controller in enumerate(controllers):
    #data.ctrl[idx] = data.qpos[idx]
    data.qfrc_applied[idx] = controller.compute(
        dt, init_theta[idx], data.qpos[idx], data.qvel[idx]
    ) #+ data.qfrc_bias[idx]

  #update simulation
  mujoco.mj_step(model, data)

  # record the end-effector world position and drop a live breadcrumb
  if site_id is not None and path_log is not None:
    ee = data.site_xpos[site_id].copy()
    path_log.append(ee)
    if len(path_log) % 25 == 0:
      add_trace_point(viewer, ee)

  viewer.sync()

def add_trace_point(viewer, pos, rgba=(1.0, 0.2, 0.2, 1.0), radius=0.004):
  """live trail."""
  scn = viewer.user_scn
  if scn.ngeom >= scn.maxgeom:
    return
  mujoco.mjv_initGeom(
      scn.geoms[scn.ngeom],
      type=mujoco.mjtGeom.mjGEOM_SPHERE,
      size=np.array([radius, 0.0, 0.0]),
      pos=np.asarray(pos, dtype=float),
      mat=np.eye(3).flatten(),
      rgba=np.array(rgba, dtype=float),
  )
  scn.ngeom += 1

def plot_ee_path(path_log, target, out_path=None):
  """Save a 3D view as png"""
  if not path_log:
    print("plot_ee_path: no path recorded, skipping")
    return

  path = np.asarray(path_log)
  target = np.asarray(target)
  if out_path is None:
    out_path = os.path.join(os.path.dirname(__file__), "ee_path.png")

  fig = plt.figure(figsize=(12, 9))

  ax = fig.add_subplot(2, 2, 1, projection='3d')
  ax.plot(path[:, 0], path[:, 1], path[:, 2], color='tab:blue', lw=1.2, label='EE path')
  ax.scatter(*path[0], color='green', s=40, label='start')
  ax.scatter(*path[-1], color='orange', s=40, label='end')
  ax.scatter(*target, color='red', marker='*', s=140, label='target')
  ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
  ax.set_title('End-effector Cartesian path (3D)')
  ax.legend(loc='upper left', fontsize=8)

  for k, ((i, j), name) in enumerate([((0, 1), 'XY'), ((0, 2), 'XZ'), ((1, 2), 'YZ')]):
    a = fig.add_subplot(2, 2, k + 2)
    a.plot(path[:, i], path[:, j], color='tab:blue', lw=1.2)
    a.scatter(path[0, i], path[0, j], color='green', s=30, label='start')
    a.scatter(path[-1, i], path[-1, j], color='orange', s=30, label='end')
    a.scatter(target[i], target[j], color='red', marker='*', s=140, label='target')
    a.set_xlabel(f'{name[0]} [m]'); a.set_ylabel(f'{name[1]} [m]')
    a.set_title(f'{name} projection'); a.axis('equal'); a.grid(True, alpha=0.3)

  fig.tight_layout()
  fig.savefig(out_path, dpi=130)
  plt.close(fig)
  print(f"saved EE path plot -> {out_path}  ({len(path)} points)")

    






if __name__ == "__main__":
    main()