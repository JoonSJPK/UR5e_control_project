import os
import sys

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pid_controller import make_joint_controller
from kinematics import UR5eKinematics, ik_step, validate_fk, bottle_grasp_target, mujoco_move, plot_ee_path

SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "..", "models", "universal_robots_ur5e", "scene.xml")

def main():
    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data = mujoco.MjData(model)
    dt = model.opt.timestep

    # initial joint position
    init_theta  = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0]
    controllers = []
    for idx in range(6):
      controllers.append(make_joint_controller(idx))

    kin = UR5eKinematics()

    mujoco.mj_forward(model, data)
    validate_fk(model, data, kin)
    p_target = bottle_grasp_target(model, data, height_frac=1.0)
    R_d = np.array([[ 1.0,  0.0,  0.0],
                    [ 0.0, -1.0,  0.0],
                    [ 0.0,  0.0, -1.0]])
    pos_tol = 0.001
    rot_tol = 0.001

    site_id = model.site("attachment_site").id
    path_log = []
    path_saved = False
    ik_active = False

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():

          # one resolved-rate IK update per simulation step
          if (data.time > 6) and not path_saved:
            if not ik_active:
              path_log.clear()
              ik_active = True

            delta_theta, e_p_mag, e_o_mag = ik_step(kin, data, p_target, R_d)

            init_theta = np.add(init_theta, delta_theta)
            print(f"e_p={e_p_mag:.4f}  e_o={e_o_mag:.4f}")

            if e_p_mag < pos_tol and e_o_mag < rot_tol:
              plot_ee_path(path_log, p_target)
              path_saved = True

          mujoco_move(dt, viewer, controllers, model, data, init_theta, site_id, path_log)


if __name__ == "__main__":
    main()
