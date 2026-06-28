import os
import sys

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pid_controller import make_joint_controller
from kinematics import (UR5eKinematics, ik_step, validate_fk, bottle_grasp_target,
                        mujoco_move, plot_ee_path, calc_p, calc_z_axis, calc_r, GRIP_OFFSET)

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

    # open the gripper at the start
    grip_id = model.actuator("gripper").id
    data.ctrl[grip_id] = 255

    mujoco.mj_forward(model, data)
    validate_fk(model, data, kin)
    p_target = bottle_grasp_target(model, data, height_frac=0.75)
    R_d = np.array([[ 0.0,  0.0,  1.0],
                    [ 0.0,  1.0,  0.0],
                    [-1.0,  0.0,  0.0]])
    pos_tol = 0.001
    rot_tol = 0.001

    home_q = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0]
    T_home, _, _, _ = kin.forward_kinematics(home_q)
    p_home = calc_p(T_home) + GRIP_OFFSET * calc_z_axis(T_home)
    R_home = calc_r(T_home)

    site_id = model.site("attachment_site").id
    path_log = []
    ik_active = False
    phase = "reach"          # reach -> return -> done

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():

          # one resolved-rate IK update per simulation step
          if (data.time > 6) and phase in ("reach", "return"):
            if not ik_active:
              path_log.clear()
              ik_active = True

            goal_p = p_target if phase == "reach" else p_home
            goal_R = R_d if phase == "reach" else R_home
            delta_theta, e_p_mag, e_o_mag = ik_step(kin, data, goal_p, goal_R)

            init_theta = np.add(init_theta, delta_theta)
            print(f"[{phase}] e_p={e_p_mag:.4f}  e_o={e_o_mag:.4f}")

            if e_p_mag < pos_tol and e_o_mag < rot_tol:
              if phase == "reach":
                plot_ee_path(path_log, p_target)
                data.ctrl[grip_id] = 0 

                t_close = data.time
                while viewer.is_running() and data.time - t_close < 2.0:
                  mujoco_move(dt, viewer, controllers, model, data, init_theta, site_id, path_log)

                phase = "return"
              else:
                phase = "done"

          mujoco_move(dt, viewer, controllers, model, data, init_theta, site_id, path_log)


if __name__ == "__main__":
    main()
