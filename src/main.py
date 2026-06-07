import os

import mujoco
import mujoco.viewer
import numpy as np
from controllers import PIDController

SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "models", "universal_robots_ur5e", "scene.xml")

def main():
  model = mujoco.MjModel.from_xml_path(SCENE_XML)
  data = mujoco.MjData(model)
  dt = model.opt.timestep

  controller = PIDController(Kp=200, Ki= 100, Kd=100)
  target = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]) 


  with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        error, torques = controller.compute(dt, target, data.qpos[:6], data.qvel[:6])
        data.qfrc_applied[:6] = torques
        mujoco.mj_step(model, data)
        viewer.sync()

if __name__ == "__main__":
    main()