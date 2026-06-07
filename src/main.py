import argparse
import os

import mujoco
import mujoco.viewer
import numpy as np
from controllers import PIDController


SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "models", "universal_robots_ur5e", "scene.xml")

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--joint", type=int, default=6, help="Joint index (1-6)")
  parser.add_argument("--kp", type=float, default=100)
  parser.add_argument("--ki", type=float, default=100)
  parser.add_argument("--kd", type=float, default=100)
  parser.add_argument("--out", type=str, default=None, help="Output plot path")
  args = parser.parse_args()

  joint_idx = args.joint - 1  # 0-based

  model = mujoco.MjModel.from_xml_path(SCENE_XML)
  data = mujoco.MjData(model)
  dt = model.opt.timestep

  controllers = []
  for i in range(6):
     controllers.append(PIDController(Kp=args.kp, Ki=args.ki, Kd=args.kd))

  #target = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
  target = -1.5708
  collect_rad = []
  count = 0

  with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():

        #for i in range(len(target)):
           #torques = controllers[i].compute(dt, target[i], data.qpos[i], data.qvel[i])
           #data.qfrc_applied[i] = torques
        torques = controllers[joint_idx].compute(dt, target, data.qpos[joint_idx], data.qvel[joint_idx])
        data.qfrc_applied[joint_idx] = torques

        mujoco.mj_step(model, data)
        viewer.sync()

        #collect data and graph ouput
        steps = 10/dt
        if(count < (steps)):
          collect_rad.append(float(data.qpos[joint_idx]))
          count += 1
        if(count == steps):
          out_path = args.out or os.path.join(os.path.dirname(__file__), "matplotlib_graph.png")
          controllers[joint_idx].plot_graph(dt, collect_rad, target, joint_num=args.joint, out_path=out_path)
          count += 1

if __name__ == "__main__":
    main()
