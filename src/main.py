import os

import mujoco
import mujoco.viewer
from controllers import PIDController
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "models", "universal_robots_ur5e", "scene.xml")

def plot_graph(dt, y_values, target_value, kp, ki, kd):
    steps = int(10 / dt)
    x_values = [i * dt for i in range(steps)]

    fig, ax = plt.subplots()
    ax.plot(x_values, y_values, label='Joint 6 angle')
    ax.axhline(y=target_value, color='r', linestyle='--', label='Target')

    ax.set_xlim(0, 10)
    ax.set_xlabel('Time (sec)')
    ax.set_ylabel('Angle (rad)')
    ax.set_title(f'Joint 6 Tracking — Kp={kp}  Ki={ki}  Kd={kd}')
    ax.legend()

    plt.tight_layout()
    filename = f'Joint6_Tracking_Kp{kp}_Ki{ki}_Kd{kd}.png'
    plt.savefig(filename)
    plt.close(fig)


def main():
  model = mujoco.MjModel.from_xml_path(SCENE_XML)
  data = mujoco.MjData(model)
  dt = model.opt.timestep

  controllers = []

  for _ in range(6):
     controllers.append(PIDController(Kp=30, Ki= 0, Kd= 0))

  #target = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]) 
  target = -1.5708
  collect_rad = []
  count = 0

  with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        
        #for i in range(len(target)):
           #torques = controllers[i].compute(dt, target[i], data.qpos[i], data.qvel[i])
           #data.qfrc_applied[i] = torques
        torques = controllers[5].compute(dt, target, data.qpos[5], data.qvel[5])
        data.qfrc_applied[5] = torques

        mujoco.mj_step(model, data)
        viewer.sync()

        steps = 10/dt
        if(count < (steps)):
          collect_rad.append(float(data.qpos[5]))
          count += 1
        if(count == steps):
          plot_graph(dt, collect_rad, target, controllers[5].Kp, controllers[5].Ki, controllers[5].Kd)
          count += 1

        
           
        


if __name__ == "__main__":
    main()