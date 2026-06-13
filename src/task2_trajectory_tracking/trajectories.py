import os
from datetime import datetime

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
from tuned_controllers import PIDController

SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "..", "models", "universal_robots_ur5e", "scene.xml")

def main():

  model = mujoco.MjModel.from_xml_path(SCENE_XML)
  data = mujoco.MjData(model)
  dt = model.opt.timestep

  init  = [-1.5708, -1.5708, -1.5708, -1.5708, -1.5708, -1.5708]
  target = [1.5708, -3.1416, 1.5708, -3.1416, 1.5708, -3.1416]

  #init  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ]
  #target  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ]


  controllers = []
  for idx in range(6):
     controllers.append(PIDController(joint = idx, target = target[idx], init = init[idx]))

  prev_time = 0.0

  trans_time = dt * 4000
  #trans_state = [0, 1, 2]

  #integrate trapazoid velocity equation to get positions

  #variable setup
  steps_total = int(10 / dt)
  collect_vel = {idx: [] for idx in range(6)}
  collect_pos = {idx: [] for idx in range(6)}
  collect_tgt_vel = {idx: [] for idx in range(6)}
  collect_tgt_pos = {idx: [] for idx in range(6)}
  count = 0
  plot_saved = False

  with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():

            # Detect viewer reset — data.time jumps back to 0
            if data.time < prev_time:
                for controller in controllers:
                    controller.reset()
            prev_time = data.time
            
            #apply torque to each joint
            #if( data.time <= trans_time ):
              #for idx, controller in enumerate(controllers):
                 # data.qfrc_applied[idx] = controller.compute(
                     # dt, targets[0][idx], data.qpos[idx], data.qvel[idx]
                 # )

            #apply torque to each joint
            #if( data.time > trans_time ):
             # for idx, controller in enumerate(controllers):
            #      data.qfrc_applied[idx] = controller.compute(
             #         dt, targets[1][idx], data.qpos[idx], data.qvel[idx]
             #     )

            if( data.time >= 0.0 and data.time < trans_time ):
                for idx, controller in enumerate(controllers):
                    data.ctrl[idx] = data.qpos[idx]
                    data.qfrc_applied[idx] = controller.compute(
                        dt, init[idx], data.qpos[idx], data.qvel[idx]
                    ) + data.qfrc_bias[idx]
            elif( data.time >= trans_time and data.time <= trans_time + 8):
                trans_tgt = []
                for idx, controller in enumerate(controllers):
                    curr_time = data.time - trans_time
                    trans_tgt.append(controller.compute_tgt_pos(curr_time, data.qpos[idx]))

                for idx, controller in enumerate(controllers):
                    tgt_vel = controller.compute_tgt_vel(curr_time)
                    data.ctrl[idx] = data.qpos[idx]  # neutralize built-in spring actuator
                    data.qfrc_applied[idx] = controller.compute(
                        dt, trans_tgt[idx], data.qpos[idx], data.qvel[idx], tgt_vel
                    ) + data.qfrc_bias[idx]
                #collect data
                if count < steps_total:
                    for idx in range(6):
                        collect_vel[idx].append(float(data.qvel[idx]))
                        collect_pos[idx].append(float(data.qpos[idx]))
                        collect_tgt_vel[idx].append(controllers[idx].compute_tgt_vel(curr_time))
                        collect_tgt_pos[idx].append(trans_tgt[idx])
                    count += 1
            else:
                for idx, controller in enumerate(controllers):
                    data.ctrl[idx] = data.qpos[idx]
                    data.qfrc_applied[idx] = controllers[idx].compute(
                        dt, target[idx], data.qpos[idx], data.qvel[idx]
                    ) + data.qfrc_bias[idx]
                #collect data
                if count < steps_total:
                    for idx in range(6):
                        collect_vel[idx].append(float(data.qvel[idx]))
                        collect_pos[idx].append(float(data.qpos[idx]))
                        collect_tgt_vel[idx].append(0.0)
                        collect_tgt_pos[idx].append(float(target[idx]))
                    count += 1

            

            
            #update simulation
            mujoco.mj_step(model, data)
            viewer.sync()



            if count == steps_total and not plot_saved:
                times = [i * dt for i in range(count)]
                fig, axes = plt.subplots(6, 2, figsize=(14, 18))
                for idx in range(6):
                    axes[idx, 0].plot(times[:len(collect_pos[idx])], collect_pos[idx], label="actual pos")
                    axes[idx, 0].plot(times[:len(collect_tgt_pos[idx])], collect_tgt_pos[idx], label="target pos", linestyle="--")
                    axes[idx, 0].set_ylabel("Position (rad)")
                    axes[idx, 0].set_title(f"Joint {idx + 1} Position")
                    axes[idx, 0].legend()
                    axes[idx, 1].plot(times[:len(collect_vel[idx])], collect_vel[idx], label="actual vel")
                    axes[idx, 1].plot(times[:len(collect_tgt_vel[idx])], collect_tgt_vel[idx], label="target vel", linestyle="--")
                    axes[idx, 1].set_ylabel("Velocity (rad/s)")
                    axes[idx, 1].set_title(f"Joint {idx + 1} Velocity")
                    axes[idx, 1].legend()
                for ax in axes[-1]:
                    ax.set_xlabel("Time (s)")
                plt.tight_layout()
                out_path = os.path.join(os.path.dirname(__file__), f"vel_6_joints_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
                fig.savefig(out_path)
                plt.close(fig)
                print(f"Plot saved to {out_path}")
                plot_saved = True

                




if __name__ == "__main__":
    main()
