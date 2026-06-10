import os

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
  target = [1.5708, -1.5708, 1.5708, 1.5708, 1.5708, 1.5708]

  controllers = []
  for idx in range(6):
     controllers.append(PIDController(joint = idx, target = target[idx], init = init[idx]))

  prev_time = 0.0

  trans_time = dt * 4000
  #trans_state = [0, 1, 2]

  #integrate trapazoid velocity equation to get positions

  #variable setup
  steps_total = int(10 / dt)
  collect = {idx: [] for idx in range(6)}
  count = 0
  plot_saved = False

  with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            
            # Detect viewer reset — data.time jumps back to 0
            if data.time < prev_time or data.time == trans_time:
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
                    data.qfrc_applied[idx] = controllers[idx].compute(
                        dt, init[idx], data.qpos[idx], data.qvel[idx]
                    )
            elif( data.time >= trans_time and data.time <= trans_time + 4):
                trans_tgt = []
                for idx, controller in enumerate(controllers):
                    curr_time = data.time - trans_time
                    trans_tgt.append(controllers[idx].compute_tgt_pos(curr_time, data.qpos[idx]))

                for idx, controller in enumerate(controllers):
                    tgt_vel = controllers[idx].compute_tgt_vel(curr_time)
                    data.qfrc_applied[idx] = controller.compute(
                        dt, trans_tgt[idx], data.qpos[idx], data.qvel[idx], tgt_vel
                    )
                #collect data
                if count < steps_total:
                    for idx in collect:
                        collect[idx].append(float(data.qvel[idx]))
                    count += 1
            else:
                for idx, controller in enumerate(controllers):
                    data.qfrc_applied[idx] = controllers[idx].compute(
                        dt, target[idx], data.qpos[idx], data.qvel[idx]
                    )
                #collect data
                if count < steps_total:
                    for idx in collect:
                        collect[idx].append(float(data.qvel[idx]))
                    count += 1

            

            
            #update simulation
            mujoco.mj_step(model, data)
            viewer.sync()



            if count == steps_total and not plot_saved:
                times = [i * dt for i in range(len(collect[4]))]
                fig, ax = plt.subplots()
                ax.plot(times, collect[4])
                ax.set_xlabel("Time (s)")
                ax.set_ylabel("Velocity (rad/s)")
                ax.set_title("Joint 4 Velocity")
                out_path = os.path.join(os.path.dirname(__file__), "vel5_graph.png")
                fig.savefig(out_path)
                plt.close(fig)
                print(f"Plot saved to {out_path}")
                plot_saved = True

                




if __name__ == "__main__":
    main()
