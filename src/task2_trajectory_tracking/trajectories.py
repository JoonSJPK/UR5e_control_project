import os
import sys
from datetime import datetime

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pid_controller import make_joint_controller
from trajectory_generator import TrajectoryGenerator

SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "..", "models", "universal_robots_ur5e", "scene.xml")

def main():

  model = mujoco.MjModel.from_xml_path(SCENE_XML)
  data = mujoco.MjData(model)
  dt = model.opt.timestep

  init  = [-1.5708, -1.5708, -1.5708, -1.5708, -1.5708, -1.5708]
  target = [1.5708, -3.1416, 1.5708, -3.1416, 1.5708, -3.1416]


  controllers = []
  trajgen = []
  for idx in range(6):
     controllers.append(make_joint_controller(idx))
     trajgen.append(TrajectoryGenerator(target = target[idx], init = init[idx]))

  prev_time = 0.0

  trans_time = dt * 4000

  #variable setup
  steps_total = int(10 / dt)
  collect_vel = {idx: [] for idx in range(6)}
  collect_pos = {idx: [] for idx in range(6)}
  collect_tgt_vel = {idx: [] for idx in range(6)}
  collect_tgt_pos = {idx: [] for idx in range(6)}
  collect_vel_error = {idx: [] for idx in range(6)}
      
  count = 0
  plot_saved = False

  with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():

            if data.time < prev_time:
                for controller in controllers:
                    controller.reset()
            prev_time = data.time

            if( data.time >= 0.0 and data.time < trans_time ):
                for idx, controller in enumerate(controllers):
                    torque = controller.compute(
                        dt, init[idx], data.qpos[idx], data.qvel[idx]
                    )
                    limit = model.actuator_ctrlrange[idx, 1]
                    data.qfrc_applied[idx] = np.clip(torque, -limit, limit)
            elif( data.time >= trans_time and data.time <= trans_time + 8):
                trans_tgt = []
                for idx, controller in enumerate(controllers):
                    curr_time = data.time - trans_time
                    trans_tgt.append(trajgen[idx].compute_tgt_pos(curr_time))

                for idx, controller in enumerate(controllers):
                    tgt_vel = trajgen[idx].compute_tgt_vel(curr_time)
                    torque = controller.compute(
                        dt, trans_tgt[idx], data.qpos[idx], data.qvel[idx], tgt_vel
                    )
                    limit = model.actuator_ctrlrange[idx, 1]
                    if limit < np.abs(torque):
                        print(f"joint {idx} saturated: {torque:.1f} > {limit:.1f} N·m")

                    data.qfrc_applied[idx] = np.clip(torque, -limit, limit)
                #collect data
                if count < steps_total:
                    for idx in range(6):
                        vel_error = np.abs(trajgen[idx].compute_tgt_vel(curr_time) - data.qvel[idx])
                        collect_vel[idx].append(float(data.qvel[idx]))
                        collect_pos[idx].append(float(data.qpos[idx]))
                        collect_tgt_vel[idx].append(trajgen[idx].compute_tgt_vel(curr_time))
                        collect_tgt_pos[idx].append(trans_tgt[idx])
                        collect_vel_error[idx].append(vel_error)
                    count += 1
            else:
                for idx, controller in enumerate(controllers):
                    torque = controllers[idx].compute(
                        dt, target[idx], data.qpos[idx], data.qvel[idx]
                    )
                    limit = model.actuator_ctrlrange[idx, 1]
                    data.qfrc_applied[idx] = np.clip(torque, -limit, limit)
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
                    if collect_vel_error[idx]:
                        avg_err = np.mean(collect_vel_error[idx])
                        max_err = np.max(collect_vel_error[idx])
                        vel_range = np.max(np.abs(collect_tgt_vel[idx])) or 1.0
                        alignment_pct = max(0.0, 1.0 - avg_err / vel_range) * 100.0
                        axes[idx, 1].text(
                            0.02, 0.95,
                            f"avg err: {avg_err:.4f} rad/s\nmax err: {max_err:.4f} rad/s\n"
                            f"alignment: {alignment_pct:.1f}%",
                            transform=axes[idx, 1].transAxes,
                            va="top", ha="left",
                            fontsize=8,
                            bbox=dict(boxstyle="round", facecolor="white", alpha=0.7),
                        )
                    axes[idx, 1].legend()
                for ax in axes[-1]:
                    ax.set_xlabel("Time (s)")
                all_vel_errors = [err for idx in range(6) for err in collect_vel_error[idx]]
                all_tgt_vels = [v for idx in range(6) for v in collect_tgt_vel[idx]]
                if all_vel_errors:
                    overall_avg_err = np.mean(all_vel_errors)
                    overall_max_err = np.max(all_vel_errors)
                    overall_vel_range = np.max(np.abs(all_tgt_vels)) or 1.0
                    overall_alignment_pct = max(0.0, 1.0 - overall_avg_err / overall_vel_range) * 100.0
                    fig.suptitle(
                        f"Overall (all 6 joints) avg err: {overall_avg_err:.4f} rad/s | "
                        f"max err: {overall_max_err:.4f} rad/s | "
                        f"alignment: {overall_alignment_pct:.1f}%"
                    )
                plt.tight_layout(rect=(0, 0, 1, 0.97))
                out_path = os.path.join(os.path.dirname(__file__), f"vel_6_joints_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
                fig.savefig(out_path)
                plt.close(fig)
                print(f"Plot saved to {out_path}")
                plot_saved = True

                




if __name__ == "__main__":
    main()
