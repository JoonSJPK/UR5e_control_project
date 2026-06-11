import argparse
import os

import mujoco
import mujoco.viewer
from .controllers import PIDController
from .output import save_csv, save_plot


SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "..", "models", "universal_robots_ur5e", "scene.xml")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--joints", type=str, required=True,
                        help="Comma-separated configs: joint:kp:ki:kd,...")
    parser.add_argument("--out", type=str, default=None, help="Output plot path")
    parser.add_argument("--data-out", type=str, default=None, help="Output CSV data path")
    args = parser.parse_args()

    # Parse "1:100:10:5:-1.5708,3:50:5:2:-1.5708" → list of (0-based idx, kp, ki, kd, target)
    joint_configs = []
    for entry in args.joints.split(","):
        j, kp, ki, kd, tgt = entry.split(":")
        joint_configs.append((int(j) - 1, float(kp), float(ki), float(kd), float(tgt)))

    #scene configuration
    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data = mujoco.MjData(model)
    dt = model.opt.timestep

    #controller objects
    controllers = {}
    targets = {}
    for idx, kp, ki, kd, tgt in joint_configs:
        controllers[idx] = PIDController(Kp=kp, Ki=ki, Kd=kd)
        targets[idx] = tgt

    #variable setup
    steps_total = int(10 / dt)
    collect = {}
    for idx, *_ in joint_configs:
        collect[idx] = []
    count = 0
    prev_time = 0.0
    plot_saved = False

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():

            # Detect viewer reset (Backspace) — data.time jumps back to 0
            if data.time < prev_time:
                for c in controllers.values():
                    c.reset()
                for key in collect:
                    collect[key].clear()
                count = 0
                plot_saved = False
            prev_time = data.time

            #apply torque to each joint
            for idx, controller in controllers.items():
                data.ctrl[idx] = data.qpos[idx] 
                data.qfrc_applied[idx] = controller.compute(
                    dt, targets[idx], data.qpos[idx], data.qvel[idx]
                ) + data.qfrc_bias[idx]
            
            #update simulation
            mujoco.mj_step(model, data)
            viewer.sync()

            #collect data
            if count < steps_total:
                for idx in collect:
                    collect[idx].append(float(data.qpos[idx]))
                count += 1

            if count == steps_total and not plot_saved:
                out_path = args.out or os.path.join(os.path.dirname(__file__), "matplotlib_graph.png")
                save_plot(dt, collect, joint_configs, out_path)
                if args.data_out:
                    save_csv(dt, collect, joint_configs, args.data_out)
                plot_saved = True


if __name__ == "__main__":
    main()
