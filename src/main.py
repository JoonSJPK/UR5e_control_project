import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
from controllers import PIDController


SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "models", "universal_robots_ur5e", "scene.xml")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--joints", type=str, required=True,
                        help="Comma-separated configs: joint:kp:ki:kd,...")
    parser.add_argument("--out", type=str, default=None, help="Output plot path")
    args = parser.parse_args()

    # Parse "1:100:10:5,3:50:5:2" → list of (0-based idx, kp, ki, kd)
    joint_configs = []
    for entry in args.joints.split(","):
        j, kp, ki, kd = entry.split(":")
        joint_configs.append((int(j) - 1, float(kp), float(ki), float(kd)))

    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data = mujoco.MjData(model)
    dt = model.opt.timestep

    controllers = {idx: PIDController(Kp=kp, Ki=ki, Kd=kd) for idx, kp, ki, kd in joint_configs}

    target = -1.5708
    steps_total = int(10 / dt)
    collect = {idx: [] for idx, *_ in joint_configs}
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

            for idx, controller in controllers.items():
                data.qfrc_applied[idx] = controller.compute(
                    dt, target, data.qpos[idx], data.qvel[idx]
                )

            mujoco.mj_step(model, data)
            viewer.sync()

            if count < steps_total:
                for idx in collect:
                    collect[idx].append(float(data.qpos[idx]))
                count += 1

            if count == steps_total and not plot_saved:
                out_path = args.out or os.path.join(os.path.dirname(__file__), "matplotlib_graph.png")
                _save_plot(dt, collect, target, joint_configs, out_path)
                plot_saved = True


def _save_plot(dt, collect, target, joint_configs, out_path):
    steps = max(len(v) for v in collect.values())
    x_values = [i * dt for i in range(steps)]

    fig, ax = plt.subplots()
    for idx, kp, ki, kd in joint_configs:
        ax.plot(x_values, collect[idx], label=f"Joint {idx + 1}")

    ax.axhline(y=target, color="r", linestyle="--", label="Target")
    ax.set_xlim(0, 10)
    ax.set_xlabel("Time (sec)")
    ax.set_ylabel("Angle (rad)")
    ax.set_title("Joint Tracking")
    ax.legend()
    plt.tight_layout()
    plt.savefig(out_path)
    plt.close(fig)


if __name__ == "__main__":
    main()
