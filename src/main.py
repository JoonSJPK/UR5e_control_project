import argparse
import csv
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
    parser.add_argument("--data-out", type=str, default=None, help="Output CSV data path")
    args = parser.parse_args()

    # Parse "1:100:10:5:-1.5708,3:50:5:2:-1.5708" → list of (0-based idx, kp, ki, kd, target)
    joint_configs = []
    for entry in args.joints.split(","):
        j, kp, ki, kd, tgt = entry.split(":")
        joint_configs.append((int(j) - 1, float(kp), float(ki), float(kd), float(tgt)))

    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data = mujoco.MjData(model)
    dt = model.opt.timestep

    controllers = {idx: PIDController(Kp=kp, Ki=ki, Kd=kd) for idx, kp, ki, kd, _ in joint_configs}
    targets = {idx: tgt for idx, *_, tgt in joint_configs}

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
                    dt, targets[idx], data.qpos[idx], data.qvel[idx]
                )

            mujoco.mj_step(model, data)
            viewer.sync()

            if count < steps_total:
                for idx in collect:
                    collect[idx].append(float(data.qpos[idx]))
                count += 1

            if count == steps_total and not plot_saved:
                out_path = args.out or os.path.join(os.path.dirname(__file__), "matplotlib_graph.png")
                _save_plot(dt, collect, joint_configs, out_path)
                if args.data_out:
                    _save_csv(dt, collect, joint_configs, args.data_out)
                plot_saved = True


def _save_csv(dt, collect, joint_configs, out_path):
    steps = max(len(v) for v in collect.values())
    x_values = [i * dt for i in range(steps)]
    headers = ["time"] + [f"joint_{idx + 1}" for idx, *_ in joint_configs]
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        for i, t in enumerate(x_values):
            writer.writerow([t] + [collect[idx][i] for idx, *_ in joint_configs])


def _save_plot(dt, collect, joint_configs, out_path):
    steps = max(len(v) for v in collect.values())
    x_values = [i * dt for i in range(steps)]

    fig, ax = plt.subplots()
    plotted_targets = set()
    for idx, kp, ki, kd, tgt in joint_configs:
        ax.plot(x_values, collect[idx], label=f"Joint {idx + 1}")
        if tgt not in plotted_targets:
            ax.axhline(y=tgt, color="r", linestyle="--",
                       label=f"Target ({tgt:.4f})" if len(plotted_targets) == 0 else f"Target ({tgt:.4f})")
            plotted_targets.add(tgt)

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
