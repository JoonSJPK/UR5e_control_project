import csv

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def save_csv(dt, collect, joint_configs, out_path):
    steps = max(len(v) for v in collect.values())
    x_values = [i * dt for i in range(steps)]
    headers = ["time"] + [f"joint_{idx + 1}" for idx, *_ in joint_configs]
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        for i, t in enumerate(x_values):
            writer.writerow([t] + [collect[idx][i] for idx, *_ in joint_configs])


def save_plot(dt, collect, joint_configs, out_path):
    steps = max(len(v) for v in collect.values())
    x_values = [i * dt for i in range(steps)]

    fig, ax = plt.subplots()
    for idx, *_ in joint_configs:
        errors = collect[idx]
        initial_sign = 1.0 if errors[0] >= 0 else -1.0
        overshoot = 0.0
        for e in errors:
            if e * initial_sign < 0 and abs(e) > overshoot:
                overshoot = abs(e)
        ax.plot(x_values, errors, label=f"Joint {idx + 1} (max overshoot: {overshoot:.4f} rad)")

    ax.axhline(y=0, color="r", linestyle="--", label="Zero error")
    ax.set_xlim(0, 10)
    ax.set_xlabel("Time (sec)")
    ax.set_ylabel("Error (rad)")
    ax.set_title("Joint Tracking Error")
    ax.legend()
    plt.tight_layout()
    plt.savefig(out_path)
    plt.close(fig)
