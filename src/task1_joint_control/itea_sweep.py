import os

import mujoco
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from .controllers import PIDController

SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "..", "models", "universal_robots_ur5e", "scene.xml")

KP_VALUES = [50, 100, 200, 350, 500]
KD_VALUES = [5, 14, 28, 42, 60]
TARGET     = -1.5708
DURATION   = 10.0


def run_headless(joint_idx, kp, kd, duration=DURATION, target=TARGET):
    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data = mujoco.MjData(model)
    dt = model.opt.timestep

    for i in range(model.nv):
        if i != joint_idx:
            eq_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, f"lock_{model.joint(i).name}")
            model.eq_active0[eq_id] = 1
            data.eq_active[eq_id] = 1

    controller = PIDController(Kp=kp, Ki=0, Kd=kd)
    steps = int(duration / dt)
    times = np.arange(steps) * dt
    positions = np.empty(steps)

    for step in range(steps):
        limit  = model.actuator_ctrlrange[joint_idx, 1]
        torque = controller.compute(dt, target, data.qpos[joint_idx], data.qvel[joint_idx])
        torque = np.clip(torque, -limit, limit)
        data.qfrc_applied[joint_idx] = torque + data.qfrc_bias[joint_idx]
        mujoco.mj_step(model, data)
        positions[step] = data.qpos[joint_idx]

    return times, positions


def calc_itae(times, positions, target):
    error = np.abs(target - positions)
    return float(np.sum(times * error * (times[1] - times[0])))


def sweep(out_path="itae_heatmap.png"):
    n_joints = 6
    n_kp = len(KP_VALUES)
    n_kd = len(KD_VALUES)

    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    axes = axes.flatten()

    for joint_idx in range(n_joints):
        itae_grid = np.zeros((n_kd, n_kp))

        for j, kp in enumerate(KP_VALUES):
            for w, kd in enumerate(KD_VALUES):
                t, pos = run_headless(joint_idx, kp, kd)
                itae_grid[w, j] = calc_itae(t, pos, TARGET)
                print(f"Joint {joint_idx+1} Kp={kp} Kd={kd} ITAE={itae_grid[w,j]:.4f}")

        ax  = axes[joint_idx]
        im  = ax.imshow(itae_grid, aspect='auto', origin='lower', cmap='viridis_r')
        fig.colorbar(im, ax=ax, label='ITAE')

        best_w, best_j = np.unravel_index(np.argmin(itae_grid), itae_grid.shape)
        ax.plot(best_j, best_w, 'r*', markersize=12)

        ax.set_xticks(range(n_kp))
        ax.set_xticklabels(KP_VALUES)
        ax.set_yticks(range(n_kd))
        ax.set_yticklabels(KD_VALUES)
        ax.set_xlabel('Kp')
        ax.set_ylabel('Kd')
        ax.set_title(f"Joint {joint_idx+1} — best Kp={KP_VALUES[best_j]} Kd={KD_VALUES[best_w]}")

    plt.suptitle('ITAE Heatmap: lower is better (★ = optimal)')
    plt.tight_layout()
    plt.savefig(out_path)
    plt.close(fig)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    sweep()
