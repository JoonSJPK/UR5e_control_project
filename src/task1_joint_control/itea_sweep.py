import os
from datetime import datetime

import mujoco
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from .controllers import PIDController

SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "..", "models", "universal_robots_ur5e", "scene.xml")

TARGET   = -1.5708
DURATION = 10.0

KP_RANGES = [
    (20, 400),
    (20, 400),
    (20, 400),
    (5,   60),
    (5,   60),
    (5,   60),
]
KD_RANGES = [
    (5,  65),
    (5,  65),
    (2,  35),
    (1,   7),
    (1,   7),
    (1,   7),
]

BIG_STEP = 20
SMALL_WINDOW = 20


def run_headless(joint_idx, kp, kd, duration=DURATION, target=TARGET):
    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data  = mujoco.MjData(model)
    dt    = model.opt.timestep

    for i in range(model.nv):
        if i != joint_idx:
            eq_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, f"lock_{model.joint(i).name}")
            model.eq_active0[eq_id] = 1
            data.eq_active[eq_id]   = 1

    controller = PIDController(Kp=kp, Ki=0, Kd=kd)
    steps      = int(duration / dt)
    times      = np.arange(steps) * dt
    positions  = np.empty(steps)

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


def run_grid(joint_idx, kp_values, kd_values):
    grid = np.zeros((len(kd_values), len(kp_values)))
    for j, kp in enumerate(kp_values):
        for w, kd in enumerate(kd_values):
            t, pos       = run_headless(joint_idx, kp, kd)
            grid[w, j]   = calc_itae(t, pos, TARGET)
    return grid


def sweep(out_path=None, progress_cb=None):
    if out_path is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path  = f"itae_heatmap_{timestamp}.png"
    n_joints = 6
    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    axes = axes.flatten()

    best_gains = []

    for joint_idx in range(n_joints):
        kp_lo, kp_hi = KP_RANGES[joint_idx]
        kd_lo, kd_hi = KD_RANGES[joint_idx]

        if progress_cb:
            progress_cb(f"Sweeping joint {joint_idx+1}/6 (coarse)…")

        # pass 1 big
        kp_big = np.arange(kp_lo, kp_hi + 1, BIG_STEP)
        kd_big = np.arange(kd_lo, kd_hi + 1, BIG_STEP)
        grid_big = run_grid(joint_idx, kp_big, kd_big)
        best_w, best_j = np.unravel_index(np.argmin(grid_big), grid_big.shape)
        best_kp_big = kp_big[best_j]
        best_kd_big = kd_big[best_w]
        print(f"Joint {joint_idx+1} big best: Kp={best_kp_big} Kd={best_kd_big}")

        if progress_cb:
            progress_cb(f"Sweeping joint {joint_idx+1}/6 (fine)…")

        # pass 2 small
        kp_small = np.arange(max(kp_lo, best_kp_big - SMALL_WINDOW),
                            min(kp_hi, best_kp_big + SMALL_WINDOW) + 1)
        kd_small = np.arange(max(kd_lo, best_kd_big - SMALL_WINDOW),
                            min(kd_hi, best_kd_big + SMALL_WINDOW) + 1)
        grid_small = run_grid(joint_idx, kp_small, kd_small)
        best_w, best_j = np.unravel_index(np.argmin(grid_small), grid_small.shape)
        best_kp = kp_small[best_j]
        best_kd = kd_small[best_w]
        print(f"Joint {joint_idx+1}   small best: Kp={best_kp} Kd={best_kd}")

        best_gains.append((int(best_kp), int(best_kd)))

        ax = axes[joint_idx]
        im = ax.imshow(grid_small, aspect='auto', origin='lower', cmap='viridis_r')
        fig.colorbar(im, ax=ax, label='ITAE')
        ax.plot(best_j, best_w, 'r*', markersize=12)

        tick_step = max(1, len(kp_small) // 5)
        ax.set_xticks(range(0, len(kp_small), tick_step))
        ax.set_xticklabels(kp_small[::tick_step])
        tick_step = max(1, len(kd_small) // 5)
        ax.set_yticks(range(0, len(kd_small), tick_step))
        ax.set_yticklabels(kd_small[::tick_step])
        ax.set_xlabel('Kp')
        ax.set_ylabel('Kd')
        ax.set_title(f"Joint {joint_idx+1}: best Kp={best_kp} Kd={best_kd}")

    plt.suptitle('ITAE Heatmap (small pass): lower is better (★ = optimal)')
    plt.tight_layout()
    plt.savefig(out_path)
    plt.close(fig)
    print(f"Saved {out_path}")
    return best_gains


if __name__ == "__main__":
    sweep()
