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
    (150, 1500),
    (150, 1500),
    (150, 1500),
    (30,   300),
    (30,   300),
    (30,   300),
]
KD_RANGES = [
    (15, 150),
    (15, 150),
    (15, 150),
    (3,   30),
    (3,   30),
    (3,   30),
]
KI_RANGES = [
    (50, 300),
    (50, 300),
    (50, 300),
    (10, 100),
    (10, 100),
    (10, 100),
]
# integral_limit expressed as a torque cap (Nm) on the I-term's contribution
# (Ki * integral); converted to the raw integral cap PIDController expects
# via internal_limit = limit_nm / Ki.
LIMIT_RANGES_NM = [
    (30, 100),
    (30, 100),
    (30, 100),
    (5,   18),
    (5,   18),
    (5,   18),
]

BIG_STEP = 20
SMALL_WINDOW = 20
BIG_STEP_KI = 20
SMALL_WINDOW_KI = 20
BIG_STEP_LIMIT = 5
SMALL_WINDOW_LIMIT = 5


def run_headless(joint_idx, kp, kd, duration=DURATION, target=TARGET):
    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data  = mujoco.MjData(model)
    dt    = model.opt.timestep

    for i in range(model.nv):
        if i != joint_idx:
            eq_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, f"lock_{model.joint(i).name}")
            model.eq_active0[eq_id] = 1
            data.eq_active[eq_id]   = 1

    controller  = PIDController(Kp=kp, Ki=0, Kd=kd)
    steps       = int(duration / dt)
    times       = np.arange(steps) * dt
    positions   = np.empty(steps)
    max_torques = np.empty(steps)
    max_torque  = model.actuator_ctrlrange[joint_idx, 1]

    for step in range(steps):
        torque = controller.compute(dt, target, data.qpos[joint_idx], data.qvel[joint_idx])
        max_torques[step] = torque
        applied = torque
        if applied > max_torque:
            applied = max_torque
        elif applied < -max_torque:
            applied = -max_torque
        data.qfrc_applied[joint_idx] = applied
        mujoco.mj_step(model, data)
        positions[step] = data.qpos[joint_idx]

    return times, positions, max_torques, max_torque


def calc_itae(target, positions, torques, tau_max, times):
    error = np.abs(target - positions)
    itae = float(np.sum(times * error * (times[1] - times[0])))

    overage = np.maximum(0.0, np.abs(torques) - tau_max)
    penalty = float(np.sum(overage**2))

    return itae + 0.00000001 * penalty


def run_grid(joint_idx, kp_values, kd_values):
    grid = np.zeros((len(kd_values), len(kp_values)))
    for j, kp in enumerate(kp_values):
        for w, kd in enumerate(kd_values):
            t, pos, max_torques, max_torque = run_headless(joint_idx, kp, kd)
            grid[w, j] = calc_itae(TARGET, pos, max_torques, max_torque, t)
    return grid


def run_headless_ki_limit(joint_idx, kp, kd, ki, limit_nm, duration=DURATION, target=TARGET):
    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data  = mujoco.MjData(model)
    dt    = model.opt.timestep

    for i in range(model.nv):
        if i != joint_idx:
            eq_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, f"lock_{model.joint(i).name}")
            model.eq_active0[eq_id] = 1
            data.eq_active[eq_id]   = 1

    # Convert the torque-unit integral cap into the raw integral cap
    # PIDController clamps (Ki * integral <= limit_nm  =>  integral <= limit_nm / Ki).
    integral_limit = limit_nm / ki if ki else float('inf')
    controller  = PIDController(Kp=kp, Ki=ki, Kd=kd, integral_limit=integral_limit)
    steps       = int(duration / dt)
    times       = np.arange(steps) * dt
    positions   = np.empty(steps)
    max_torques = np.empty(steps)
    max_torque  = model.actuator_ctrlrange[joint_idx, 1]

    for step in range(steps):
        torque = controller.compute(dt, target, data.qpos[joint_idx], data.qvel[joint_idx])
        max_torques[step] = torque
        applied = torque
        if applied > max_torque:
            applied = max_torque
        elif applied < -max_torque:
            applied = -max_torque
        data.qfrc_applied[joint_idx] = applied
        mujoco.mj_step(model, data)
        positions[step] = data.qpos[joint_idx]

    return times, positions, max_torques, max_torque


def run_grid_ki_limit(joint_idx, kp, kd, ki_values, limit_values):
    grid = np.zeros((len(limit_values), len(ki_values)))
    for j, ki in enumerate(ki_values):
        for w, limit_nm in enumerate(limit_values):
            t, pos, max_torques, max_torque = run_headless_ki_limit(joint_idx, kp, kd, ki, limit_nm)
            grid[w, j] = calc_itae(TARGET, pos, max_torques, max_torque, t)
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


def sweep_ki_limit(best_kp_kd, out_path=None, progress_cb=None):
    """best_kp_kd: list of (kp, kd) per joint, e.g. the output of sweep()."""
    if out_path is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path  = f"itae_ki_limit_heatmap_{timestamp}.png"
    n_joints = 6
    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    axes = axes.flatten()

    best_ki_limit = []

    for joint_idx in range(n_joints):
        kp, kd = best_kp_kd[joint_idx]
        ki_lo, ki_hi       = KI_RANGES[joint_idx]
        limit_lo, limit_hi = LIMIT_RANGES_NM[joint_idx]

        if progress_cb:
            progress_cb(f"Sweeping joint {joint_idx+1}/6 Ki/limit (coarse)…")

        # pass 1 big
        ki_big    = np.arange(ki_lo, ki_hi + 1, BIG_STEP_KI)
        limit_big = np.arange(limit_lo, limit_hi + 1, BIG_STEP_LIMIT)
        grid_big  = run_grid_ki_limit(joint_idx, kp, kd, ki_big, limit_big)
        best_w, best_j = np.unravel_index(np.argmin(grid_big), grid_big.shape)
        best_ki_big    = ki_big[best_j]
        best_limit_big = limit_big[best_w]
        print(f"Joint {joint_idx+1} big best: Ki={best_ki_big} limit={best_limit_big} Nm")

        if progress_cb:
            progress_cb(f"Sweeping joint {joint_idx+1}/6 Ki/limit (fine)…")

        # pass 2 small
        ki_small    = np.arange(max(ki_lo, best_ki_big - SMALL_WINDOW_KI),
                                 min(ki_hi, best_ki_big + SMALL_WINDOW_KI) + 1)
        limit_small = np.arange(max(limit_lo, best_limit_big - SMALL_WINDOW_LIMIT),
                                 min(limit_hi, best_limit_big + SMALL_WINDOW_LIMIT) + 1)
        grid_small = run_grid_ki_limit(joint_idx, kp, kd, ki_small, limit_small)
        best_w, best_j = np.unravel_index(np.argmin(grid_small), grid_small.shape)
        best_ki    = ki_small[best_j]
        best_limit = limit_small[best_w]
        print(f"Joint {joint_idx+1}   small best: Ki={best_ki} limit={best_limit} Nm")

        best_ki_limit.append((int(best_ki), int(best_limit)))

        ax = axes[joint_idx]
        im = ax.imshow(grid_small, aspect='auto', origin='lower', cmap='viridis_r')
        fig.colorbar(im, ax=ax, label='ITAE')
        ax.plot(best_j, best_w, 'r*', markersize=12)

        tick_step = max(1, len(ki_small) // 5)
        ax.set_xticks(range(0, len(ki_small), tick_step))
        ax.set_xticklabels(ki_small[::tick_step])
        tick_step = max(1, len(limit_small) // 5)
        ax.set_yticks(range(0, len(limit_small), tick_step))
        ax.set_yticklabels(limit_small[::tick_step])
        ax.set_xlabel('Ki')
        ax.set_ylabel('integral_limit (Nm)')
        ax.set_title(f"Joint {joint_idx+1}: best Ki={best_ki} limit={best_limit} Nm")

    plt.suptitle('ITAE Heatmap Ki/limit (small pass): lower is better (★ = optimal)')
    plt.tight_layout()
    plt.savefig(out_path)
    plt.close(fig)
    print(f"Saved {out_path}")
    return best_ki_limit


if __name__ == "__main__":
    best_gains = sweep()
    sweep_ki_limit(best_gains)
