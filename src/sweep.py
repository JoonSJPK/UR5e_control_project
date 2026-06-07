import os

import mujoco
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from controllers import PIDController

SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "models", "universal_robots_ur5e", "scene.xml")


class Sweep:

    def __init__(self, joint_idx=5, duration=10.0, target=-1.5708):
        self.joint_idx = joint_idx
        self.duration = duration
        self.target = target

    def run_headless(self, kp, ki, kd):
        model = mujoco.MjModel.from_xml_path(SCENE_XML)
        data = mujoco.MjData(model)
        dt = model.opt.timestep
        controller = PIDController(Kp=kp, Ki=ki, Kd=kd)
        steps = int(self.duration / dt)
        positions = []
        for _ in range(steps):
            torque = controller.compute(dt, self.target, data.qpos[self.joint_idx], data.qvel[self.joint_idx])
            data.qfrc_applied[self.joint_idx] = torque
            mujoco.mj_step(model, data)
            positions.append(float(data.qpos[self.joint_idx]))
        time_axis = [i * dt for i in range(steps)]
        return time_axis, positions

    def sweep_kp(self, kp_values=range(0, 301, 50), ki=0, kd=0):
        fig, ax = plt.subplots()
        colors = plt.cm.viridis(np.linspace(0, 1, len(kp_values)))

        for kp, color in zip(kp_values, colors):
            t, pos = self.run_headless(kp=kp, ki=ki, kd=kd)
            ax.plot(t, pos, label=f'Kp={kp}', color=color)

        ax.axhline(y=self.target, color='r', linestyle='--', label='Target')
        ax.set_xlim(0, self.duration)
        ax.set_xlabel('Time (sec)')
        ax.set_ylabel('Angle (rad)')
        ax.set_title(f'Joint {self.joint_idx + 1} — Kp Sweep (Ki={ki}, Kd={kd})')
        ax.legend(loc='upper right', fontsize=7)
        plt.tight_layout()
        filename = f'Sweep_Kp0-300_Ki{ki}_Kd{kd}.png'
        plt.savefig(filename)
        plt.close(fig)
        print(f'Saved {filename}')


if __name__ == "__main__":
    Sweep().sweep_kp()
