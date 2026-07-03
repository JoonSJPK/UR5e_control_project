from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

try:
    from .ik_math import SOArm100Kinematics, ik_step
except ImportError:  # allow running as a plain script
    from ik_math import SOArm100Kinematics, ik_step

ROOT = Path(__file__).resolve().parents[2]
SCENE = ROOT / "models" / "trs_so_arm100" / "scene_so101.xml"
OUT = ROOT / "docs" / "task5_data" / "sim"

ARM = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"]  # 5 positioning DOF
JAW = "Jaw"

R_D = np.array([[1.0, 0.0,  0.0],
                [0.0, 0.0, -1.0],
                [0.0, 1.0,  0.0]])

FINGER_OFFSET = np.array([-0.017, -0.08, 0.0])


class Sim:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(SCENE))
        self.d = mujoco.MjData(self.m)
        self.jaw_body = self.m.body("Fixed_Jaw").id
        self.arm_qadr = [self.m.joint(n).qposadr[0] for n in ARM]
        self.arm_act = [self.m.actuator(n).id for n in ARM]
        self.jaw_qadr = self.m.joint(JAW).qposadr[0]
        self.jaw_act = self.m.actuator(JAW).id
        self.cube_qadr = self.m.joint("cube_free").qposadr[0]
        self.kin = SOArm100Kinematics(FINGER_OFFSET)

    def solve_ik(self, target, iters=300, pos_tol=1e-3, ori_tol=0.02,
                 K_p=1.0, K_o=0.5, lambda_squared=0.08 ** 2):
        q = np.array([self.d.qpos[a] for a in self.arm_qadr])
        for _ in range(iters):
            dq, e_p_mag, e_o_mag = ik_step(self.kin, q, target, R_D, K_p, K_o, lambda_squared)
            if e_p_mag < pos_tol and e_o_mag < ori_tol:
                break
            q = q + dq
            for i in range(len(ARM)):
                lo, hi = self.m.jnt_range[self.m.joint(ARM[i]).id]
                q[i] = np.clip(q[i], lo, hi)
        p_grasp, _ = self.kin.grasp_pose(q)
        residual = np.linalg.norm(target - p_grasp)
        return q, residual

    def move_to(self, arm_target, jaw_target, steps=500):
        for i, act in enumerate(self.arm_act):
            self.d.ctrl[act] = arm_target[i]
        self.d.ctrl[self.jaw_act] = jaw_target
        for _ in range(steps):
            mujoco.mj_step(self.m, self.d)

    def hold_current(self):
        for adr, act in zip(self.arm_qadr, self.arm_act):
            self.d.ctrl[act] = self.d.qpos[adr]

    def render(self, name):
        r = mujoco.Renderer(self.m, height=720, width=1280)
        mujoco.mj_forward(self.m, self.d)
        r.update_scene(self.d, camera=-1)
        import cv2
        OUT.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(OUT / f"{name}.png"), cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR))


def main():
    sim = Sim()
    m, d = sim.m, sim.d

    mujoco.mj_resetDataKeyframe(m, d, m.key("home").id)
    cube_xy = np.array([0.0, -0.22]) 
    d.qpos[sim.cube_qadr:sim.cube_qadr + 3] = [cube_xy[0], cube_xy[1], 0.025]
    d.qpos[sim.cube_qadr + 3:sim.cube_qadr + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    cube_z0 = d.qpos[sim.cube_qadr + 2]
    print(f"cube start height: {cube_z0*1000:.1f} mm")

    JAW_OPEN, JAW_CLOSED = 0.9, 0.0   

    target = np.array([cube_xy[0], cube_xy[1], 0.023])
    pre = np.array([cube_xy[0], cube_xy[1], 0.025 + 0.08]) 

    sim.hold_current()

    # reach pre-grasp with jaw open
    pre_q, err = sim.solve_ik(pre)
    print(f"pre-grasp IK residual: {err*1000:.1f} mm")
    sim.move_to(pre_q, JAW_OPEN, steps=800)

    # descend onto the cube
    grasp_q, err = sim.solve_ik(target)
    print(f"grasp IK residual: {err*1000:.1f} mm")
    sim.move_to(grasp_q, JAW_OPEN, steps=600)

    # close the jaw on the cube
    sim.move_to(grasp_q, JAW_CLOSED, steps=800)

    # lift straight back up to pre-grasp height, keeping the jaw closed
    sim.move_to(pre_q, JAW_CLOSED, steps=800)

    cube_z1 = d.qpos[sim.cube_qadr + 2]
    print(f"cube end height:   {cube_z1*1000:.1f} mm")
    lifted = cube_z1 - cube_z0
    print(f"lifted: {lifted*1000:+.1f} mm -> {'GRASP OK' if lifted > 0.03 else 'FAILED'}")
    sim.render("after_grasp")


if __name__ == "__main__":
    main()
