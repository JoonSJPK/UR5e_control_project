from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from . import detect_color
from .joint_map import JointMap, SERVO_JOINT
from .sensing import load_homography, pixel_to_table
from .servo_bus import ServoBus
from .sim_grasp import ROOT, Sim

ARM_IDS = [1, 2, 3, 4, 5]
GRIP_ID = 6
JAW_OPEN, JAW_CLOSED = 0.9, 0.0
CAM_INDEX = 0

STEP_TICKS = 6       
STEP_DELAY = 0.02
SPEED_REG = 200


def user_xy_to_mujoco(X_mm, Y_mm, z=0.025):
    return np.array([Y_mm / 1000.0, -X_mm / 1000.0, z])


def detect_cube_pixel(cap, tries=8, save_overlay=None, H=None):
    dets, last = [], None
    for _ in range(tries):
        ok, frame = cap.read()
        if not ok:
            continue
        last = frame
        d = detect_color.detect(frame)
        if d is not None:
            dets.append(d)
    if not dets:
        return None
    u = float(np.median([d[0] for d in dets]))
    v = float(np.median([d[1] for d in dets]))
    if save_overlay is not None and last is not None:
        img = detect_color.draw(last.copy(), dets[-1])
        if H is not None:
            X, Y = pixel_to_table(H, u, v)
            cv2.putText(img, f"({u:.0f},{v:.0f}) -> X={X:.0f} Y={Y:.0f}mm", (int(u) + 8, int(v)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imwrite(str(save_overlay), img)
        print(f"detection overlay saved -> {save_overlay}")
    return (u, v, len(dets))


def plan(sim, target_mj):
    import mujoco
    mujoco.mj_resetDataKeyframe(sim.m, sim.d, sim.m.key("home").id)  # reaching seed
    pre_q, e1 = sim.solve_ik(target_mj + [0, 0, 0.08])
    grasp_q, e2 = sim.solve_ik(target_mj)
    to_dict = lambda q: {sid: float(q[i]) for i, sid in enumerate(ARM_IDS)}
    return to_dict(pre_q), to_dict(grasp_q), e1, e2


def move_to(bus, jm, targets, hold=None):
    targets = dict(targets)
    if hold:
        for s, t in hold.items():
            targets.setdefault(s, t)
    ids = list(targets)
    seg, n = {}, 1
    for s in ids:
        cur = bus.read_position(s)
        d = ((cur - targets[s] + 2048) % 4096) - 2048   # current near target (continuous)
        cur_cont = targets[s] + d
        seg[s] = (cur_cont, targets[s])
        n = max(n, int(abs(targets[s] - cur_cont) / STEP_TICKS))
    for k in range(1, n + 1):
        for s in ids:
            cc, tc = seg[s]
            bus.write_goal(s, int(round(cc + (tc - cc) * k / n)) % 4096)
        time.sleep(STEP_DELAY)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="plan only, no motion")
    ap.add_argument("--hover", action="store_true", help="reach above cube, then stop")
    args = ap.parse_args()

    jm = JointMap()
    sim = Sim()
    H = load_homography()

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera {CAM_INDEX}")
    overlay = ROOT / "docs" / "task5_data" / "dryrun_view.png" if args.dry_run else None
    det = detect_cube_pixel(cap, save_overlay=overlay, H=H)
    cap.release()
    if det is None:
        raise SystemExit("no orange block detected")
    u, v, n = det
    X, Y = pixel_to_table(H, u, v)
    target_mj = user_xy_to_mujoco(X, Y)
    print(f"block pixel=({u:.0f},{v:.0f}) [{n} frames] -> table X={X:.0f} Y={Y:.0f} mm")

    pre_q, grasp_q, e1, e2 = plan(sim, target_mj)
    print(f"IK residuals: pre {e1*1000:.1f} mm, grasp {e2*1000:.1f} mm")
    print("planned servo ticks:")
    for sid in ARM_IDS:
        pt = jm.angle_to_tick(sid, pre_q[sid]); gt = jm.angle_to_tick(sid, grasp_q[sid])
        flag = "  <-- CLAMPED" if (jm.clamps_hit(sid, pre_q[sid]) or
                                   jm.clamps_hit(sid, grasp_q[sid])) else ""
        print(f"  {sid} {SERVO_JOINT[sid]:12} pre={pt:4d} grasp={gt:4d} sign={jm.sign[sid]:+d}{flag}")

    if e1 > 0.01 or e2 > 0.01:
        print("WARNING: IK residual high - target may be out of reach.")
    if args.dry_run:
        print("dry-run: no motion."); return
    if input("proceed to MOVE the real arm? [y/N] ").strip().lower() != "y":
        print("aborted."); return

    bus = ServoBus()
    bus.set_torque_all(True)
    for sid in SERVO_JOINT:
        bus.set_speed(sid, SPEED_REG)

    # 1) open gripper + reach pre-grasp
    move_to(bus, jm, {**{s: jm.angle_to_tick(s, pre_q[s]) for s in ARM_IDS},
                      GRIP_ID: jm.angle_to_tick(GRIP_ID, JAW_OPEN)})
    if args.hover:
        print("hovering above cube. Check aim, then Ctrl-C / power off if wrong."); bus.close(); return

    # 2) descend  3) close  4) lift
    move_to(bus, jm, {s: jm.angle_to_tick(s, grasp_q[s]) for s in ARM_IDS})
    move_to(bus, jm, {GRIP_ID: jm.angle_to_tick(GRIP_ID, JAW_CLOSED)}); time.sleep(0.5)
    move_to(bus, jm, {s: jm.angle_to_tick(s, pre_q[s]) for s in ARM_IDS},
            hold={GRIP_ID: jm.angle_to_tick(GRIP_ID, JAW_CLOSED)})
    print("done - cube should be lifted. 'home' in jog.py to reset.")
    bus.close()


if __name__ == "__main__":
    main()
