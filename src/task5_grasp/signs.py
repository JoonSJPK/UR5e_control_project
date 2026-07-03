from __future__ import annotations

import json

import mujoco
import numpy as np

from .joint_map import JointMap, SERVO_JOINT, SIGN_FILE, SCENE
from .servo_bus import ServoBus

DIRECT = (1, 5)


def signed_delta(now, ref):
    return ((now - ref + 2048) % 4096) - 2048


def main():
    jm = JointMap()
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    home = model.key("home").qpos
    reach_angle = {}
    for sid in SERVO_JOINT:
        qadr = model.joint(SERVO_JOINT[sid]).qposadr[0]
        reach_angle[sid] = float(home[qadr])

    bus = ServoBus()
    bus.set_torque_all(False)
    input("Torque off. Fold arm to rest, enter ")
    base_tick = {}
    for sid in SERVO_JOINT:
        base_tick[sid] = bus.read_position(sid)

    signs = dict(jm.sign)
    print("Move one joint at a time in its positive direction, then press Enter.")
    input("Joint 1 (Rotation): swing base left (from behind) ")
    signs[1] = int(np.sign(signed_delta(bus.read_position(1), base_tick[1])))
    input("Fold back to rest, enter ")
    input("Joint 5 (Wrist_Roll): rotate wrist CCW ")
    signs[5] = int(np.sign(signed_delta(bus.read_position(5), base_tick[5])))
    input("Fold back to rest, Enter... ")

    input("Hold reaching pose (forearm horizontal, gripper down), Enter ")
    for sid in SERVO_JOINT:
        if sid in DIRECT:
            continue
        d_ang = reach_angle[sid] - jm.home_angle[sid]
        if abs(np.degrees(d_ang)) < 10:
            continue
        d_tick = signed_delta(bus.read_position(sid), jm._c(sid)["home"])
        signs[sid] = int(np.sign(d_tick / d_ang))
    bus.close()

    SIGN_FILE.write_text(json.dumps({str(k): v for k, v in signs.items()}, indent=2))
    print(f"saved {signs} -> {SIGN_FILE}")


if __name__ == "__main__":
    main()
