from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CALIB = ROOT / "docs" / "task5_data" / "calibration.json"
SIGN_FILE = ROOT / "docs" / "task5_data" / "joint_sign.json"
SCENE = ROOT / "models" / "trs_so_arm100" / "scene_so101.xml"

TICKS_PER_RAD = 4096.0 / (2 * np.pi)

SERVO_JOINT = {1: "Rotation", 2: "Pitch", 3: "Elbow",
               4: "Wrist_Pitch", 5: "Wrist_Roll", 6: "Jaw"}


SIGN = {1: +1, 2: +1, 3: +1, 4: +1, 5: +1, 6: +1}


class JointMap:
    def __init__(self):
        self.calib = json.loads(CALIB.read_text())
        m = mujoco.MjModel.from_xml_path(str(SCENE))
        rest = m.key("rest").qpos
        self.home_angle = {sid: float(rest[m.joint(SERVO_JOINT[sid]).qposadr[0]])
                           for sid in SERVO_JOINT}
        self.sign = dict(SIGN)
        if SIGN_FILE.exists(): 
            self.sign.update({int(k): v for k, v in
                              json.loads(SIGN_FILE.read_text()).items()})

    @staticmethod
    def short_path(cur_raw, target_cont, step=6):
        d = ((cur_raw - target_cont + 2048) % 4096) - 2048
        cur_cont = target_cont + d
        n = max(1, int(abs(target_cont - cur_cont) / step))
        return [int(round(cur_cont + (target_cont - cur_cont) * k / n)) % 4096
                for k in range(1, n + 1)]

    def _c(self, sid):
        return self.calib[str(sid)]

    def _raw_tick(self, sid, angle):
        c = self._c(sid)
        return c["home"] + self.sign[sid] * (angle - self.home_angle[sid]) * TICKS_PER_RAD

    def _range(self, sid):
        c = self._c(sid)
        return c["home"] + c["min_off"], c["home"] + c["max_off"]

    def angle_to_tick(self, sid, angle):
        lo, hi = self._range(sid)
        return int(round(min(max(self._raw_tick(sid, angle), lo), hi)))

    def tick_to_angle(self, sid, tick):
        c = self._c(sid)
        return self.home_angle[sid] + self.sign[sid] * (tick - c["home"]) / TICKS_PER_RAD

    def clamps_hit(self, sid, angle):
        lo, hi = self._range(sid)
        return not (lo <= self._raw_tick(sid, angle) <= hi)


if __name__ == "__main__":
    jm = JointMap()
    print("servo | home_tick | home_angle(rad) | safe tick range")
    for sid in SERVO_JOINT:
        c = jm._c(sid)
        print(f"  {sid} {SERVO_JOINT[sid]:12} | {c['home']:5} | "
              f"{jm.home_angle[sid]:+.3f} | [{c['home']+c['min_off']}, {c['home']+c['max_off']}]")
