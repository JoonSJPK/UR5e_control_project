from __future__ import annotations

import json
import time
from pathlib import Path

from .servo_bus import ServoBus, JOINT_NAMES, ADDR_TORQUE_ENABLE

import sys
import select

SET_MIDDLE = 128
ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "docs" / "task5_data" / "calibration.json"


def wrap_delta(d):
    d %= 4096
    if d > 2048:
        d -= 4096
    return d

def wait_enter():
    sys.stdin.readline()

def sweep_to_stop(bus, sid, start):
    cont= 0
    prev = start
    print("move to other end, Enter at end")
    while True:
        cur = bus.read_position(sid)
        if cur is not None:
            cont += wrap_delta(cur - prev)
            prev = cur
            print(f"\r  swept {cont:+5d} ticks   ", end="")
        if select.select([sys.stdin], [], [], 0.03)[0]:
            wait_enter()
            return cont


def sample_until_enter(bus, ids, home, min_off, max_off):
    cont = {}
    prev = {}
    for sid in ids:
        cont[sid] = 0
        prev[sid] = home[sid]

    print("moving right now (press Enter to stop)")
    while True:
        for sid in ids:
            raw = bus.read_position(sid)
            if raw is None:
                continue
            cont[sid] += wrap_delta(raw - prev[sid])
            prev[sid] = raw
            min_off[sid] = min(min_off[sid], cont[sid])
            max_off[sid] = max(max_off[sid], cont[sid])
        if select.select([sys.stdin], [], [], 0)[0]:
            wait_enter()
            break
        time.sleep(0.02)

def park(bus, sid, target):
    while True:
        cur = bus.read_position(sid)
        if cur is not None:
            off = wrap_delta(cur - target)
            tag = "OK press Enter" if abs(off) < 40 else "keep moving"
            print(f"\r  current={cur} target={target} off={off:+5d}  {tag}      ", end="")
        if select.select([sys.stdin], [], [], 0.05)[0]:
            wait_enter()
            return

def re_zero(bus):
    bus.set_torque_all(False)
    print("Torque OFF. Move each joint by hand as prompted.\n")

    for sid, name in JOINT_NAMES.items():
        print(f"servo {sid} {name}")
        input("move to one end, then Enter")
        a = bus.read_position(sid)
        if a is None:
            print("read failed; skipping this joint"); continue
        swept = sweep_to_stop(bus, sid, a)
        span = abs(swept)
        mid = (a + swept // 2) % 4096
        print(f"\n  span {span} ticks. Now go to the MIDDLE (~{mid}):")
        park(bus, sid, mid)
        bus.packet.write1ByteTxRx(bus.port, sid, ADDR_TORQUE_ENABLE, SET_MIDDLE)
        time.sleep(0.6)
        bus.set_torque(sid, False)
        print()

def home_pose(bus, ids):
    print("1. home: hold the arm at its initial pose.")
    input("Press Enter to record pose")
    home = {}
    for sid in ids:
        home[sid] = bus.read_position(sid)

    for sid in ids:
        print(f"  {sid} {JOINT_NAMES[sid]:12}: home={home[sid]}")

    return home

def main():
    ids = list(JOINT_NAMES)
    bus = ServoBus()

    re_zero(bus)

    input("Calibration. Enter to DISABLE torque")
    bus.set_torque_all(False)
    print("torque off\n")

    home = home_pose(bus, ids)

    min_off = {}
    max_off = {}
    for sid in ids:
        min_off[sid] = 0
        max_off[sid] = 0

    print("\n2. range: starting from home, move all joint to")
    print("its full safe range")
    print("Return to home after")
    sample_until_enter(bus, ids, home, min_off, max_off)
    for sid in ids:
        print(f"  {sid} {JOINT_NAMES[sid]:12}: min_off={min_off[sid]:+5d} "
              f"max_off={max_off[sid]:+5d} span={max_off[sid]-min_off[sid]}")

    calibration = {
        str(sid): {"name": JOINT_NAMES[sid], "home": home[sid],
                   "min_off": min_off[sid], "max_off": max_off[sid]}
        for sid in ids
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(calibration, indent=2))
    print(f"\nsaved -> {OUT}")
    print("torque left OFF. Power-cycle or re-enable torque before commanding motion.")
    bus.close()


if __name__ == "__main__":
    main()
