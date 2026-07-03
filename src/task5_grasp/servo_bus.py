from __future__ import annotations

import time

import scservo_sdk as scs

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42  #2 bytes
ADDR_GOAL_SPEED = 46  #2 bytes
ADDR_PRESENT_POSITION = 56  #2 bytes

PORT = "/dev/cu.usbmodem5B3D0407881"
BAUD = 1_000_000
PROTOCOL_END = 0 

JOINT_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}


class ServoBus:
    def __init__(self, port=PORT, baud=BAUD):
        self.port = scs.PortHandler(port)
        self.packet = scs.PacketHandler(PROTOCOL_END)
        if not self.port.openPort():
            raise SystemExit(f"could not open port {port}")
        if not self.port.setBaudRate(baud):
            raise SystemExit(f"could not set baud {baud} on {port}")

    def ping(self, sid):
        model, comm, err = self.packet.ping(self.port, sid)
        if (comm != scs.COMM_SUCCESS) or (err != 0):
            return None
        return model

    def read_position(self, sid, retries=5):
        for _ in range(retries):
            try:
                pos, comm, err = self.packet.read2ByteTxRx(self.port, sid, ADDR_PRESENT_POSITION)
            except (IndexError, TypeError):
                time.sleep(0.05)
                continue
            if comm == scs.COMM_SUCCESS and err == 0:
                return pos
            time.sleep(0.05)
        return None

    def set_torque(self, sid, on):
        self.packet.write1ByteTxRx(self.port, sid, ADDR_TORQUE_ENABLE, 1 if on else 0)

    def set_torque_all(self, on, ids=range(1, 7)):
        for sid in ids:
            self.set_torque(sid, on)

    def set_speed(self, sid, ticks_per_step):
        self.packet.write2ByteTxRx(self.port, sid, ADDR_GOAL_SPEED, ticks_per_step)

    def write_goal(self, sid, ticks):
        self.packet.write2ByteTxRx(self.port, sid, ADDR_GOAL_POSITION, int(ticks))

    def close(self):
        self.port.closePort()


def scan(port=PORT, baud=BAUD, ids=range(1, 7)):
    bus = ServoBus(port, baud)
    print(f"scanning {port} @ {baud} baud")
    found = []
    for sid in ids:
        model = bus.ping(sid)
        if model is None:
            print(f"  id {sid} ({JOINT_NAMES.get(sid, '?'):12}): no response")
            continue
        pos = bus.read_position(sid)
        print(f"  id {sid} ({JOINT_NAMES.get(sid, '?'):12}): model={model} pos={pos}")
        found.append(sid)
    bus.close()
    print(f"responded: {sorted(found)}")
    return found


if __name__ == "__main__":
    scan()
