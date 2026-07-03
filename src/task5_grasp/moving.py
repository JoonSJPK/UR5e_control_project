#from .servo_bus import ServoBus, JOINT_NAMES, PORT, BAUD
from pathlib import Path
import time
import mujoco

ROOT = Path(__file__).resolve().parents[2]
SCENE = ROOT / "models" / "trs_so_arm100" / "scene_so101.xml"
SERVO_JOINT = {1: "Rotation", 2: "Pitch", 3: "Elbow",
               4: "Wrist_Pitch", 5: "Wrist_Roll", 6: "Jaw"}

JOINT_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}

def main():
    #bus = ServoBus(PORT, BAUD)
    ids = list(JOINT_NAMES)
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, model.key("rest").id)

    for sid in ids:
        name = SERVO_JOINT[sid]    
        act = model.actuator(name).id
        qadr = model.joint(name).qposadr[0]
        data.ctrl[act] = data.qpos[qadr]

    rest_q = model.key("rest").qpos.copy()
    home_q = model.key("home").qpos.copy()

    START_T = 4.0     
    DURATION = 3.0     

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            t = (data.time - START_T) / DURATION
            if 0.0 <= t <= 1.0:
                s = t * t * (3 - 2 * t) 
                for sid in ids:
                    name = SERVO_JOINT[sid]
                    act = model.actuator(name).id
                    qadr = model.joint(name).qposadr[0]
                    data.ctrl[act] = (1 - s) * rest_q[qadr] + s * home_q[qadr]

            mujoco.mj_step(model, data)
            viewer.sync()

            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)
        

if __name__ == "__main__":
    main()