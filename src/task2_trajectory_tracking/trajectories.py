import os

import mujoco
import mujoco.viewer
from tuned_controllers import PIDController

SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "..", "models", "universal_robots_ur5e", "scene.xml")

def main():

  model = mujoco.MjModel.from_xml_path(SCENE_XML)
  data = mujoco.MjData(model)
  dt = model.opt.timestep

  fsm_state = 0
  fsm_pose1 = 0
  fsm_pose2 = 1

  targets = []
  targets.append([-1.5708, -1.5708, -1.5708, -1.5708, -1.5708, -1.5708,])
  targets.append([1.5708, -1.5708, 1.5708, 1.5708, 1.5708, 1.5708,])

  controllers = []
  for idx in range(6):
     controllers.append(PIDController(joint = idx))

  prev_time = 0.0

  trans_time = dt * 4000
  #trans_state = [0, 1, 2]

  with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            
            # Detect viewer reset — data.time jumps back to 0
            if data.time < prev_time or data.time == trans_time:
                for controller in controllers:
                    controller.reset()
            prev_time = data.time
            
            #apply torque to each joint
            if( data.time <= trans_time ):
              for idx, controller in enumerate(controllers):
                  data.qfrc_applied[idx] = controller.compute(
                      dt, targets[0][idx], data.qpos[idx], data.qvel[idx]
                  )

            #apply torque to each joint
            if( data.time > trans_time ):
              for idx, controller in enumerate(controllers):
                  data.qfrc_applied[idx] = controller.compute(
                      dt, targets[1][idx], data.qpos[idx], data.qvel[idx]
                  )

            


            #update simulation
            mujoco.mj_step(model, data)
            viewer.sync()




if __name__ == "__main__":
    main()
