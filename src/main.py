import os

import mujoco
import mujoco.viewer

SCENE_XML = os.path.join(os.path.dirname(__file__), "..", "models", "universal_robots_ur5e", "scene.xml")

def main():
  model = mujoco.MjModel.from_xml_path(SCENE_XML)
  data = mujoco.MjData(model)

  mujoco.viewer.launch(model, data)

if __name__ == "__main__":
    main()