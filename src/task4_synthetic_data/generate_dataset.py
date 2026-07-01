import json
from pathlib import Path
import random
import math

import mujoco
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
SCENE = ROOT / "models" / "trs_so_arm100" / "scene_so101.xml"
OUT = ROOT / "docs" / "task4_data"
W, H = 1280, 720
N_FRAMES = 200
MIN_CUBE_PIXELS = 400
SEED = 0


def main():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=H, width=W)

    key_id = model.key("look").id
    cube_geom_id = model.geom("cube_geom").id
    cube_qadr = model.jnt_qposadr[model.joint("cube_free").id]
    cube_mat_id = model.material("cube_mat").id
    arm_joints = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"]
    arm_qadr = [model.jnt_qposadr[model.joint(n).id] for n in arm_joints]

    (OUT / "images").mkdir(parents=True, exist_ok=True)
    (OUT / "masks").mkdir(parents=True, exist_ok=True)
    annotations = []

    saved = 0
    for i in range(N_FRAMES):
        mujoco.mj_resetDataKeyframe(model, data, key_id)

        randomize(model, data, cube_qadr, arm_qadr, cube_mat_id)

        mujoco.mj_forward(model, data)

        renderer.disable_segmentation_rendering()
        renderer.update_scene(data, camera="wrist_cam")
        rgb = renderer.render()

        renderer.enable_segmentation_rendering()
        renderer.update_scene(data, camera="wrist_cam")
        seg = renderer.render()[:, :, 0] 
        mask = seg == cube_geom_id
        
        # drop this frame if cube not visible
        cube_pixels = int(mask.sum())
        if cube_pixels < MIN_CUBE_PIXELS:
            continue

        # bbox from the mask -> (x, y, w, h) in pixels
        ys, xs = np.where(mask)
        x = int(xs.min())
        y = int(ys.min())
        w = int(xs.max() - xs.min() + 1)
        h = int(ys.max() - ys.min() + 1)

        # save
        image_name = f"{saved:04d}.png"
        Image.fromarray(rgb).save(OUT / "images" / image_name)
        Image.fromarray(mask.astype(np.uint8) * 255).save(OUT / "masks" / image_name)
        annotations.append({"image": image_name,
            "bbox": [x, y, w, h],
            "cube_pixels": cube_pixels,
        })
        saved += 1

    (OUT / "annotations.json").write_text(json.dumps(annotations, indent=2))
    print(f"saved {saved} frames to {OUT}")


def quat_from_yaw(theta):
    return np.array([np.cos(theta / 2), 0.0, 0.0, np.sin(theta / 2)])


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def randomize(model, data, cube_qadr, arm_qadr, cube_mat_id):
    # random cube position
    cube_x = random.uniform(-0.20 - 0.05, -0.20 + 0.05)
    cube_y = random.uniform(-0.132 - 0.05, -0.132 + 0.05)
    cube_z = 0.025
    data.qpos[cube_qadr + 0] = cube_x
    data.qpos[cube_qadr + 1] = cube_y
    data.qpos[cube_qadr + 2] = cube_z

    theta = random.uniform(-math.pi, math.pi)
    quat = quat_from_yaw(theta)
    data.qpos[cube_qadr + 3] = quat[0]
    data.qpos[cube_qadr + 4] = quat[1]
    data.qpos[cube_qadr + 5] = quat[2]
    data.qpos[cube_qadr + 6] = quat[3]

    # jitter arm
    for adr in arm_qadr:
        jitter = random.uniform(-0.05, 0.05)
        data.qpos[adr] = data.qpos[adr] + jitter

    # jitter lighting
    light_x = random.uniform(-0.5, 0.5)
    light_y = random.uniform(-0.5, 0.5)
    light_z = 1.5
    model.light_pos[0][0] = light_x
    model.light_pos[0][1] = light_y
    model.light_pos[0][2] = light_z

    model.light_diffuse[0][0] = random.uniform(0.6, 1.0) 
    model.light_diffuse[0][1] = random.uniform(0.6, 1.0) 
    model.light_diffuse[0][2] = random.uniform(0.6, 1.0) 

    model.vis.headlight.diffuse[0] = random.uniform(0.4, 0.8)
    model.vis.headlight.diffuse[1] = random.uniform(0.4, 0.8)
    model.vis.headlight.diffuse[2] = random.uniform(0.4, 0.8)

    # random cube color (keep orange however)
    base_r, base_g, base_b = 1.0, 0.45, 0.05
    r = clamp(base_r + random.uniform(-0.1, 0.1), 0.0, 1.0)
    g = clamp(base_g + random.uniform(-0.1, 0.1), 0.0, 1.0)
    b = clamp(base_b + random.uniform(-0.1, 0.1), 0.0, 1.0)
    model.mat_rgba[cube_mat_id][0] = r
    model.mat_rgba[cube_mat_id][1] = g
    model.mat_rgba[cube_mat_id][2] = b
    model.mat_rgba[cube_mat_id][3] = 1.0


def preview(rows=4, cols=5, out=None):
    out = out or (OUT / "_preview.png")
    random.seed(SEED)

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    tw, th = 320, 180
    renderer = mujoco.Renderer(model, height=th, width=tw)

    key_id = model.key("look").id
    cube_geom_id = model.geom("cube_geom").id
    cube_qadr = model.jnt_qposadr[model.joint("cube_free").id]
    cube_mat_id = model.material("cube_mat").id
    arm_joints = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"]
    arm_qadr = [model.jnt_qposadr[model.joint(n).id] for n in arm_joints]

    sheet = Image.new("RGB", (cols * tw, rows * th), (30, 30, 30))
    for i in range(rows * cols):
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        randomize(model, data, cube_qadr, arm_qadr, cube_mat_id)
        mujoco.mj_forward(model, data)

        renderer.disable_segmentation_rendering()
        renderer.update_scene(data, camera="wrist_cam")
        rgb = renderer.render()

        renderer.enable_segmentation_rendering()
        renderer.update_scene(data, camera="wrist_cam")
        seg = renderer.render()[:, :, 0]
        mask = seg == cube_geom_id
        n = int(mask.sum())

        tile = Image.fromarray(rgb)
        draw = ImageDraw.Draw(tile)
        ys, xs = np.where(mask)
        ok = n >= MIN_CUBE_PIXELS
        colour = (0, 255, 0) if ok else (255, 60, 60)
        if xs.size:
            draw.rectangle((xs.min(), ys.min(), xs.max(), ys.max()),
                           outline=colour, width=2)
        draw.text((4, 4), f"{n}px {'OK' if ok else 'SKIP'}", fill=colour)

        r, c = divmod(i, cols)
        sheet.paste(tile, (c * tw, r * th))

    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
