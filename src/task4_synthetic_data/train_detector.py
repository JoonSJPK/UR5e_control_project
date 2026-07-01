import json
import random
import shutil
from pathlib import Path

from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "docs" / "task4_data"
YOLO_DIR = DATA / "yolo"
W, H = 1280, 720
VAL_FRACTION = 0.15
SEED = 0
CLASS_NAMES = {0: "cube"}


def build_yolo_dataset():
    annotations = json.loads((DATA / "annotations.json").read_text())

    random.seed(SEED)
    random.shuffle(annotations)
    n_val = int(len(annotations) * VAL_FRACTION)
    split_of = {a["image"]: ("val" if i < n_val else "train")
                for i, a in enumerate(annotations)}

    if YOLO_DIR.exists():
        shutil.rmtree(YOLO_DIR)
    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        (YOLO_DIR / sub).mkdir(parents=True, exist_ok=True)

    for a in annotations:
        name = a["image"]
        split = split_of[name]
        x, y, w, h = a["bbox"]

        cx = (x + w / 2) / W
        cy = (y + h / 2) / H
        nw = w / W
        nh = h / H

        shutil.copy(DATA / "images" / name, YOLO_DIR / "images" / split / name)
        label_path = YOLO_DIR / "labels" / split / (Path(name).stem + ".txt")
        label_path.write_text(f"0 {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")

    names_block = "\n".join(f"  {i}: {n}" for i, n in CLASS_NAMES.items())
    (YOLO_DIR / "data.yaml").write_text(
        f"path: {YOLO_DIR}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"names:\n{names_block}\n"
    )
    n_train = len(annotations) - n_val
    print(f"built YOLO dataset: {n_train} train / {n_val} val -> {YOLO_DIR}")


def train(epochs=50, imgsz=640, model="yolo11n.pt"):
    YOLO(model).train(
        data=str(YOLO_DIR / "data.yaml"),
        epochs=epochs,
        imgsz=imgsz,
        seed=SEED,
        project=str(DATA / "runs"),
        name="cube_detector",
    )


if __name__ == "__main__":
    build_yolo_dataset()
    train()
