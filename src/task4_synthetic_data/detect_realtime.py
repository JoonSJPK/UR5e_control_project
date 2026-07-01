import argparse
from pathlib import Path

import cv2
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[2]
WEIGHTS = ROOT / "docs" / "task4_data" / "runs" / "cube_detector-3" / "weights" / "best.pt"
CONF = 0.25


def draw(frame, result):
    best = None
    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        conf = float(box.conf[0])
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        cv2.putText(frame, f"cube {conf:.2f}", (int(x1), int(y1) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if best is None or conf > best[2]:
            best = (cx, cy, conf)
    return best


def run_image(model, path):
    frame = cv2.imread(path)
    best = draw(frame, model(frame, conf=CONF, verbose=False)[0])
    print("cube centre:", best)
    cv2.imshow("cube detector", frame)
    cv2.waitKey(0)


def run_camera(model, cam):
    cap = cv2.VideoCapture(cam)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera {cam} (try --cam 1/2, or check it's free)")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        best = draw(frame, model(frame, conf=CONF, verbose=False)[0])
        if best:
            cv2.putText(frame, f"centre=({best[0]},{best[1]})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow("cube detector", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, default=0, help="camera index")
    ap.add_argument("--image", type=str, default=None, help="run on one still instead")
    args = ap.parse_args()
    
    model = YOLO(str(WEIGHTS))

    if args.image:
        run_image(model, args.image)
    else:
        run_camera(model, args.cam)


if __name__ == "__main__":
    main()
