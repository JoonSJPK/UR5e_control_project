from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CAM_INDEX = 0
LOWER = (8, 200, 190)
UPPER = (28, 255, 255)
MIN_AREA = 3000


def detect(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER, UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) >= MIN_AREA]
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    M = cv2.moments(c)
    cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    (_, _), (w, h), angle = cv2.minAreaRect(c)
    return cx, cy, cv2.boundingRect(c), float(angle)


def draw(frame, det):
    x, y, w, h = det[2]
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
    cv2.circle(frame, (det[0], det[1]), 6, (0, 0, 255), -1)
    return frame


def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera {CAM_INDEX}")
    print("live preview - 'q' to quit")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        det = detect(frame)
        if det:
            draw(frame, det)
            cv2.putText(frame, f"({det[0]},{det[1]}) angle={det[3]:.0f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow("orange detect", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
