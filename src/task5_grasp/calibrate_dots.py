from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "docs" / "task5_data" / "hand_eye.json"
CAM_INDEX = 0 


def refine_to_blob(gray, u, v, win=20):
    h, w = gray.shape
    x0, x1 = max(0, u - win), min(w, u + win)
    y0, y1 = max(0, v - win), min(h, v + win)
    patch = gray[y0:y1, x0:x1].astype(np.float32)
    thr = max(patch.max() * 0.6, patch.mean() + 2 * patch.std())
    mask = patch >= thr
    if mask.sum() < 3:
        return float(u), float(v)
    ys, xs = np.nonzero(mask)
    wts = patch[ys, xs]
    return float(x0 + (xs * wts).sum() / wts.sum()), float(y0 + (ys * wts).sum() / wts.sum())


def capture_frame():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera {CAM_INDEX}")
    print("SPACE = freeze frame, ESC = cancel")
    frame = None
    while True:
        ok, f = cap.read()
        if not ok:
            continue
        frame = f
        cv2.imshow("calibrate: SPACE to freeze", f)
        k = cv2.waitKey(1) & 0xFF
        if k == 32: # space
            break
        if k == 27:   # esc
            cap.release(); cv2.destroyAllWindows()
            raise SystemExit("cancelled")
    cap.release()
    cv2.destroyAllWindows()
    return frame


def on_mouse(event, x, y, flags, param):
    gray, picks, n = param
    if event == cv2.EVENT_LBUTTONDOWN and len(picks) < n:
        picks.append(refine_to_blob(gray, x, y))


def click_dots(frame, n):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    picks = []

    win = "click each dot in order"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse, (gray, picks, n))
    while True:
        disp = frame.copy()
        for i, (u, v) in enumerate(picks):
            cv2.circle(disp, (int(u), int(v)), 6, (0, 255, 0), 2)
            cv2.putText(disp, str(i + 1), (int(u) + 8, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(disp, f"{len(picks)}/{n} clicked", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow(win, disp)
        k = cv2.waitKey(20) & 0xFF
        if len(picks) == n and k in (13, 32):   #enter or space to confirm
            break
        if k == 27:
            break
    cv2.destroyAllWindows()
    return picks


def main():
    world = []
    print("Enter each dot's X Y in mm (X forward, Y left). Type 'done' after >=4.\n")
    while True:
        s = input(f"dot {len(world)+1} > X Y (mm), or 'done': ").strip()
        if s.lower() == "done":
            break
        try:
            X, Y = (float(v) for v in s.split())
        except ValueError:
            print("  need two numbers, e.g. '150 -40'")
            continue
        world.append([X, Y])
    if len(world) < 4:
        raise SystemExit(f"need >=4 dots, got {len(world)}")

    frame = capture_frame()
    picks = click_dots(frame, len(world))
    if len(picks) != len(world):
        raise SystemExit(f"clicked {len(picks)} but entered {len(world)} coords")

    src = np.array(picks, dtype=np.float64)
    dst = np.array(world, dtype=np.float64)
    H, _ = cv2.findHomography(src, dst, method=0)

    errs = []
    for (u, v), (X, Y) in zip(picks, world):
        p = H @ np.array([u, v, 1.0])
        errs.append(((p[0] / p[2] - X) ** 2 + (p[1] / p[2] - Y) ** 2) ** 0.5)
    print(f"\nfit on {len(world)} dots: mean {np.mean(errs):.1f} mm, max {np.max(errs):.1f} mm")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps({
        "H": H.tolist(),
        "camera_index": CAM_INDEX,
        "units": "mm",
        "frame": "arm base rotation axis; X forward, Y left; table plane",
        "method": "white dots on table plane",
        "points": {"pixels": [list(p) for p in picks], "world": world},
        "mean_err_mm": float(np.mean(errs)),
    }, indent=2))
    print(f"saved -> {OUT}")
    if np.mean(errs) > 10:
        print("WARNING: high error - check dot order matches entry order, "
              "re-click, or verify measurements.")


if __name__ == "__main__":
    main()
