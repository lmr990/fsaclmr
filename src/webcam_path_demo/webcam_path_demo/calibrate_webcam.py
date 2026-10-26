"""Chessboard calibration for the webcam: captures frames from the camera
until enough good chessboard views are found, then saves K + distortion to a
yaml usable as webcam_path_demo's intrinsics_file.

Usage: ros2 run webcam_path_demo webcam_calibrate --output my_cam.yaml
"""

import argparse
import os

import cv2
import numpy as np
import yaml

PATTERN = (9, 6)          # inner corners
SQUARE_M = 0.025          # 25 mm squares


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--views", type=int, default=15)
    ap.add_argument("--output", default=os.path.expanduser("~/webcam_intrinsics.yaml"))
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open camera {args.device}")
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    objp = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE_M
    obj_points, img_points = [], []
    size = None

    print(f"Show a {PATTERN[0]}x{PATTERN[1]} chessboard ({SQUARE_M*1000:.0f}mm squares).")
    print(f"Need {args.views} good views. Press 's' to force capture, 'q' to finish early.")
    while len(obj_points) < args.views:
        ok, frame = cap.read()
        if not ok:
            continue
        size = (frame.shape[1], frame.shape[0])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, PATTERN, None)
        disp = frame.copy()
        if found:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            obj_points.append(objp)
            img_points.append(corners)
            cv2.drawChessboardCorners(disp, PATTERN, corners, found)
            cv2.putText(disp, f"{len(obj_points)}/{args.views}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        else:
            cv2.putText(disp, f"{len(obj_points)}/{args.views} - no board", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv2.imshow("calibrate (q=finish, any key=manual shot)", disp)
        key = cv2.waitKey(200 if not found else 400) & 0xFF
        if key == ord("q"):
            break
        if not found and key != 255:
            continue

    cap.release()
    cv2.destroyAllWindows()
    if len(obj_points) < 5:
        raise SystemExit(f"Only {len(obj_points)} views - not enough, rerun and vary angles.")

    rms, K, dist, _, _ = cv2.calibrateCamera(
        obj_points, img_points, size, None, None)
    out = {"K": K.tolist(), "dist": dist.tolist(), "rms": float(rms),
           "image_size": list(size), "pattern": list(PATTERN), "square_m": SQUARE_M}
    with open(args.output, "w") as f:
        yaml.safe_dump(out, f)
    print(f"Saved {args.output} | rms={rms:.3f}px")
    print(f"fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}")


if __name__ == "__main__":
    main()
