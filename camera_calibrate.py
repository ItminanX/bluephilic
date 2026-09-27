"""
camera_calibrate.py
One-time OV2640 intrinsic calibration using a printed checkerboard.

Print a standard 9x6-internal-corner checkerboard, tape it to something
flat, and capture 15-20 photos of it at different angles/distances/
positions in the frame (corners matter more than center shots — that's
where lens distortion actually shows up).

Usage:
    python camera_calibrate.py --source 0 --capture
        (press 'c' to save a frame, 'q' when you have ~15-20)

    python camera_calibrate.py --calibrate
        (processes calib_images/ -> calibration.npz)

vision_pipeline.py and server.py both auto-load calibration.npz if it
exists in the working directory.
"""
import argparse
import glob
import os

import cv2
import numpy as np

from mjpeg_client import mjpeg_frames

CHECKERBOARD = (8, 5)  # internal corners, not squares — confirmed via debug_checkerboard.py (15/15 hit)
CAPTURE_DIR = "calib_images"


def _frame_source(source: str):
    """Same MJPEG-safe source logic as vision_pipeline.py — using our
    own multipart parser for http(s) URLs instead of cv2.VideoCapture,
    which is unreliable against ESP32-style streams."""
    if source.startswith("http://") or source.startswith("https://"):
        yield from mjpeg_frames(source)
        return
    cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
    if not cap.isOpened():
        raise SystemExit(f"Could not open source: {source}")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            yield frame
    finally:
        cap.release()


def capture(source: str):
    os.makedirs(CAPTURE_DIR, exist_ok=True)
    count = len(glob.glob(f"{CAPTURE_DIR}/*.jpg"))
    print("Press 'c' to capture a frame, 'q' to quit.")
    for frame in _frame_source(source):
        preview = frame.copy()
        found, corners = cv2.findChessboardCorners(frame, CHECKERBOARD, None)
        if found:
            cv2.drawChessboardCorners(preview, CHECKERBOARD, corners, found)
        cv2.putText(preview, f"saved: {count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("capture", preview)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            path = f"{CAPTURE_DIR}/img_{count:03d}.jpg"
            cv2.imwrite(path, frame)
            print(f"saved {path}")
            count += 1
        elif key == ord('q'):
            break
    cv2.destroyAllWindows()


def calibrate():
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    objpoints, imgpoints = [], []
    images = glob.glob(f"{CAPTURE_DIR}/*.jpg")
    if len(images) < 10:
        raise SystemExit(f"Only found {len(images)} images in {CAPTURE_DIR}/ — capture at least 10-15 first.")

    img_shape = None
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        if found:
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            objpoints.append(objp)
            imgpoints.append(corners2)

    if len(objpoints) < 8:
        raise SystemExit(f"Checkerboard only found in {len(objpoints)}/{len(images)} images — "
                          f"retake with better lighting/flatness.")

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None
    )
    print(f"RMS reprojection error: {ret:.4f} (lower is better — under ~1.0 is good)")
    print("Camera matrix:\n", camera_matrix)
    np.savez("calibration.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
    print("Saved calibration.npz")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="0")
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--calibrate", action="store_true")
    args = parser.parse_args()
    if args.capture:
        capture(args.source)
    if args.calibrate:
        calibrate()