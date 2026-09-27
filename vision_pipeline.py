"""
vision_pipeline.py
Black-beam detector + tracker + pinhole 3D projection.

Run standalone against a webcam, video file, or MJPEG stream URL —
no ESP32 or backend needed to test this part:

    python vision_pipeline.py --source 0                          # webcam
    python vision_pipeline.py --source http://<esp32-ip>/stream   # live rig
    python vision_pipeline.py --source clip.mp4 --tune             # tune thresholds

--tune opens HSV trackbars so you can dial in the black-beam threshold
live against YOUR lighting before trusting any numbers this spits out.
"""

import argparse
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from mjpeg_client import mjpeg_frames


# ---------------------------------------------------------------------------
# Config — tune these against your actual lighting, not the defaults.
# ---------------------------------------------------------------------------

@dataclass
class DetectorConfig:
    # Color detection now runs on HUE, not brightness — blue is an
    # actual color, so this is far more robust than the black/white
    # "low brightness" approach. OpenCV hue range is 0-179; pure blue
    # sits roughly 100-130, adjust with --tune against your object.
    h_min: int = 100
    h_max: int = 130
    s_min: int = 80    # reject washed-out/gray pixels
    v_min: int = 50    # reject near-black pixels
    min_area: int = 400          # px^2 — reject small noise blobs
    min_aspect_ratio: float = 1.0  # 1.0 = any shape; raise if your object is elongated
    morph_kernel: int = 5
    blur_kernel: int = 5


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def load(cls, path: str = "calibration.npz") -> "CameraIntrinsics":
        data = np.load(path)
        mtx = data["camera_matrix"]
        return cls(fx=mtx[0, 0], fy=mtx[1, 1], cx=mtx[0, 2], cy=mtx[1, 2])

    @classmethod
    def guess_for_resolution(cls, width: int, height: int) -> "CameraIntrinsics":
        """
        Fallback ONLY for getting the pipeline running end to end before
        you've calibrated. OV2640 is roughly 66-70deg horizontal FOV.
        Do not trust X/Y in centimeters until you run camera_calibrate.py.
        """
        fx = fy = width / (2 * np.tan(np.deg2rad(68 / 2)))
        return cls(fx=fx, fy=fy, cx=width / 2, cy=height / 2)


@dataclass
class Detection:
    pixel_x: float
    pixel_y: float
    angle_deg: float
    width_px: float
    height_px: float
    box: np.ndarray  # 4x2 box corner points, for drawing


@dataclass
class Position3D:
    x_cm: float
    y_cm: float
    z_cm: float


# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------

class BlackBeamDetector:
    """Name kept from the original black-beam version — this now does
    generic hue-based color detection (see DetectorConfig), works for
    any object color, not just black."""

    def __init__(self, config: DetectorConfig):
        self.config = config

    def _mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        cfg = self.config
        blurred = cv2.GaussianBlur(frame_bgr, (cfg.blur_kernel, cfg.blur_kernel), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        lower = np.array([cfg.h_min, cfg.s_min, cfg.v_min])
        upper = np.array([cfg.h_max, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((cfg.morph_kernel, cfg.morph_kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def debug_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        return self._mask(frame_bgr)

    def find(self, frame_bgr: np.ndarray) -> Optional[Detection]:
        cfg = self.config
        mask = self._mask(frame_bgr)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best = None
        best_score = -1.0
        for c in contours:
            area = cv2.contourArea(c)
            if area < cfg.min_area:
                continue
            rect = cv2.minAreaRect(c)  # ((cx,cy),(w,h),angle)
            (cx, cy), (w, h), angle = rect
            if w == 0 or h == 0:
                continue
            long_side, short_side = max(w, h), min(w, h)
            aspect = long_side / short_side
            if aspect < cfg.min_aspect_ratio:
                continue
            score = area * aspect  # favor big AND clearly beam-shaped
            if score > best_score:
                best_score = score
                box = cv2.boxPoints(rect)
                norm_angle = angle if w >= h else angle + 90
                best = Detection(
                    pixel_x=cx, pixel_y=cy, angle_deg=norm_angle,
                    width_px=long_side, height_px=short_side, box=box,
                )
        return best


# ---------------------------------------------------------------------------
# Tracker — cheap frame-to-frame follow, re-locked periodically by the
# full detector so it never quietly drifts onto the wrong dark blob.
# ---------------------------------------------------------------------------

class BeamTracker:
    def __init__(self, detector: BlackBeamDetector, relock_every: int = 15):
        self.detector = detector
        self.relock_every = relock_every
        self.tracker = None
        self.frames_since_relock = 0

    def _new_cv_tracker(self):
        # OpenCV's tracking API moved around across versions/packages —
        # try the current namespace first, then the legacy one. Requires
        # opencv-contrib-python (plain opencv-python doesn't include it).
        if hasattr(cv2, "TrackerCSRT_create"):
            return cv2.TrackerCSRT_create()
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
            return cv2.legacy.TrackerCSRT_create()
        raise RuntimeError(
            "cv2 has no TrackerCSRT_create. Run:\n"
            "  pip uninstall opencv-python\n"
            "  pip install opencv-contrib-python"
        )

    def update(self, frame_bgr: np.ndarray) -> Optional[Detection]:
        need_relock = self.tracker is None or self.frames_since_relock >= self.relock_every

        if need_relock:
            det = self.detector.find(frame_bgr)
            if det is None:
                self.tracker = None
                return None
            xs, ys = det.box[:, 0], det.box[:, 1]
            # cv2 tracker init wants an int-typed bounding box, not float
            bbox = (int(xs.min()), int(ys.min()),
                    int(xs.max() - xs.min()), int(ys.max() - ys.min()))
            self.tracker = self._new_cv_tracker()
            self.tracker.init(frame_bgr, bbox)
            self.frames_since_relock = 0
            return det

        ok, bbox = self.tracker.update(frame_bgr)
        self.frames_since_relock += 1
        if not ok:
            self.tracker = None
            return self.update(frame_bgr)  # force a relock right now

        bx, by, bw, bh = bbox
        cx, cy = bx + bw / 2, by + bh / 2
        # Tracker gives an axis-aligned box (no rotation) between relocks —
        # fine for position-following; angle refreshes on the next relock.
        box = np.array([[bx, by], [bx + bw, by], [bx + bw, by + bh], [bx, by + bh]])
        return Detection(pixel_x=cx, pixel_y=cy, angle_deg=0.0, width_px=bw, height_px=bh, box=box)


# ---------------------------------------------------------------------------
# Pinhole back-projection: pixel (u,v) + depth Z -> camera-frame (X,Y,Z)
# ---------------------------------------------------------------------------

def project_to_3d(u: float, v: float, z_cm: float, intr: CameraIntrinsics) -> Position3D:
    x = (u - intr.cx) * z_cm / intr.fx
    y = (v - intr.cy) * z_cm / intr.fy
    return Position3D(x_cm=x, y_cm=y, z_cm=z_cm)


def estimate_depth_from_size(det: Detection, intr: CameraIntrinsics, real_size_cm: float) -> float:
    """
    Approximates distance from a single camera using the object's known
    real-world size vs. how large it appears in pixels (pinhole model):

        z_cm = (real_size_cm * focal_length_px) / apparent_size_px

    This is NOT as accurate as a real depth sensor (VL53L0X) — it assumes
    you've measured real_size_cm correctly, that the object is roughly
    perpendicular to the camera, and it degrades if the object rotates
    (apparent size changes with viewing angle, not just distance). Good
    enough for rough "how far," not for precision grasping. Swap back to
    a real sensor reading later without any other code changes — see
    project_to_3d's depth_cm parameter, which this is just a fallback for.
    """
    apparent_px = max(det.width_px, 1e-6)  # guard divide-by-zero
    return (real_size_cm * intr.fx) / apparent_px


# ---------------------------------------------------------------------------
# Standalone runner
# ---------------------------------------------------------------------------

def _nothing(_):
    pass


def frame_source(source: str):
    """Yields BGR frames from a webcam index, video file, or an MJPEG
    stream URL (http/https) — the ESP32 case goes through our own
    multipart parser instead of cv2.VideoCapture, which is unreliable
    against ESP32-style MJPEG streams."""
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


def run_standalone(source: str, tune: bool, object_size_cm: float):
    config = DetectorConfig()
    detector = BlackBeamDetector(config)
    tracker = BeamTracker(detector)
    intr = None

    if tune:
        cv2.namedWindow("tune")
        cv2.createTrackbar("H min", "tune", config.h_min, 179, _nothing)
        cv2.createTrackbar("H max", "tune", config.h_max, 179, _nothing)
        cv2.createTrackbar("S min", "tune", config.s_min, 255, _nothing)
        cv2.createTrackbar("V min", "tune", config.v_min, 255, _nothing)

    for frame in frame_source(source):
        if intr is None:
            h, w = frame.shape[:2]
            intr = CameraIntrinsics.guess_for_resolution(w, h)
            print(f"[warn] FOV-guess intrinsics fx={intr.fx:.1f} — "
                  f"run camera_calibrate.py before trusting X/Y in cm")

        if tune:
            config.h_min = cv2.getTrackbarPos("H min", "tune")
            config.h_max = cv2.getTrackbarPos("H max", "tune")
            config.s_min = cv2.getTrackbarPos("S min", "tune")
            config.v_min = cv2.getTrackbarPos("V min", "tune")
            cv2.imshow("mask", detector.debug_mask(frame))

        det = tracker.update(frame)
        display = frame.copy()

        if det is not None:
            box = det.box.astype(int)
            cv2.drawContours(display, [box], 0, (0, 255, 0), 2)
            cv2.circle(display, (int(det.pixel_x), int(det.pixel_y)), 5, (0, 0, 255), -1)
            pos = project_to_3d(det.pixel_x, det.pixel_y,
                                 estimate_depth_from_size(det, intr, object_size_cm), intr)
            label = f"X={pos.x_cm:.1f}cm Y={pos.y_cm:.1f}cm Z={pos.z_cm:.1f}cm angle={det.angle_deg:.0f}"
            cv2.putText(display, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(display, "no beam detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow("tracking", display)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="0", help="webcam index, video file path, or MJPEG stream URL")
    parser.add_argument("--tune", action="store_true", help="open HSV trackbars for live threshold tuning")
    parser.add_argument("--object-size-cm", type=float, default=5.0,
                         help="cm — the real-world size (longest side) of your tracked object, "
                              "used to estimate distance from apparent pixel size. Measure your "
                              "actual object and pass this accurately, or Z will be wrong.")
    args = parser.parse_args()
    run_standalone(args.source, args.tune, args.object_size_cm)
