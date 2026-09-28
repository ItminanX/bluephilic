"""
server.py
FastAPI backend meant to run on Render. Receives JPEG frames (and a
VL53L0X depth reading) pushed from the ESP32, runs the black-beam
detector/tracker, and exposes the latest 3D position as JSON.

Run locally:
    uvicorn server:app --reload --port 8000

Test without any hardware at all:
    curl -X POST http://localhost:8000/ingest \
         -F "depth_cm=30.0" \
         -F "frame=@test.jpg"

Watch it live in a browser while tuning:
    http://localhost:8000/debug_feed
"""
import time
import threading
from typing import Optional

import cv2
import numpy as np
from fastapi import FastAPI, UploadFile, Form
from fastapi.responses import JSONResponse, StreamingResponse

from vision_pipeline import (
    BlackBeamDetector, DetectorConfig,
    CameraIntrinsics, project_to_3d, estimate_depth_from_size, Detection,
)

app = FastAPI()

# Measure your actual object's longest side in cm and set this accurately —
# it's what the camera-only depth estimate is calibrated against. If you
# later add a real depth sensor (VL53L0X via a relay MCU, etc.), send a
# real depth_cm > 0 in the request and it will be trusted over this
# estimate automatically — no other code changes needed.
OBJECT_SIZE_CM = 5.0

_config = DetectorConfig()
_detector = BlackBeamDetector(_config)
# NOTE: deliberately NOT using BeamTracker (CSRT) here — measured at
# ~97ms/frame vs. ~0.6ms/frame for plain detection. CSRT is built for
# tracking complex, textured objects; for a simple color-blob detector
# like this one, it's pure overhead with no accuracy benefit. Plain
# per-frame detection is both faster and simpler.
_intrinsics: Optional[CameraIntrinsics] = None

_lock = threading.Lock()
_state = {
    "last_frame": None,      # annotated JPEG bytes, for /debug_feed
    "last_detection": None,  # dict
    "last_update_ts": 0.0,
}

try:
    _intrinsics = CameraIntrinsics.load("calibration.npz")
    print("[server] loaded calibration.npz")
except Exception:
    print("[server] no calibration.npz yet — using FOV guess until you calibrate")


@app.post("/ingest")
async def ingest(frame: UploadFile, depth_cm: float = Form(-1.0)):
    global _intrinsics

    raw = await frame.read()
    img_array = np.frombuffer(raw, dtype=np.uint8)
    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    if img is None:
        return JSONResponse({"error": "could not decode frame"}, status_code=400)

    if _intrinsics is None:
        h, w = img.shape[:2]
        _intrinsics = CameraIntrinsics.guess_for_resolution(w, h)

    det: Optional[Detection] = _detector.find(img)
    result = {"timestamp": time.time(), "found": det is not None}

    display = img.copy()
    if det is not None:
        # A real sensor reading (> 0) is trusted; otherwise fall back to
        # the camera-only known-size estimate. This is what lets you plug
        # a real depth sensor back in later with zero code changes here.
        used_real_sensor = depth_cm > 0
        z_cm = depth_cm if used_real_sensor else estimate_depth_from_size(det, _intrinsics, OBJECT_SIZE_CM)

        pos = project_to_3d(det.pixel_x, det.pixel_y, z_cm, _intrinsics)
        result.update({
            "pixel_x": det.pixel_x, "pixel_y": det.pixel_y,
            "angle_deg": det.angle_deg,
            "x_cm": pos.x_cm, "y_cm": pos.y_cm, "z_cm": pos.z_cm,
            "z_source": "sensor" if used_real_sensor else "estimated_from_size",
        })
        box = det.box.astype(int)
        cv2.drawContours(display, [box], 0, (0, 255, 0), 2)
        cv2.circle(display, (int(det.pixel_x), int(det.pixel_y)), 5, (0, 0, 255), -1)
        label = f"X={pos.x_cm:.1f} Y={pos.y_cm:.1f} Z~{pos.z_cm:.1f}cm"
        cv2.putText(display, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        result["found"] = False
        cv2.putText(display, "no object detected", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    ok, jpeg = cv2.imencode(".jpg", display)
    with _lock:
        _state["last_frame"] = jpeg.tobytes() if ok else None
        _state["last_detection"] = result
        _state["last_update_ts"] = time.time()

    return JSONResponse(result)


@app.get("/latest")
def latest():
    with _lock:
        if _state["last_detection"] is None:
            return JSONResponse({"error": "no frames received yet"}, status_code=404)
        return JSONResponse(_state["last_detection"])


@app.get("/debug_feed")
def debug_feed():
    """MJPEG stream of the last annotated frame — open in a browser to
    watch detection live while the rover is moving, without needing
    physical access to it."""
    def generate():
        boundary = b"--frame"
        while True:
            with _lock:
                frame = _state["last_frame"]
            if frame is not None:
                yield (boundary + b"\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.05)
    return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/")
def root():
    return {
        "status": "ok",
        "endpoints": ["/ingest (POST, multipart: frame + depth_cm)", "/latest (GET)", "/debug_feed (GET, MJPEG)"],
        "calibrated": _intrinsics is not None,
    }
