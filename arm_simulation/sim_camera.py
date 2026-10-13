"""
sim_camera.py - synthetic version of what the ESP32 + server.py would send.
Mimics the real pipeline: pixel centroid + size-based depth -> camera-frame X,Y,Z (cm),
with the noise, dropouts and latency seen in your Serial logs (found:false ~25-35% of frames).
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class VisionFrame:
    found: bool
    x_cm: float = 0.0       # camera frame: X right, Y down, Z forward (same as server.py output)
    y_cm: float = 0.0
    z_cm: float = 0.0
    u: float = 0.0
    v: float = 0.0


@dataclass
class CameraNoise:
    pixel_sigma: float = 1.5      # centroid jitter (px)
    size_sigma_px: float = 1.0    # apparent-size jitter (px) -> depth error
    dropout: float = 0.25         # probability the detector returns found:false
    min_range_cm: float = 6.0
    max_range_cm: float = 80.0
    margin_px: int = 6
    object_size_cm: float = 5.0   # OBJECT_SIZE_CM in server.py


class SimCamera:
    def __init__(self, arm, noise=CameraNoise(), seed=1):
        self.arm, self.noise = arm, noise
        self.rng = np.random.default_rng(seed)

    def observe(self, q_at_capture, obj_world):
        K, n = self.arm.K, self.noise
        if obj_world is None:
            return VisionFrame(False)
        uv, pc = self.arm.project(q_at_capture, obj_world)
        if uv is None or not (n.min_range_cm <= pc[2] <= n.max_range_cm):
            return VisionFrame(False)
        u, v = uv
        if not (n.margin_px <= u <= K.w - n.margin_px and n.margin_px <= v <= K.h - n.margin_px):
            return VisionFrame(False)
        if self.rng.random() < n.dropout:
            return VisionFrame(False)
        u += self.rng.normal(0, n.pixel_sigma)
        v += self.rng.normal(0, n.pixel_sigma)
        apparent = n.object_size_cm * K.fx / pc[2] + self.rng.normal(0, n.size_sigma_px)
        z = n.object_size_cm * K.fx / max(apparent, 1.0)             # = estimate_depth_from_size()
        return VisionFrame(True, (u - K.cx) * z / K.fx, (v - K.cy) * z / K.fy, z, u, v)
