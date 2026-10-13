"""Writes sample_logs/synthetic_still_object.jsonl: what server.py would return for a blue 5 cm object resting at
(6, 24, 2.5) cm on the floor, seen from a camera held fixed at the search pose (30 fps, 25 s, ~25 % dropouts)."""
import json, os
import numpy as np
from arm_model import Arm
from sim_camera import SimCamera
from designs import DESIGNS

def make(path="sample_logs/synthetic_still_object.jsonl", design="camera_forward", obj=(6, 24, 2.5), seconds=25, fps=30, seed=7):
    P, cfg = DESIGNS[design]
    arm = Arm(P); cam = SimCamera(arm, seed=seed); q = np.array(cfg.home_q, float)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        for i in range(int(seconds * fps)):
            fr = cam.observe(q, obj)
            d = {"timestamp": 1790000000.0 + i / fps, "found": bool(fr.found)}
            if fr.found:
                d.update(pixel_x=round(fr.u, 1), pixel_y=round(fr.v, 1), x_cm=fr.x_cm, y_cm=fr.y_cm, z_cm=fr.z_cm, z_source="estimated_from_size")
            f.write(json.dumps(d) + "\n")
    return path

if __name__ == "__main__":
    print("wrote", make())
