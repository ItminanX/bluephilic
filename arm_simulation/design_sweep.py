"""
design_sweep.py - with the camera TILT fixed at 60 deg (devkit/OV2640 mounting, cannot be changed), which
mechanical/pose choices make the object visible?  Free choices: where the camera bracket sits along the M3 arm
and the search (home) pose.  Robust = worst case over camera tilt 55/60/65 deg (measurement uncertainty).
  acquire      : floor objects (x -12..12, y 14..30 cm) visible from the search pose, 25 px margin
  keep_in_view : same objects visible at the pre-grasp pose (TCP 6 cm above the object)
"""
import json, itertools
from dataclasses import replace
import numpy as np
from arm_model import Arm, ArmParams

XS = np.arange(-12, 12.1, 4.0)
YS = np.arange(14, 30.1, 4.0)
M = 25


def visible(arm, q, pt):
    uv, pc = arm.project(q, pt)
    return uv is not None and pc[2] >= 8 and M <= uv[0] <= arm.K.w - M and M <= uv[1] <= arm.K.h - M


def score(P, q_home):
    arm = Arm(P)
    a = b = n = 0
    for x in XS:
        for y in YS:
            pt = np.array([x, y, 2.5]); n += 1
            a += visible(arm, q_home, pt)
            qp = arm.ik(pt + [0, 0, 6.0])
            if qp is not None:
                b += visible(arm, qp, pt)
    return a / n, b / n


def robust(P, q_home):
    ss = [score(replace(P, cam_pitch_deg=p), q_home) for p in (55.0, 60.0, 65.0)]
    return min(min(s) for s in ss), float(np.mean([sum(s) for s in ss]))


if __name__ == "__main__":
    base = ArmParams()
    out = {}
    for along in (7.6, 10.0, 12.0, 13.0, 14.0):
        P = replace(base, cam_along=along)
        rows = []
        for th2, th3 in itertools.product(range(40, 91, 10), range(30, 121, 15)):
            if Arm(P).collides((0, th2, th3)):
                continue
            w, m = robust(P, np.array([0.0, th2, th3]))
            rows.append(dict(home_th2=th2, home_th3=th3, worst=round(w, 2), mean=round(m, 2)))
        rows.sort(key=lambda r: (-r["worst"], -r["mean"]))
        out[f"cam_along_{along}"] = rows[:3]
        print("cam_along", along, rows[:2])
    json.dump(out, open("results/design_sweep.json", "w"), indent=1)
