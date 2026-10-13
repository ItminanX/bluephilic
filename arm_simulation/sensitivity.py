"""
sensitivity.py - what the clean simulation hides: model error.
  1. one-at-a-time errors    -> how much does each hand-measured number hurt the grasp?
  2. Monte-Carlo mismatch    -> random but plausible calibration errors on every parameter at once
(the camera-pitch / search-pose trade-off lives in design_sweep.py)
"""
import json
from dataclasses import replace
import numpy as np
from arm_model import Arm, ArmParams
from run_sim import run, still_obj
from designs import DESIGNS

OBJ = [6, 24, 2.5]


def one_at_a_time(base, cfg):
    cases = {
        "L2 +0.5 cm": dict(L2=base.L2 + .5), "L3 +0.5 cm": dict(L3=base.L3 + .5),
        "grip_reach +1.5 cm": dict(grip_reach=base.grip_reach + 1.5),
        "m2_rise +0.5 cm": dict(m2_rise=base.m2_rise + .5), "base_height +0.5 cm": dict(base_height=base.base_height + .5),
        "cam_pitch +5 deg": dict(cam_pitch_deg=base.cam_pitch_deg + 5), "cam_along +0.5 cm": dict(cam_along=base.cam_along + .5),
        "cam_perp +0.5 cm": dict(cam_perp=base.cam_perp + .5),
        "m3_rel_at_mean +5 deg": dict(m3_rel_at_mean_deg=base.m3_rel_at_mean_deg + 5),
    }
    out = []
    for k, kw in cases.items():
        s, _ = run(k, still_obj(OBJ), 45, cfg=cfg, params=base, true_params=replace(base, **kw))
        out.append(dict(error=k, delivered=s["object_delivered"], grasp_error_cm=None if s["grasp_error_cm"] is None else round(s["grasp_error_cm"], 2)))
    for j, nm in enumerate(["M1", "M2", "M3"]):
        b = [0, 0, 0]; b[j] = 2.0
        s, _ = run(nm, still_obj(OBJ), 45, cfg=cfg, params=base, bias=b)
        out.append(dict(error=f"{nm} servo zero offset +2 deg", delivered=s["object_delivered"], grasp_error_cm=None if s["grasp_error_cm"] is None else round(s["grasp_error_cm"], 2)))
    return out


def monte_carlo(base, cfg, n=40, tol=2.0):
    errs = []; wins = 0
    for i in range(n):
        rng = np.random.default_rng(1000 + i)
        tp = replace(base, L2=base.L2 + rng.normal(0, .3), L3=base.L3 + rng.normal(0, .3), grip_reach=base.grip_reach + rng.normal(0, .8),
                     m2_rise=base.m2_rise + rng.normal(0, .3), base_height=base.base_height + rng.normal(0, .3),
                     cam_pitch_deg=base.cam_pitch_deg + rng.normal(0, 3), cam_along=base.cam_along + rng.normal(0, .3),
                     cam_perp=base.cam_perp + rng.normal(0, .3), m3_rel_at_mean_deg=base.m3_rel_at_mean_deg + rng.normal(0, 2))
        bias = rng.normal(0, 1.5, 3)
        s, _ = run("mc", still_obj(OBJ), 45, seed=i, cfg=cfg, params=base, true_params=tp, bias=bias)
        if s["grasp_error_cm"] is not None:
            errs.append(s["grasp_error_cm"])
        wins += s["grasp_error_cm"] is not None and s["grasp_error_cm"] <= tol
    errs = np.array(errs)
    return dict(runs=n, grasp_within_tol=int(wins), tol_cm=tol, grasp_error_median_cm=round(float(np.median(errs)), 2),
                grasp_error_p90_cm=round(float(np.percentile(errs, 90)), 2), grasp_error_max_cm=round(float(errs.max()), 2),
                assumed_1sigma="lengths +-0.3 cm (grip_reach +-0.8), camera pitch +-3 deg, M3 offset +-2 deg, servo zero +-1.5 deg")


if __name__ == "__main__":
    res = {d: dict(one_at_a_time=one_at_a_time(P, c), monte_carlo_mismatch=monte_carlo(P, c)) for d, (P, c) in DESIGNS.items()}
    json.dump(res, open("results/sensitivity.json", "w"), indent=2)
    print(json.dumps(res, indent=1))
