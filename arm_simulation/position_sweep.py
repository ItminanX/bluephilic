"""
position_sweep.py - the test that decides between camera placements.
Object placed at 15 floor positions; the REAL camera tilt is 55 / 60 / 65 deg while the controller believes 60.
Counts completed grabs (object delivered) per design.
"""
import json
from dataclasses import replace
import numpy as np
from run_sim import run, still_obj
from designs import DESIGNS

POS = [(x, y) for x in (-12, -6, 0, 6, 12) for y in (16, 22, 28)]

if __name__ == "__main__":
    out = {}
    for dname, (P, cfg) in DESIGNS.items():
        out[dname] = {}
        for pitch in (55.0, 60.0, 65.0):
            ok, lost, errs = 0, [], []
            for (x, y) in POS:
                s, _ = run("p", still_obj([x, y, 2.5]), 40, cfg=cfg, params=P, true_params=replace(P, cam_pitch_deg=pitch))
                if s["object_delivered"]:
                    ok += 1
                else:
                    lost.append((x, y))
                if s["grasp_error_cm"] is not None:
                    errs.append(s["grasp_error_cm"])
            out[dname][f"true_tilt_{int(pitch)}"] = dict(delivered=f"{ok}/{len(POS)}", never_grabbed=lost,
                                                        median_grasp_err_cm=round(float(np.median(errs)), 2) if errs else None)
        print(dname, json.dumps(out[dname]))
    json.dump(out, open("results/position_sweep.json", "w"), indent=1)
