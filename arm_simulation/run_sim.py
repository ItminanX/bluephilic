"""
run_sim.py - closed-loop simulation: synthetic object + camera ON the arm + controller + servo model.
Usage:  python run_sim.py            (runs all scenarios, writes results/*.png and results/summary.json)
"""
import json, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from arm_model import Arm
from sim_camera import SimCamera, CameraNoise
from arm_controller import Controller, Config, State

DT = 1 / 30.0          # vision/control rate (your stream gives 30-47 fps)
GRASP_TOL = 2.0        # cm, TCP-to-object distance accepted as a successful grasp (ASSUMPTION)


def slew(cur, tgt, max_step):
    return cur + np.clip(tgt - cur, -max_step, max_step)


def run(name, obj_fn, T=45.0, seed=1, cfg=Config(), noise=CameraNoise(), params=None, true_params=None, bias=(0, 0, 0)):
    """params = what the controller believes; true_params/bias = what the physical arm really is."""
    from arm_model import ArmParams
    arm = Arm(params or ArmParams())
    plant = Arm(true_params or params or ArmParams())
    bias = np.asarray(bias, float)
    cam = SimCamera(plant, noise, seed)
    ctl = Controller(arm, cfg)
    q = np.array(cfg.home_q, float)
    grip = arm.p.grip_open_deg
    q_prev = q.copy()
    attached, delivered = False, False
    rec = dict(t=[], q=[], state=[], err=[], obj=[], tcp=[], grip=[], found=[], minz=[], hit=[])
    grasp_err = None
    t = 0.0
    while t < T:
        obj = None if delivered else (plant.fk(q + bias)["tcp"] if attached else obj_fn(t))
        frame = cam.observe(q_prev + bias, obj)                    # frame was captured one step ago
        q_tgt, g_tgt, st = ctl.step(t, frame, q_prev, q)
        prev_state = ctl.log[-1][1] if ctl.log else None
        q_prev = q.copy()
        q = slew(q, q_tgt, cfg.max_rate * DT)
        grip = slew(grip, g_tgt, 130.0 / 0.6 * DT)
        f = plant.fk(q + bias)
        if st == State.CLOSE and not attached and not delivered and grasp_err is None:
            true_obj = obj_fn(t)
            grasp_err = float(np.linalg.norm(f["tcp"] - true_obj))
            if grasp_err <= GRASP_TOL and grip > arm.p.grip_open_deg + 5:
                attached = True
        if st == State.CLOSE and not attached and grasp_err is not None and grip > 100 and grasp_err <= GRASP_TOL:
            attached = True
        if st == State.RELEASE and attached and grip < arm.p.grip_closed_deg - 40:
            attached, delivered = False, True
        rec["t"].append(t); rec["q"].append(q.copy()); rec["state"].append(st.value)
        rec["tcp"].append(f["tcp"]); rec["grip"].append(grip); rec["found"].append(frame.found); rec.setdefault("exists", []).append(obj is not None and not attached)
        rec["obj"].append(np.full(3, np.nan) if obj is None else np.asarray(obj))
        rec["err"].append(np.nan if ctl.est is None or obj is None else float(np.linalg.norm(ctl.est - obj)))
        rec["minz"].append(min(f["m3"][2], f["s1"][2], f["tcp"][2])); rec["hit"].append(plant.collides(q + bias))
        t += DT
    for k in rec:
        rec[k] = np.array(rec[k])
    rec['exists'] = rec['exists'].astype(bool)
    steps = np.abs(np.diff(rec["q"], axis=0)) / DT
    states_seen = [s for _, s in ctl.log]
    summary = dict(
        scenario=name,
        states_visited=states_seen,
        object_delivered=bool(delivered),
        grasp_error_cm=grasp_err,
        detection_rate_while_object_visible_to_world=float(rec["found"][rec["exists"]].mean()),
        need_closer_flag=bool(ctl.need_closer),
        world_estimate_error_median_cm=float(np.nanmedian(rec["err"])) if np.isfinite(rec["err"]).any() else None,
        world_estimate_error_p95_cm=float(np.nanpercentile(rec["err"], 95)) if np.isfinite(rec["err"]).any() else None,
        max_servo_rate_deg_s=float(steps.max()),
        min_joint_or_tcp_height_cm=float(rec["minz"].min()),
        chassis_collision_frames=int(np.sum(rec["hit"])),
    )
    return summary, rec


# ---------------------------------------------------------------- scenarios
def still_obj(p):
    return lambda t: np.array(p, float)


def moving_then_still(t):
    """slides sideways and forward for 8 s, then rests."""
    p0, p1 = np.array([-8.0, 22.0, 2.5]), np.array([8.0, 26.0, 2.5])
    s = np.clip(t / 8.0, 0, 1)
    return p0 + s * (p1 - p0)


def plot_run(rec, title, path):
    fig, ax = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    t = rec["t"]
    ax[0].plot(t, rec["q"][:, 0], label="M1 pan"); ax[0].plot(t, rec["q"][:, 1], label="M2"); ax[0].plot(t, rec["q"][:, 2], label="M3")
    ax[0].plot(t, rec["grip"], label="S1 grip", ls="--"); ax[0].set_ylabel("deg"); ax[0].legend(ncol=4, fontsize=8)
    ax[1].plot(t, rec["tcp"][:, 0], label="TCP x"); ax[1].plot(t, rec["tcp"][:, 1], label="TCP y"); ax[1].plot(t, rec["tcp"][:, 2], label="TCP z")
    ax[1].plot(t, rec["obj"][:, 1], "k:", label="object y"); ax[1].set_ylabel("cm"); ax[1].legend(ncol=4, fontsize=8)
    ax[2].plot(t, rec["err"]); ax[2].set_ylabel("world est. error (cm)")
    ax[3].step(t, rec["state"], where="post"); ax[3].set_ylabel("state id"); ax[3].set_xlabel("time (s)")
    ax[3].set_yticks(range(8)); ax[3].set_yticklabels([s.name for s in State], fontsize=7)
    fig.suptitle(title); fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


def plot_workspace(path):
    arm = Arm()
    p = arm.p
    # side view: reachable TCP points at pan=0 from sampling joint space
    th2, th3 = np.meshgrid(np.linspace(0, 180, 181), np.linspace(0, 180, 181))
    pts = np.array([arm.fk((0, a, b))["tcp"] for a, b in zip(th2.ravel(), th3.ravel())])
    fig, ax = plt.subplots(1, 2, figsize=(13, 6))
    ax[0].scatter(pts[:, 1], pts[:, 2], s=1, alpha=0.25, label="TCP reachable (joint sampling)")
    for q, lab, c in [((0, 90, 90), "home", "g"), ((0, 60, 60), "pose B", "r")]:
        f = arm.fk(q); xs = [0, 0, f["m3"][1], f["s1"][1], f["tcp"][1]]; zs = [0, p.z_m2, f["m3"][2], f["s1"][2], f["tcp"][2]]
        ax[0].plot(xs, zs, "-o", c=c, label=lab)
    ax[0].axhline(0, c="k"); ax[0].set_aspect("equal"); ax[0].set_xlabel("Y forward (cm)"); ax[0].set_ylabel("Z (cm)")
    ax[0].set_title("Side view, pan = 0"); ax[0].legend(fontsize=8)
    # top view: where can a 5 cm object lying on the floor (z=2.5) be grasped?
    xs, ys = np.meshgrid(np.arange(-40, 40.1, 1.0), np.arange(-20, 40.1, 1.0))
    ok = np.array([arm.ik((x, y, 2.5)) is not None for x, y in zip(xs.ravel(), ys.ravel())]).reshape(xs.shape)
    ax[1].imshow(ok, origin="lower", extent=[-40.5, 40.5, -20.5, 40.5], cmap="Greens", alpha=0.8)
    ax[1].plot(0, 0, "k^", label="arm base (rover)"); ax[1].set_xlabel("X right (cm)"); ax[1].set_ylabel("Y forward (cm)")
    ax[1].set_title("Floor-grasp envelope (object centre z = 2.5 cm)"); ax[1].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)
    yy = ys[ok]; rr = np.hypot(xs[ok], ys[ok])
    return dict(floor_grasp_y_min_cm=float(yy.min()), floor_grasp_y_max_cm=float(yy.max()),
                floor_grasp_radius_max_cm=float(rr.max()),
                floor_grasp_radius_min_cm=float(np.hypot(xs[ok], ys[ok]).min()),
                tcp_z_range_cm=[float(pts[:, 2].min()), float(pts[:, 2].max())],
                tcp_max_forward_cm=float(pts[:, 1].max()))


def main():
    from designs import DESIGNS
    out = {"workspace": plot_workspace("results/workspace.png"), "designs": {}}
    scen = [
        ("A_still_object_in_reach", still_obj([6, 24, 2.5]), 45),
        ("B_moving_then_still", moving_then_still, 50),
        ("C_object_out_of_reach", still_obj([0, 55, 2.5]), 30),
    ]
    for dname, (P, cfg) in DESIGNS.items():
        d = {"scenarios": []}
        for name, fn, T in scen:
            s, rec = run(name, fn, T, cfg=cfg, params=P)
            d["scenarios"].append(s)
            plot_run(rec, f"{dname}: {name}", f"results/{dname}_{name}.png")
        wins, errs = 0, []
        for sd in range(20):
            s, _ = run("mc", still_obj([6, 24, 2.5]), 45, seed=100 + sd, cfg=cfg, params=P)
            wins += s["object_delivered"]
            if s["grasp_error_cm"] is not None:
                errs.append(s["grasp_error_cm"])
        d["noise_only_monte_carlo_A"] = dict(runs=20, delivered=wins)
        out["designs"][dname] = d
    json.dump(out, open("results/summary.json", "w"), indent=2)
    for dname, d in out["designs"].items():
        print(dname)
        for s in d["scenarios"]:
            print("  ", s["scenario"], "delivered", s["object_delivered"], "need_closer", s["need_closer_flag"],
                  "switches", len(s["states_visited"]), "det", round(s["detection_rate_while_object_visible_to_world"], 2))
        print("   noise-only MC:", d["noise_only_monte_carlo_A"])


if __name__ == "__main__":
    main()
