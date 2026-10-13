"""Run:  python test_model.py     (plain asserts, no pytest needed)"""
import numpy as np
from arm_model import Arm, ArmParams
from arm_controller import Controller, State
from run_sim import run, still_obj
from designs import DESIGNS


def test_neutral_pose_matches_builder_description():
    f = Arm().fk((0, 90, 90))
    assert abs(f["alpha3_deg"] - 30.0) < 1e-9, "M2=90,M3=90 must put the M3 arm 60 deg from vertical"
    assert abs(f["m3"][0]) < 1e-9 and abs(f["m3"][1]) < 1e-9, "M2 straight up -> M3 pivot directly above M2"
    assert abs(f["m3"][2] - (10 + 3.6 + 15.6)) < 1e-9
    assert f["tcp"][1] > 0, "gripper must point forward at the mean pose"


def test_pan_sign():
    f = Arm().fk((90, 90, 90))
    assert f["tcp"][0] > 10 and abs(f["tcp"][1]) < 1e-6, "pan +90 swings the arm to the right (+X)"


def test_fk_ik_roundtrip():
    a, rng = Arm(), np.random.default_rng(0)
    n = 0
    while n < 2000:
        q = np.array([rng.uniform(-90, 90), rng.uniform(0, 180), rng.uniform(0, 180)])
        f = a.fk(q)
        if a.collides(q) or min(f["m3"][2], f["s1"][2]) < a.p.min_joint_z:
            continue                      # poses the model deliberately refuses (chassis / ground)
        n += 1
        t = f["tcp"]
        s = a.ik(t, q_ref=q)
        assert s is not None and np.linalg.norm(a.fk(s)["tcp"] - t) < 1e-9


def test_unreachable_returns_none():
    a = Arm()
    assert a.ik((0, 60, 2.5)) is None and a.ik((0, 0, 100)) is None
    assert a.ik((0, -13, 15)) is None, "rear deck at low height is outside the M3 fold range (documented limitation)"


def test_camera_transform_roundtrip_and_handedness():
    a = Arm(ArmParams(cam_pitch_deg=40.0))
    q = (25, 70, 45)
    assert abs(np.linalg.det(a.fk(q)["R_wc"]) - 1) < 1e-9
    p = np.array([5.0, 30.0, 3.0])
    assert np.allclose(a.cam_to_world(q, a.world_to_cam(q, p)), p)
    uv, pc = a.project(q, a.cam_to_world(q, (0, 0, 20)))
    assert abs(uv[0] - a.K.cx) < 1e-6 and abs(uv[1] - a.K.cy) < 1e-6, "a point on the optical axis projects to the principal point"


def test_servo_mapping():
    d = Arm().to_servo_deg((-90, 90, 90), 30)
    assert d["M1"] == 0 and d["M2"] == 90 and d["S1"] == 30


def test_dwell_rule():
    P, cfg = DESIGNS["camera_forward"]
    s, rec = run("t", still_obj([6, 24, 2.5]), 45, cfg=cfg, params=P)
    t_track = next(t for t, st in zip(rec["t"], rec["state"]) if st == State.TRACK.value)
    t_appr = next(t for t, st in zip(rec["t"], rec["state"]) if st == State.APPROACH.value)
    assert t_appr - t_track >= cfg.dwell_s - 0.5, "grab must not start before ~10 s of stillness"
    assert s["object_delivered"] and s["min_joint_or_tcp_height_cm"] > 0 and s["chassis_collision_frames"] == 0


def test_no_grab_while_object_keeps_moving():
    P, cfg = DESIGNS["camera_forward"]
    wander = lambda t: np.array([10 * np.sin(t / 2.0), 24 + 3 * np.sin(t / 3.0), 2.5])
    s, _ = run("t", wander, 40, cfg=cfg, params=P)
    assert "APPROACH" not in s["states_visited"]


if __name__ == "__main__":
    for n, fn in list(globals().items()):
        if n.startswith("test_"):
            fn(); print("PASS", n)
