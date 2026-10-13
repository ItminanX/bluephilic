"""
live_bridge.py - run YOUR real vision output through the arm controller ("shadow mode", no servos needed).

  LIVE    python live_bridge.py --url http://localhost:8000/latest [--record logs/session1.jsonl]
  REPLAY  python live_bridge.py --replay logs/session1.jsonl
  OPTIONS --design camera_forward|as_measured   --rate 25   --csv logs/out.csv   --seconds 60   --quiet

What it does: reads the server's detection JSON (camera-frame x_cm, y_cm, z_cm), runs the same Controller as the
simulator, and prints the servo numbers (0..180) it WOULD send, plus the state (IDLE/TRACK/APPROACH/...).

IMPORTANT - what shadow mode can and cannot tell you:
  * The real camera does not move (the arm is not built), so the bridge assumes the camera sits fixed at the arm's
    SEARCH (home) pose. Hold/mount the ESP32 where the printed 'camera at home pose' says, tilted as printed.
  * It tests plumbing, units, filtering, the 10 s dwell rule and the state sequence on real data.
  * It cannot test arm motion or grasp accuracy - that stays in the simulator and, later, the hardware.
"""
import argparse, csv, json, os, sys, time, urllib.request, urllib.error
import numpy as np
from arm_model import Arm
from sim_camera import VisionFrame
from arm_controller import Controller
from designs import DESIGNS


def frame_from_json(d):
    """server.py detection dict -> VisionFrame (camera frame, cm). Anything unexpected counts as 'not found'."""
    try:
        if not d.get("found"):
            return VisionFrame(False)
        return VisionFrame(True, float(d["x_cm"]), float(d["y_cm"]), float(d["z_cm"]),
                           float(d.get("pixel_x", 0.0)), float(d.get("pixel_y", 0.0)))
    except (KeyError, TypeError, ValueError):
        return VisionFrame(False)


class Shadow:
    def __init__(self, design="camera_forward"):
        P, self.cfg = DESIGNS[design]
        self.arm = Arm(P)
        self.ctl = Controller(self.arm, self.cfg)
        self.q = np.array(self.cfg.home_q, float)
        self.grip = P.grip_open_deg
        self.t_prev = None
        self.q_cam = np.array(self.cfg.home_q, float)          # the real camera is stationary at the home pose

    def camera_pose_text(self):
        f = self.arm.fk(self.q_cam)
        return ("camera at home pose: position (x %.1f, y %.1f, z %.1f) cm from the ground point under the M1 axis; "
                "optical axis (%.2f, %.2f, %.2f)" % (*f["cam_pos"], *f["R_wc"][:, 2]))

    def step(self, t, d):
        dt = 1 / 30 if self.t_prev is None else float(np.clip(t - self.t_prev, 1e-3, 0.25))
        self.t_prev = t
        q_tgt, g_tgt, st = self.ctl.step(t, frame_from_json(d), self.q_cam, self.q)
        r = self.cfg.max_rate * dt
        self.q = self.q + np.clip(q_tgt - self.q, -r, r)
        self.grip = self.grip + np.clip(g_tgt - self.grip, -130 / 0.6 * dt, 130 / 0.6 * dt)
        servo = self.arm.to_servo_deg(self.q, self.grip)
        return st, servo


def fmt(t, d, st, servo, ctl):
    est = "  --  " if ctl.est is None else "(%5.1f,%5.1f,%5.1f)" % tuple(ctl.est)
    return "t=%6.1fs %-8s found=%d world=%s M1=%3.0f M2=%3.0f M3=%3.0f S1=%3.0f%s" % (
        t, st.name, int(bool(d.get("found"))), est, servo["M1"], servo["M2"], servo["M3"], servo["S1"],
        "  [object too far: rover should move closer]" if ctl.need_closer else "")


def run_stream(items, args, sh, record=None, csvw=None):
    t0, last_print = None, -1e9
    for d in items:
        t = d["_t"]
        t0 = t if t0 is None else t0
        ts = t - t0
        st, servo = sh.step(ts, d)
        if record:
            record.write(json.dumps({k: v for k, v in d.items() if k != "_t"}) + "\n")
        if csvw:
            csvw.writerow([round(ts, 3), st.name, int(bool(d.get("found"))), *(["", "", ""] if sh.ctl.est is None else np.round(sh.ctl.est, 2)),
                           round(servo["M1"], 1), round(servo["M2"], 1), round(servo["M3"], 1), round(servo["S1"], 1)])
        if not args.quiet and ts - last_print >= 0.5:
            print(fmt(ts, d, st, servo, sh.ctl)); last_print = ts
        if args.seconds and ts >= args.seconds:
            break
    return sh


def live_items(url, rate, seconds):
    last_ts, start = None, time.time()
    while True:
        if seconds and time.time() - start > seconds:
            return
        try:
            with urllib.request.urlopen(url, timeout=2) as r:
                d = json.loads(r.read().decode())
        except urllib.error.HTTPError as e:
            if e.code == 404:
                print("waiting: server has no frames yet (is the ESP32 streaming?)"); time.sleep(1); continue
            print("HTTP error", e.code); time.sleep(1); continue
        except (urllib.error.URLError, TimeoutError, ConnectionError, OSError) as e:
            print("cannot reach", url, "-", e, "(is uvicorn running?)"); time.sleep(1); continue
        ts = d.get("timestamp")
        if ts is None or ts == last_ts:                       # no new frame yet
            time.sleep(1.0 / rate); continue
        last_ts = ts
        d["_t"] = float(ts)
        yield d
        time.sleep(1.0 / rate)


def replay_items(path):
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                d = json.loads(line)
                d["_t"] = float(d["timestamp"])
                yield d


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url"); ap.add_argument("--replay")
    ap.add_argument("--design", default="camera_forward", choices=list(DESIGNS))
    ap.add_argument("--rate", type=float, default=25.0); ap.add_argument("--seconds", type=float, default=0)
    ap.add_argument("--record"); ap.add_argument("--csv"); ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args(argv)
    if bool(a.url) == bool(a.replay):
        ap.error("give exactly one of --url (live) or --replay FILE")
    sh = Shadow(a.design)
    print(sh.camera_pose_text())
    rec = None
    csvf = None
    if a.record:
        os.makedirs(os.path.dirname(a.record) or ".", exist_ok=True); rec = open(a.record, "w")
    csvw = None
    if a.csv:
        os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True); csvf = open(a.csv, "w", newline="")
        csvw = csv.writer(csvf); csvw.writerow(["t_s", "state", "found", "wx", "wy", "wz", "M1", "M2", "M3", "S1"])
    try:
        items = replay_items(a.replay) if a.replay else live_items(a.url, a.rate, a.seconds)
        run_stream(items, a, sh, rec, csvw)
    except KeyboardInterrupt:
        pass
    finally:
        for fh in (rec, csvf):
            if fh:
                fh.close()
    print("states visited:", " -> ".join(s for _, s in sh.ctl.log) or "(none)")
    return sh


if __name__ == "__main__":
    main()
