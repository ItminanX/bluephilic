"""
viewer_server.py - runs the arm controller and shows it in the 3D viewer in your web browser.

  DEMO (no hardware):   python viewer_server.py --demo
  LIVE (your camera):   python viewer_server.py --live http://localhost:8000/latest
  then open             http://localhost:8010

  --design camera_forward|as_measured   --port 8010   --speed 1.0 (demo only: 2.0 = twice as fast)   --host 0.0.0.0 (let other devices connect)

DEMO : a pretend blue object slides around on the floor, rests, and the simulated arm tracks it, waits 10 s, grabs it
       and drops it in the basket. Then a new object appears. It uses the SAME controller as run_sim.py.
LIVE : your real ESP32 camera sees a real object (held still at the search pose, see live_bridge.py), the controller turns
       that into arm movements, and the 3D arm in the browser moves as the real arm would. Nothing physical moves.
"""
import argparse, json, os, threading, time, urllib.error, urllib.request
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
from arm_model import Arm
from sim_camera import SimCamera, VisionFrame
from arm_controller import Controller, State
from designs import DESIGNS
from live_bridge import frame_from_json

DT = 1 / 30.0
GRASP_TOL = 2.0                                   # cm: same grasp rule as run_sim.py
HERE = os.path.dirname(os.path.abspath(__file__))


def L(v):
    return None if v is None else [float(x) for x in v]


class Base:
    mode = "base"

    def __init__(self, design="camera_forward"):
        self.design = design
        self.P, self.cfg = DESIGNS[design]
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        self.arm = Arm(self.P)
        self.ctl = Controller(self.arm, self.cfg)
        self.q = np.array(self.cfg.home_q, float)
        self.q_prev = self.q.copy()
        self.grip = self.P.grip_open_deg
        self.t = 0.0
        self.obj = None
        self.raw = None
        self.found = False
        self.attached = False
        self.state = State.IDLE
        self.link = "ok"

    def _move(self, q_tgt, g_tgt, dt):
        """servo model: each joint moves toward its target no faster than max_rate."""
        r = self.cfg.max_rate * dt
        self.q_prev = self.q.copy()
        self.q = self.q + np.clip(q_tgt - self.q, -r, r)
        self.grip += float(np.clip(g_tgt - self.grip, -130 / 0.6 * dt, 130 / 0.6 * dt))

    def config(self):
        return dict(params=asdict(self.P), K=asdict(self.arm.K), basket=list(self.cfg.basket), home=list(self.cfg.home_q),
                    design=self.design, mode=self.mode)

    def snapshot(self):
        f = self.arm.fk(self.q)
        servo = {k: round(float(v), 1) for k, v in self.arm.to_servo_deg(self.q, self.grip).items()}
        return dict(t=round(self.t, 2), mode=self.mode, state=self.state.name, found=bool(self.found), q=L(self.q),
                    grip=float(self.grip), servo=servo, tcp=L(f["tcp"]), obj=L(self.obj), est=L(self.ctl.est), raw=L(self.raw),
                    attached=bool(self.attached), need_closer=bool(self.ctl.need_closer), link=self.link)


class Demo(Base):
    """Pretend world: an object slides, rests; arm tracks, waits 10 s, grabs, drops in the basket; next object appears."""
    mode = "demo"
    EPISODES = [((-8, 22), (8, 26), 7.0), ((10, 17), (-6, 27), 6.0), ((-11, 26), (2, 18), 6.0), ((6, 24), (6, 24), 0.0)]

    def reset(self):
        super().reset()
        self.cam = SimCamera(self.arm, seed=3)
        self.ep = 0
        self.deliveries = 0
        self._start_episode()

    def _start_episode(self):
        self.spec = self.EPISODES[self.ep % len(self.EPISODES)]
        self.t_ep, self.delivered, self.attached, self.grasp_err, self.wait = 0.0, False, False, None, 0.0

    def obj_at(self, te):
        a, b, T = self.spec
        s = 1.0 if T <= 0 else min(1.0, max(0.0, te / T))
        return np.array([a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1]), 2.5])

    def tick(self, dt=DT):
        self.t += dt
        if self.delivered:
            self.wait += dt
            if self.wait > 2.5 and self.state == State.IDLE:
                self.ep += 1
                self._start_episode()
        else:
            self.t_ep += dt
        obj = None if self.delivered else (self.arm.fk(self.q)["tcp"] if self.attached else self.obj_at(self.t_ep))
        frame = self.cam.observe(self.q_prev, obj)               # frame captured one step ago, like run_sim.py
        q_tgt, g_tgt, st = self.ctl.step(self.t, frame, self.q_prev, self.q)
        self._move(q_tgt, g_tgt, dt)
        tcp = self.arm.fk(self.q)["tcp"]
        if st == State.TRACK:
            self.grasp_err = None                                 # allows a retry if a grasp was missed
        if st == State.CLOSE and not self.attached and not self.delivered and self.grasp_err is None:
            self.grasp_err = float(np.linalg.norm(tcp - self.obj_at(self.t_ep)))
            if self.grasp_err <= GRASP_TOL and self.grip > self.P.grip_open_deg + 5:
                self.attached = True
        if st == State.CLOSE and not self.attached and self.grasp_err is not None and self.grip > 100 and self.grasp_err <= GRASP_TOL:
            self.attached = True
        if st == State.RELEASE and self.attached and self.grip < self.P.grip_closed_deg - 40:
            self.attached, self.delivered, self.wait = False, True, 0.0
            self.deliveries += 1
        self.state, self.found = st, frame.found
        self.obj = None if self.delivered else (tcp if self.attached else self.obj_at(self.t_ep))


class Live(Base):
    """Real camera: the object comes from your vision server; the real camera is assumed fixed at the search pose."""
    mode = "live"

    def __init__(self, design, url):
        self.url, self.latest, self.latest_wall, self.seen = url, None, 0.0, None
        super().__init__(design)
        threading.Thread(target=self._poll, daemon=True).start()

    def reset(self):
        super().reset()
        self.q_cam = np.array(self.cfg.home_q, float)
        self.t0 = time.time()

    def _poll(self):
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=2) as r:
                    d = json.loads(r.read().decode())
                self.link = "ok"
                if d.get("timestamp") is not None:
                    self.latest, self.latest_wall = d, time.time()
            except urllib.error.HTTPError as e:
                self.link = "waiting: the vision server has no frames yet (is the ESP32 streaming?)" if e.code == 404 else "vision server error %d" % e.code
            except Exception:
                self.link = "cannot reach the vision server at " + self.url + " (is uvicorn running?)"
            time.sleep(0.03)

    def tick(self, dt=DT):
        now = time.time()
        self.t = now - self.t0
        d = self.latest
        new = d is not None and d.get("timestamp") != self.seen
        stale = d is None or (now - self.latest_wall) > 0.25
        frame = None
        if new:
            self.seen = d["timestamp"]
            frame = frame_from_json(d)
        elif stale:
            frame = VisionFrame(False)
        if frame is not None:
            q_tgt, g_tgt, st = self.ctl.step(self.t, frame, self.q_cam, self.q)
            self.found = frame.found
            self.raw = self.arm.cam_to_world(self.q_cam, (frame.x_cm, frame.y_cm, frame.z_cm)) if frame.found else None
        else:
            q_tgt, g_tgt, st = self.ctl.q_target, self.ctl.grip_target, self.ctl.state
        self._move(q_tgt, g_tgt, dt)
        self.state = st
        self.obj = self.ctl.est


def make_server(engine, host="127.0.0.1", port=8010):
    class H(BaseHTTPRequestHandler):
        def _send(self, body, ctype="application/json", code=200):
            if isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self._send(open(os.path.join(HERE, "viewer", "index.html"), "rb").read(), "text/html; charset=utf-8")
            elif path == "/state":
                with engine.lock:
                    self._send(json.dumps(engine.snapshot()))
            elif path == "/config":
                with engine.lock:
                    self._send(json.dumps(engine.config()))
            elif path == "/reset":
                with engine.lock:
                    engine.reset()
                self._send('{"ok":true}')
            else:
                self._send('{"error":"not found"}', code=404)

        def log_message(self, *a):
            pass

    return ThreadingHTTPServer((host, port), H)


def run_loop(engine, speed=1.0, stop=None):
    nxt = time.perf_counter()
    while not (stop and stop.is_set()):
        with engine.lock:
            engine.tick(DT)
        nxt += DT / (speed if (speed and engine.mode == "demo") else 1.0)
        time.sleep(max(0.0, nxt - time.perf_counter()))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--demo", action="store_true")
    g.add_argument("--live", metavar="URL")
    ap.add_argument("--design", default="camera_forward", choices=list(DESIGNS))
    ap.add_argument("--port", type=int, default=8010)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--speed", type=float, default=1.0)
    a = ap.parse_args(argv)
    engine = Live(a.design, a.live) if a.live else Demo(a.design)
    srv = make_server(engine, a.host, a.port)
    threading.Thread(target=run_loop, args=(engine, a.speed), daemon=True).start()
    print("mode:", engine.mode, "| design:", a.design)
    if a.live:
        print("camera must be held fixed at the search pose:")
        f = engine.arm.fk(engine.q_cam)
        print("  position (x %.1f, y %.1f, z %.1f) cm, looking along (%.2f, %.2f, %.2f)" % (*f["cam_pos"], *f["fwd"]))
    print("open  http://%s:%d   in your browser (Ctrl+C here to stop)" % ("localhost" if a.host in ("127.0.0.1", "0.0.0.0") else a.host, a.port))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
