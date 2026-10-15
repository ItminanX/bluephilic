"""Run: python test_viewer.py - checks the 3D viewer's maths against the Python code, the server, and the demo loop."""
import json, os, shutil, subprocess, tempfile, threading, urllib.request
from dataclasses import asdict
import numpy as np
from arm_model import Arm
from designs import DESIGNS
from viewer_server import Demo, make_server, DT

HERE = os.path.dirname(os.path.abspath(__file__))


def test_javascript_maths_matches_python():
    node = shutil.which("node")
    if not node:
        print("SKIP (node is not installed): the browser maths was not compared with Python"); return
    P, _ = DESIGNS["camera_forward"]; a = Arm(P); rng = np.random.default_rng(5); L = lambda v: [float(x) for x in v]
    fkv, ikv = [], []
    for _ in range(150):
        q = [rng.uniform(-90, 90), rng.uniform(0, 180), rng.uniform(0, 180)]; f = a.fk(q)
        fkv.append(dict(q=q, m3=L(f["m3"]), s1=L(f["s1"]), tcp=L(f["tcp"]), cam=L(f["cam_pos"]), right=L(f["R_wc"][:, 0]),
                        up=L(-f["R_wc"][:, 1]), fwd=L(f["R_wc"][:, 2]), collides=bool(a.collides(q))))
    for _ in range(200):
        t = [rng.uniform(-35, 35), rng.uniform(-30, 40), rng.uniform(0, 40)]; qref = [rng.uniform(-60, 60), rng.uniform(20, 160), rng.uniform(20, 160)]
        best = a.ik(t, q_ref=qref)
        ikv.append(dict(t=t, qref=qref, sols=[L(s) for s in a.ik_all(t)], best=None if best is None else L(best)))
    d = asdict(P); d["chassis"] = list(d["chassis"])
    path = os.path.join(tempfile.gettempdir(), "arm_vectors.json"); json.dump(dict(P=d, fk=fkv, ik=ikv), open(path, "w"))
    r = subprocess.run([node, os.path.join(HERE, "viewer", "js_check.js"), path], capture_output=True, text=True)
    assert r.returncode == 0 and r.stdout.startswith("OK"), r.stdout + r.stderr


def test_server_endpoints():
    e = Demo("camera_forward"); srv = make_server(e, "127.0.0.1", 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    get = lambda p: urllib.request.urlopen("http://127.0.0.1:%d%s" % (srv.server_port, p)).read().decode()
    s = json.loads(get("/state")); c = json.loads(get("/config"))
    srv.shutdown()
    for k in ("state", "q", "grip", "servo", "tcp", "found", "attached"):
        assert k in s, k
    assert c["params"]["L2"] == 15.6 and len(c["basket"]) == 3 and "<canvas" in open(os.path.join(HERE, "viewer", "index.html")).read()


def test_demo_delivers_every_object():
    e = Demo("camera_forward")
    for _ in range(int(220 / DT)):
        e.tick(DT)
        if e.deliveries >= 4:
            break
    assert e.deliveries == 4, "demo delivered %d of 4 objects" % e.deliveries


if __name__ == "__main__":
    for n, fn in list(globals().items()):
        if n.startswith("test_"):
            fn(); print("PASS", n)
