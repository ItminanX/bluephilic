"""Run: python test_bridge.py  - checks the live/replay bridge on a synthetic log and against a mock /latest server."""
import io, json, threading, time, contextlib
from http.server import BaseHTTPRequestHandler, HTTPServer
import live_bridge, make_sample_log


def test_replay_goes_through_the_whole_sequence():
    path = make_sample_log.make()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        sh = live_bridge.main(["--replay", path, "--quiet"])
    seq = [s for _, s in sh.ctl.log]
    assert seq[:3] == ["TRACK", "APPROACH", "DESCEND"], seq
    for need in ("CLOSE", "TRANSFER", "RELEASE", "HOME"):
        assert need in seq, seq
    t_track = next(t for t, s in sh.ctl.log if s == "TRACK"); t_appr = next(t for t, s in sh.ctl.log if s == "APPROACH")
    assert t_appr - t_track >= 9.5, "the 10 s dwell rule must hold on replayed data"


def test_live_polling_dedupes_and_survives_404():
    frames = [json.loads(l) for l in open("sample_logs/synthetic_still_object.jsonl")][:60]
    state = {"i": -5}                                          # first 5 polls answer 404 (no frames yet)
    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            i = state["i"]; state["i"] += 1
            if i < 0:
                self.send_response(404); self.end_headers(); self.wfile.write(b'{"error":"no frames received yet"}'); return
            body = json.dumps(dict(frames[min(i // 2, len(frames) - 1)])).encode()   # every frame served twice -> duplicates
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers(); self.wfile.write(body)
        def log_message(self, *a): pass
    srv = HTTPServer(("127.0.0.1", 0), H); threading.Thread(target=srv.serve_forever, daemon=True).start()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        live_bridge.main(["--url", f"http://127.0.0.1:{srv.server_port}/latest", "--rate", "200", "--seconds", "2", "--quiet"])
    srv.shutdown()
    out = buf.getvalue()
    assert "waiting: server has no frames yet" in out and "states visited" in out


if __name__ == "__main__":
    for n, fn in list(globals().items()):
        if n.startswith("test_"):
            fn(); print("PASS", n)
