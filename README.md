# Rover Vision — color object tracking, from scratch

Pipeline: ESP32-S3 + OV2640 captures frames -> ESP32 also reads a VL53L0X
depth sensor -> both get POSTed to a FastAPI backend (deployed on Render)
-> backend detects the tracked object (currently tuned for blue), tracks
it, and back-projects pixel position + depth into a real-world (X, Y, Z)
in centimeters, camera-frame.

Arm/car IK is intentionally out of scope for this stage — this gets you a
working, testable 3D position feed first.

Tested locally: your own Python install for the CV side, Arduino IDE for
firmware. The Colab notebook is kept only as a fallback for when local
Python isn't available.

## Files

- `esp32_cam_client/esp32_mjpeg_stream_server.ino` — ESP32 firmware
  serving a continuous live video stream for local testing. **Flash
  this first.**
- `mjpeg_client.py` — reads that stream reliably (hand-rolled multipart
  parser — `cv2.VideoCapture` is unreliable against ESP32-style streams).
- `vision_pipeline.py` — the detector (hue-based color detection),
  tracker, and pinhole 3D projection. Run this directly against your
  ESP32's live stream with real trackbars.
- `camera_calibrate.py` — one-time OV2640 checkerboard calibration,
  also against the live stream.
- `server.py` — FastAPI backend, deploy this to Render once you're
  ready to go live with depth fusion.
- `esp32_cam_client/esp32_cam_client.ino` — ESP32 firmware for the final
  live system: POSTs frames + depth to your Render backend.
- `esp32_cam_client/esp32_snapshot_server.ino` — optional single-photo
  sketch, useful for grabbing stills for the Colab fallback notebook.
- `requirements-local.txt` — deps for running the pipeline on your own
  machine. `requirements.txt` — deps for the Render backend.
- `rover_vision_tuning.ipynb` — Colab-based tuning notebook, fallback
  only.

## Step 0 — Install local dependencies

```bash
pip install -r requirements-local.txt
```

This installs `opencv-contrib-python`, not plain `opencv-python` —
the object tracker (`cv2.TrackerCSRT_create`) lives in the contrib
package, not the base one. If you ever see
`AttributeError: module 'cv2' has no attribute 'TrackerCSRT_create'`,
this is why — run:
```bash
pip uninstall opencv-python -y
pip install opencv-contrib-python
```

## Step 1 — Flash the ESP32 and get the stream running

Open `esp32_cam_client/esp32_mjpeg_stream_server.ino` in Arduino IDE,
fill in `WIFI_SSID` and `WIFI_PASS`, flash it. This uses the ESP-IDF
`esp_http_server` (the same approach Espressif's own official camera
example uses) rather than the Arduino `WebServer` library, which turned
out to be unreliable for hand-rolled MJPEG streaming.

Open Serial Monitor at 115200 baud — once connected, it prints:
```
Connected! Stream URL for your Python scripts:
http://192.168.x.x/stream
```

Sanity-check it in a regular browser first — you should see a live
video feed.

**Important limitation:** this server can only handle one viewer of
`/stream` at a time. If you leave a browser tab open on the stream and
then also try to run a Python script against it, the Python request
will hang and eventually time out — it's queued behind the browser
connection, not broken. Always close any open stream tab before running
`vision_pipeline.py` or `camera_calibrate.py`.

## Step 2 — Tune the detector live

```bash
python vision_pipeline.py --source http://<esp32-ip>/stream --tune
```

Three windows open: `tracking` (main view with detection overlay),
`mask` (what the detector currently sees as a match), and `tune`
(sliders). Hold your object in frame and adjust:
- **H min / H max** — the hue range for your object's color (0-179 in
  OpenCV). Pure blue is roughly 100-130; narrow or shift this until the
  mask shows only your object.
- **S min** — raise if white/gray background leaks into the mask.
- **V min** — raise if dark shadows leak into the mask.

`min_area` and `min_aspect_ratio` aren't exposed as sliders — edit them
directly in `DetectorConfig` if you need to reject small noise blobs or
require a specific shape (e.g. raise `min_aspect_ratio` above `1.0` if
you're tracking an elongated object, not a blob).

Once the green box locks cleanly onto the object and follows it around
the frame, hardcode the slider values into `DetectorConfig` in
`vision_pipeline.py` **and** `server.py` — the trackbars are for tuning,
not production. Press `q` to quit.

## Step 3 — Calibrate the OV2640 (do this before trusting X/Y in cm)

Print a 9x6-internal-corner checkerboard pattern (search "opencv
checkerboard pattern 9x6" for a printable PDF), tape it flat.

```bash
python camera_calibrate.py --source http://<esp32-ip>/stream --capture
# press 'c' ~15-20 times at different angles/distances/corners of frame
# press 'q' when done
python camera_calibrate.py --calibrate
```

This writes `calibration.npz`. Both `vision_pipeline.py` and `server.py`
auto-load it from the working directory if present. Without it, the
pipeline runs on a rough FOV guess — fine for testing detection, not
accurate enough for real-world centimeters.

## Step 4 — Run the backend locally (sanity check before deploying)

```bash
pip install -r requirements.txt
uvicorn server:app --reload --port 8000
```

Grab one frame (a saved photo of your object) and test the ingest
endpoint:

```bash
curl -X POST http://localhost:8000/ingest \
     -F "depth_cm=30.0" \
     -F "frame=@/path/to/a/saved/photo/with/the/object.jpg"
```

You should get back JSON with `x_cm`, `y_cm`, `z_cm`, `angle_deg`. Open
`http://localhost:8000/debug_feed` in a browser to watch the annotated
feed live once frames are coming in continuously.

## Step 5 — Deploy the backend to Render

1. Push this folder to a GitHub repo.
2. On Render: New -> Web Service -> connect the repo.
3. Build command: `pip install -r requirements.txt`
4. Start command: `uvicorn server:app --host 0.0.0.0 --port $PORT`
5. Once deployed, note the URL (e.g. `https://your-app.onrender.com`).

Commit `calibration.npz` alongside the code so the deployed backend has
real intrinsics, not the FOV guess. Note that `requirements.txt` uses
`opencv-contrib-python-headless` for the same tracking-module reason as
the local setup — don't swap it back to plain `opencv-python-headless`.

## Step 6 — Flash the final ESP32 firmware and go live

1. Open `esp32_cam_client/esp32_cam_client.ino` in Arduino IDE.
2. Fill in `WIFI_SSID`, `WIFI_PASS`, and `BACKEND_URL` (your Render URL +
   `/ingest`).
3. Wire the VL53L0X over I2C (see wiring comment in the sketch), adjust
   the `Wire.begin(SDA, SCL)` pins to match whatever's free on your board.
4. Flash it, open Serial Monitor at 115200 baud, confirm WiFi connects
   and you see `POST -> 200` messages.
5. Watch `https://your-app.onrender.com/debug_feed` in a browser — you
   should see your live rig's view with the object boxed and its
   (X,Y,Z) printed on-frame.

## Known rough edges to expect

- **Single-viewer stream limit** — see Step 1. This only affects the
  testing sketch (`esp32_mjpeg_stream_server.ino`); the production
  sketch (`esp32_cam_client.ino`) POSTs to your backend instead and
  doesn't have this limitation.
- **Camera pin map** — all three `.ino` sketches use a common
  AI-Thinker-style pin layout as a starting point. Verify it against
  your specific N16R8 dev board's schematic/silkscreen before flashing;
  if the camera fails to init, this is the first thing to check.
- **opencv-contrib vs opencv-python** — see Step 0. This bit us once
  already; both `requirements-local.txt` and `requirements.txt` are
  already set correctly, just don't swap them.
- **Render free-tier cold starts** — if the service spins down from
  inactivity, the first frame after a gap will be slow.
- **WiFi latency/jitter** — the `delay(100)` in the ESP32 client loop
  targets ~10 fps; raise it if you see POST timeouts or a lagging feed.
- **VL53L0X range** — rated for a few meters but accuracy degrades past
  ~1-1.2m and on non-perpendicular surfaces; keep the object reasonably
  close and roughly facing the sensor for now.
- **Color threshold drift** — re-run Step 2's `--tune` any time you
  change rooms or lighting conditions materially, or switch to tracking
  a different color object.
