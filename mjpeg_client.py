"""
mjpeg_client.py
Reads an MJPEG-over-HTTP stream (multipart/x-mixed-replace, the format
ESP32-CAM streaming sketches serve) and yields decoded BGR frames.

Deliberately not using cv2.VideoCapture(url) for this — OpenCV's
FFmpeg-based HTTP backend is unreliable with ESP32's stream format
across platforms. This does the multipart parsing by hand instead,
which is a well-understood, dependable pattern.
"""
from typing import Generator, Iterable

import cv2
import numpy as np
import requests

JPEG_START = b"\xff\xd8"
JPEG_END = b"\xff\xd9"


def iter_jpegs_from_bytes(chunks: Iterable[bytes]) -> Generator[np.ndarray, None, None]:
    """Pure parsing logic, kept separate from the network call so it can
    be unit tested without a real server: feed it any iterable of byte
    chunks and it yields decoded BGR frames."""
    buf = b""
    for chunk in chunks:
        buf += chunk
        while True:
            start = buf.find(JPEG_START)
            if start == -1:
                buf = b""  # no frame start yet, drop garbage before it
                break
            end = buf.find(JPEG_END, start + 2)
            if end == -1:
                break  # frame not fully arrived yet, wait for more chunks
            jpg_bytes = buf[start:end + 2]
            buf = buf[end + 2:]
            img = cv2.imdecode(np.frombuffer(jpg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                yield img


def mjpeg_frames(url: str, timeout: float = 10.0) -> Generator[np.ndarray, None, None]:
    """Connects to an MJPEG stream URL (e.g. http://<esp32-ip>/stream)
    and yields decoded BGR frames indefinitely."""
    with requests.get(url, stream=True, timeout=timeout) as r:
        r.raise_for_status()
        yield from iter_jpegs_from_bytes(r.iter_content(chunk_size=2048))
