"""
debug_checkerboard.py
Diagnostic: tests your saved calib_images/ photos against several common
checkerboard sizes at once, so we know immediately whether the problem
is a size mismatch (fixable in seconds) or something else (focus,
lighting, flatness).

Usage:
    python debug_checkerboard.py
"""
import glob
import cv2

# (columns, rows) of INTERNAL corners, not squares. Trying the common
# printable sizes people actually use.
SIZES_TO_TRY = [
    (9, 6), (6, 9),
    (8, 6), (6, 8),
    (7, 6), (6, 7),
    (7, 5), (5, 7),
    (8, 5), (5, 8),
    (9, 7), (7, 9),
    (6, 5), (5, 6),
    (4, 3), (3, 4),
]

FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

images = sorted(glob.glob("calib_images/*.jpg"))
if not images:
    raise SystemExit("No images found in calib_images/ — run the --capture step first.")

sample = images[: min(15, len(images))]
print(f"Testing {len(sample)} of your {len(images)} saved photos against {len(SIZES_TO_TRY)} board sizes...\n")

results = []
for size in SIZES_TO_TRY:
    found_count = 0
    for fname in sample:
        img = cv2.imread(fname)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, _ = cv2.findChessboardCorners(gray, size, flags=FLAGS)
        if found:
            found_count += 1
    results.append((size, found_count))
    marker = "  <-- hit!" if found_count > 0 else ""
    print(f"  size={size!s:>8}: found in {found_count}/{len(sample)}{marker}")

best = max(results, key=lambda r: r[1])

print()
if best[1] > 0:
    print(f"BEST MATCH: CHECKERBOARD = {best[0]}")
    print(f"Update CHECKERBOARD = {best[0]} at the top of camera_calibrate.py, then re-run --calibrate.")
else:
    print("No size matched at all across every photo.")
    print("This points to focus, lighting, or flatness — not a size mismatch. Specifically:")
    print("  1. Check the OV2640 module's lens — most ship badly out of focus for")
    print("     close range. Gently twist the small lens barrel while looking at a")
    print("     sharp-edged object up close, until edges look crisp, not blurry.")
    print("  2. Make sure the checkerboard is taped completely flat — even a slight")
    print("     curl/wave in the paper breaks corner detection.")
    print("  3. Even, diffuse lighting — avoid glare/hotspots on the paper (a direct")
    print("     flashlight can actually cause this by blowing out part of the board).")
    print("  4. Fill a good portion of the frame with the board — too small/far away")
    print("     means too few pixels per square to resolve corners reliably.")
