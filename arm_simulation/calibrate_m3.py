"""
calibrate_m3.py - pin down the M3 direction and zero with a protractor (2 minutes, no code changes needed).

PROCEDURE
  1. Command M2 = 90 deg (M2 arm straight up). Check it with a square/plumb line first.
  2. Command M3 = 0, then 90, then 180 deg (add 45 and 135 if you like).
  3. For each, measure the angle of the M3 arm ABOVE the forward horizontal, in degrees:
        0 = pointing straight ahead, 90 = straight up, 180 = pointing straight back, negative = pointing below horizontal.
  4. python calibrate_m3.py 0:-60 90:30 180:120        (format  th3:measured_elevation)

If the current model is right you should measure about  -60, 30, 120.
The script prints the m3_dir and m3_rel_at_mean_deg to put in ArmParams, and warns if the slope is not +-1.
"""
import sys
import numpy as np

def main(args):
    pts = [tuple(float(v) for v in a.split(":")) for a in args]
    if len(pts) < 2:
        print(__doc__); return
    th3 = np.array([p[0] for p in pts]); alpha = np.array([p[1] for p in pts])
    m, c = np.polyfit(th3, alpha, 1)
    direction = 1.0 if m > 0 else -1.0
    # model (M2 = 90):  alpha3 = 90 + dir*(th3-90) + rel_mean
    rel_mean = float(np.polyval([m, c], 90.0) - 90.0)
    resid = alpha - np.polyval([m, c], th3)
    print(f"fit: elevation = {m:+.3f} * th3 {c:+.1f}   (max residual {np.abs(resid).max():.1f} deg)")
    print(f"  m3_dir             = {direction:+.0f}")
    print(f"  m3_rel_at_mean_deg = {rel_mean:+.1f}   (current model: -60)")
    if abs(abs(m) - 1) > 0.1:
        print("  WARNING: slope is not +-1 -> the horn is slipping, the servo range is not 180 deg, or a reading is wrong.")
    if np.abs(resid).max() > 3:
        print("  WARNING: readings are not on a straight line - re-measure.")

if __name__ == "__main__":
    main(sys.argv[1:])
