# SPEC - single source of truth for the simulation (updated after the camera / M3 / basket answers)

Values are HAND-MEASURED (~+-0.5 cm) on a cardboard prototype. **VERIFY** marks assumptions the results depend on.

## 1. Goal (arm only; the rover is a separate WiFi-controlled system)
Blue object seen by the arm-mounted ESP32-S3 camera -> arm follows it -> object still >= 10 s -> arm grips it and drops it in a basket on the car -> returns to the search pose.

## 2. Hardware
| Part | Role | Notes |
|---|---|---|
| M1 MG995 (or MG996R) | pan, -90..+90, 90 = straight ahead | M1 axis 10 cm above ground (5 cm chassis + 5.35 cm servo) |
| M2 MG996R | shoulder, 0..180 | pivot 3.6 cm above M1 axis |
| M3 MG996R | elbow, 0..180 | pivot 15.4-15.8 cm from M2 (model 15.6) |
| S1 SG90 | gripper, closed 150-175, open < ~30 | S1 axis 11.1-11.5 cm from M3 (model 11.3) |
| Grip zone | centre 7-10 cm beyond S1 (model 8.5) | TCP = M3 pivot + 19.8 cm along the M3 arm |
| Camera | fixed to the M3 arm, rotates with it | bracket is 90 deg to the arm; the OV2640 tilt on the devkit makes the optical axis **60 deg to the arm, toward the gripper side**; cannot be changed |

Camera bracket position along the M3 arm: **7.6 cm now** (model default). The simulation recommends moving it to **~13 cm** (see GUIDE section 2).
Intrinsics (calibration.npz, 320x240, RMS 3.92 px): fx 202.8, fy 210.9, cx 156.9, cy 112.3.
Vision output (server.py): camera frame X right, Y down, Z forward, cm; Z from the known object size (5 cm); 30-47 fps; found:false on ~25-35 % of frames.

## 3. Conventions used by the code
World: origin on the ground under the M1 axis; +X right, +Y forward, +Z up. pan 0 = ahead, + = right.
- th2 = world elevation of the M2 arm from forward-horizontal (90 = straight up).
- th3: arm angle relative to the M2 arm = m3_dir*(th3-90) - 60. With m3_dir = +1 (**bigger th3 raises the arm**, builder confirmed), M2 = 90 / M3 = 90 puts the M3 arm 60 deg from vertical.
  Expected protractor readings with M2 = 90: th3 = 0 / 90 / 180 -> -60 / 30 / 120 deg above forward-horizontal. Run `calibrate_m3.py` to confirm (**VERIFY**: the phrase "raises up to 90, then lowers" is not fully reconciled with the 60-deg-at-mean statement).
- The "30 deg inclined plane" is modelled as an angular offset inside the arm's vertical plane, not a sideways tilt (**VERIFY**).
- IK is closed form (pan = atan2, planar 2-link law of cosines, two elbow branches, then limits / ground / chassis filter).

## 4. Basket / drop point (decided from the sim)
Rear deck at low height is NOT reachable: it needs the other elbow branch (M3 would have to fold forward by ~140 deg, its range stops at +30) or M2 at 170-178 deg against its stop (only above z ~ 34 cm).
Default drop point in the arm frame: **x = -14, y = 12, z = 20 cm** (left side-front of the base, mirror to +14 for the right), IK joints about M1 -49 / M2 87 / M3 36, 36 deg from every limit, swing path clear of the chassis box.

## 5. Placeholders still to measure
Chassis keep-out box (x -9..9, y -22..6, top 5 cm); grasp tolerance (2.0 cm sphere); object (5 cm cube on the floor); basket position; masses.
