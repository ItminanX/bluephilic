# Plan -> Simulate -> Document -> Build: the complete guide

Everything below was run, not just written. Reproduce it all with:

```bash
pip install numpy scipy matplotlib
python test_model.py        # 8 unit tests (geometry, IK, camera transform, 10 s rule)
python run_sim.py           # closed-loop scenarios, both designs  -> results/*.png, results/summary.json
python design_sweep.py      # camera pitch x search pose trade-off -> results/design_sweep.json
python sensitivity.py       # model-error analysis                 -> results/sensitivity.json
python position_sweep.py    # 15 positions x 3 camera tilts, both brackets -> results/position_sweep.json
python calibrate_m3.py 0:-60 90:30 180:120   # protractor readings -> m3_dir and offset
```

## 0. Corrections to what I gave you earlier (read this first)
The first version of the simulation files (and its DOCUMENTATION.md) was written without being run. When I ran it:
- the "neutral" pose put the gripper *behind* the base, contradicting your geometry;
- the numbers in the old documentation (e.g. "validated to +-1 cm", "workspace +-45 cm") were not measured - they were wrong;
- the controller fed camera-frame coordinates straight into IK (they must first be transformed through the arm's own pose, because the camera rides on the arm);
- the 10-second dwell used wall-clock time inside a 200-frame loop, so it could never trigger in a test;
- the GPIO pin table was invented. Ignore it; pin assignment is still to be decided (camera pins on the S3 board constrain it).
- I earlier wrote MG996R stall current as ~3.5 A. The commonly quoted figure is ~2.5 A at 6 V (SG90 ~0.7 A) - check your datasheet/seller.
All of that is replaced by the files in this folder.

## 1. What is in the folder
| File | Job |
|---|---|
| `SPEC.md` | the compressed single source of truth: dimensions, conventions, open items |
| `arm_model.py` | geometry, FK, closed-form IK, camera pose + projection, servo-angle mapping |
| `sim_camera.py` | synthetic ESP32/server output (pixel noise, size-based depth, 25 % dropouts) |
| `arm_controller.py` | state machine IDLE -> TRACK -> APPROACH -> DESCEND -> CLOSE -> TRANSFER -> RELEASE -> HOME |
| `run_sim.py` | closed loop: object + camera on the arm + controller + slew-limited servos; plots |
| `design_sweep.py` | camera pitch x search pose trade-off |
| `sensitivity.py` | what happens when your measurements are wrong |
| `designs.py` | `as_measured` (bracket 7.6 cm) vs `camera_forward` (bracket 13 cm) |
| `position_sweep.py` | robustness to camera-tilt error across floor positions |
| `calibrate_m3.py` | M3 direction / zero from three protractor readings |
| `live_bridge.py` | feeds your real server `/latest` output (or a recorded log) through the controller in shadow mode |
| `make_sample_log.py`, `test_bridge.py` | synthetic replay log and the bridge tests |
| `HOW_IT_ALL_WORKS.md` | plain-language explanation: what runs, every plot, the FK/IK math, the 3D viewer |
| `viewer/index.html`, `viewer_server.py` | the 3D viewer (manual, demo, live) and the server that feeds it |
| `test_viewer.py`, `viewer/js_check.js` | checks that the viewer's math matches Python, plus server and demo tests |
| `RUN_AND_COMMIT.md` | how to run everything on your PC and commit it to GitHub |
| `test_model.py` | unit tests |

Signal flow (this is also the real firmware flow):
`frame (camera XYZ) -> FK of the arm at capture time -> world XYZ -> gate + EMA -> state machine -> target TCP -> IK -> slew limit -> 4 servo angles`

## 2. What the simulation found (all numbers measured by the scripts in this folder)

**Camera.** With the tilt fixed at 60 deg, where the camera bracket sits along the M3 arm decides how robust vision is.
`position_sweep.py` places the object at 15 floor positions (x -12..12, y 16..28 cm) and lets the *real* tilt differ from the assumed 60 deg:

| Real camera tilt | Bracket at 7.6 cm (now) | Bracket at 13 cm (recommended) |
|---|---|---|
| 55 deg | 4 / 15 grabs | 15 / 15 |
| 60 deg | 15 / 15 | 15 / 15 |
| 65 deg | 7 / 15 | 13 / 15 |

So the present bracket only works if the tilt really is 60.0 deg; a typical +-5 deg measuring error breaks it. Moving the bracket forward to ~13 cm
(about 1.7 cm past the S1 axis, camera just above the gripper) tolerates it. `design_sweep.py` gives the same trend as a coverage score
(worst case over tilt 55-65 deg): 11 % at 7.6 cm, 46 % at 10, 83 % at 12, 91 % at 13, 100 % at 14. Check that the gripper jaws do not enter the image.
Search (home) pose used: M2 70 / M3 60 for the current bracket, M2 80 / M3 45 for 13 cm.

**Reach.** TCP max forward reach 35.4 cm. A 5 cm object on the floor is graspable out to ~31 cm ahead; the nearest graspable floor point is ~7 cm from the base
(set by the placeholder chassis box). Park the rover so the object is 15-28 cm ahead of the M1 axis.

**Basket.** The rear deck at low height is outside the arm's range (see SPEC section 4). A drop point at the side-front of the base,
(-14, 12, 20) cm, is reachable with 36 deg margin on every joint and a collision-free swing; that is the simulated default.
If you insist on the rear, the object must be released from z >= ~34 cm with M2 at 170-178 deg (against its stop), which a few degrees of zero error will break.

**Closed loop (30 Hz, noisy camera, 25 % dropouts).** Both designs complete the still-object and moving-then-still scenarios; 20/20 noise-only repeats succeed;
grasp never starts before ~10 s of stillness; nothing is grabbed while the object keeps moving; no chassis collision; servo rate stays at the 90 deg/s limit;
an object beyond reach is not chased (the arm holds the search pose).

**Model error (the honest part).** Random errors typical of hand-measured cardboard (lengths +-0.3 cm, grip reach +-0.8 cm, camera tilt +-3 deg,
M3 offset +-2 deg, servo zero +-1.5 deg), 40 runs: only 31/40 (current bracket) and 32/40 (forward bracket) put the TCP within the assumed 2 cm; median ~1.1-1.2 cm,
p90 ~2.5 cm, worst ~3.4 cm. Biggest single contributors: camera tilt +5 deg -> 1.7-2.0 cm; grip reach +1.5 cm -> 1.5 cm; M3 offset +5 deg -> 1.0 cm; M2 zero +2 deg -> 0.9 cm.
The controller is not the weak link; calibration and the open-loop final approach are. Mitigations in order: (1) measure the camera tilt and bracket offsets;
(2) run `calibrate_m3.py` and set servo zeros/directions with a jig; (3) re-measure grip reach with the real jaw; (4) later, close the loop on the last ~10 cm with the camera.

## 3. The process (phases with exit criteria)
**Phase 0 - freeze the spec.** Fill the parameter register (section 5) and settle the 7 open items in `SPEC.md`.
Exit: every number has a measurement method and an uncertainty.

**Phase 1 - kinematics.** `python test_model.py` green, plus a printed 1:1 paper template of the arm at 3 poses checked against FK.
Exit: FK predicts the real TCP position within 1 cm at 3 poses.

**Phase 2 - vision-to-world.** (first move the camera bracket if you adopt the 13 cm position) Place a marker at known floor positions with the arm at the search pose; compare
`cam_to_world(...)` to the tape-measure position. Exit: error < 1.5 cm, otherwise fix camera extrinsics (pitch/offset), not the controller.

**Phase 3 - controller in simulation.** Done here. Re-run `run_sim.py`, `sensitivity.py` after every parameter change; keep results in git.

**Phase 4 - servo bring-up (no arm load).** Power rail first (section 7). Find each servo's real pulse range, direction, and zero. Write them into `ArmParams`
(`m3_dir`, offsets) and re-run the tests.

**Phase 5 - arm only, vision off.** Drive IK targets from a keyboard/serial: reach 5 floor points, record TCP error.
Exit: median error < 1 cm.

**Phase 6 - closed loop on the bench.** Object on a table, rover stationary: tracking -> dwell -> grab -> basket. Log everything; compare with the sim traces.

**Phase 7 - rover coupling.** Use `need_closer` and the world position estimate to tell the rover how far to go; stop it when the object is ~15-28 cm ahead of the M1 axis.

## 4. Simulation tooling: what to use
- **Python (this folder)**: right for this arm. IK is closed form, the loop is fast, tests are plain asserts, and it ports almost line-for-line to the ESP32 (C++).
- **MATLAB + Robotics System Toolbox / Peter Corke's Robotics Toolbox**: worth it if your report needs DH tables, Jacobians or Simulink block diagrams. This arm is planar after the pan, so a DH table adds notation, not accuracy; derive it from `arm_model.py` if a judge wants it.
- **PyBullet or MuJoCo (later)**: only when you need collisions with the chassis/basket or motor torque limits. Add after Phase 5.
- **Physical honesty**: none of these model backlash, servo deadband, cardboard flex or battery sag. Those are measured, not simulated.

## 5. Documentation system (so every claim is traceable)
```
docs/
  00_overview.md            goal, block diagram, status
  01_parameter_register.md  table: name | value | how measured | uncertainty | date | used in
  02_decision_log.md        one entry per decision: context, options, choice, evidence (a results/*.json or plot)
  03_kinematics.md          conventions + derivation (copy the SPEC section, add the IK derivation)
  04_experiments/           one file per run: date, git commit, config, plot, outcome, next step
  05_power_and_wiring.md    load table, regulator choice, schematic
  06_open_issues.md
results/                    generated, never hand-edited
```
Rules that keep it rigorous: (1) a number in the docs must come from a script output or a dated measurement; (2) never delete a failed experiment;
(3) tag git at each phase exit; (4) every figure is regenerated by a command listed under it.

## 6. Still open (each one moves the results)
1. Is your camera tilt really 60.0 deg? Measure it (a protractor against the lens plane / a ruler-and-distance method); +-5 deg matters.
2. M3: run `calibrate_m3.py` with three readings - this settles direction and zero (and the "peak at 90" wording).
3. Basket: confirm a side-front drop point such as (-14, 12, 20) cm is physically possible on your car; otherwise give me the region you can build.
4. Chassis outline relative to the M1 axis (replaces the placeholder box).
5. Gripper jaw opening and tolerated position error.
6. Real object size/height.
7. Rough masses for the shoulder torque check.

## 7. Power (re-checked)
Worst case, every servo stalled: 3 x 2.5 A + 0.7 A ~ 8.2 A at 6 V. Real motion with a light cardboard arm is typically 0.5-1.5 A per large servo, and the slew limit
(90 deg/s) in the controller lowers peaks. Size the rail for ~10 A peak.
- Preferred: 2S LiPo (>= 2200 mAh, 20C) -> 6.0 V UBEC/buck rated >= 10 A peak (MG996R is rated to 7.2 V; do not exceed it).
- Put 1000-2200 uF across the servo rail, tie grounds with the ESP32, and power the ESP32-S3 camera from its own regulator so servo brownouts do not reset the stream.
- Do not run the servos from the ESP32 5 V pin.
- Shoulder torque check (M2): torque = sum(mass x distance). Rough illustration with *guessed* masses (M3 servo 55 g at 15.6 cm, wrist assembly ~100 g at ~22 cm, a 50 g object at 35 cm) gives ~5 kg-cm
  against about 11 kg-cm stall at 6 V - fine, but weigh the real parts and redo it; holding above ~50 % of stall overheats servos.

## 8. Using AI well in this project
- Give it the SPEC and the results JSON, not a summary from memory; ask it to change one thing at a time and to re-run the tests.
- Ask for the *derivation and a test*, not just code. A model that cannot be falsified by a test is a guess.
- Make it list assumptions and unknowns (this guide's section 6 came from that).
- Do not trust it for datasheet numbers, sign conventions or "it works" statements; trust the script output and the tape measure.
- Keep a prompt log in `docs/04_experiments/` next to each result so a reader can reproduce the reasoning.
