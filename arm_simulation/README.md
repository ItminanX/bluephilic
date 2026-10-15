# arm_simulation

Kinematics + closed-loop simulation for the blue-object-tracking arm (ESP32-S3 camera, cardboard arm, MG995/MG996R + SG90).

    pip install -r requirements-sim.txt
    python test_model.py && python test_bridge.py && python test_viewer.py     # 8 + 2 + 3 checks, a few seconds
    python run_sim.py                                  # closed-loop scenarios -> results/
    python live_bridge.py --url http://localhost:8000/latest   # your real vision output, shadow mode

    python viewer_server.py --demo                     # 3D viewer: open http://localhost:8010 (or double-click viewer\index.html)

Start with `HOW_IT_ALL_WORKS.md` (plain-language explanation of everything, the math, and the 3D viewer), then `RUN_AND_COMMIT.md` (run / test / git), `GUIDE.md` (the plan-simulate-document process and findings) and `SPEC.md` (dimensions and conventions).
