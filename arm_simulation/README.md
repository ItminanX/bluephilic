# arm_simulation

Kinematics + closed-loop simulation for the blue-object-tracking arm (ESP32-S3 camera, cardboard arm, MG995/MG996R + SG90).

    pip install -r requirements-sim.txt
    python test_model.py && python test_bridge.py     # 8 + 2 checks, a few seconds
    python run_sim.py                                  # closed-loop scenarios -> results/
    python live_bridge.py --url http://localhost:8000/latest   # your real vision output, shadow mode

Start with `RUN_AND_COMMIT.md` (run / test / git), `GUIDE.md` (the plan-simulate-document process and findings) and `SPEC.md` (dimensions and conventions).
