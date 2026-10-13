"""
Two designs compared throughout (camera tilt is fixed at 60 deg in both):
  as_measured    : camera bracket 7.6 cm along the M3 arm, best search pose found for it
  camera_forward : camera bracket moved to 13 cm along the arm (about 1.7 cm past the S1 axis), search pose M2 80 / M3 45
"""
from dataclasses import replace
from arm_model import ArmParams
from arm_controller import Config

DESIGNS = {
    "as_measured": (ArmParams(), Config(home_q=(0.0, 70.0, 60.0))),
    "camera_forward": (replace(ArmParams(), cam_along=13.0), Config(home_q=(0.0, 80.0, 45.0))),
}
