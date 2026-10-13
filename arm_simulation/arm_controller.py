"""
arm_controller.py - tracking / dwell / grab state machine for the arm (arm only; rover is separate).

Pipeline per vision frame:
  camera-frame point --(FK of the arm AT CAPTURE TIME)--> world point --> filter/gate -->
  state machine --> target TCP point --> analytical IK --> slew-limited servo targets.
Time is passed in (t, dt) so the same code runs in the simulator and on real time.
"""
from dataclasses import dataclass, field
from enum import Enum
import numpy as np


class State(Enum):
    IDLE = 0
    TRACK = 1
    APPROACH = 2
    DESCEND = 3
    CLOSE = 4
    TRANSFER = 5
    RELEASE = 6
    HOME = 7


@dataclass
class Config:
    home_q: tuple = (0.0, 90.0, 90.0)
    hover_h: float = 8.0               # TCP hover height above the tracked object
    approach_h: float = 6.0            # pre-grasp height above the object
    obj_center_z: float = 2.5          # grasp height = object centre (5 cm object on the floor)
    alpha: float = 0.25                # EMA gain on world position
    gate_cm: float = 12.0              # reject single-frame jumps larger than this
    acquire_frames: int = 3
    lost_timeout: float = 1.5          # s without a valid detection -> back to IDLE
    dwell_s: float = 10.0              # the 10-second rule
    still_tol: float = 2.0             # cm, drift allowed during the dwell
    max_rate: float = 90.0             # deg/s servo slew limit
    settle_tol: float = 1.5            # deg
    close_time: float = 0.8
    release_time: float = 0.6
    cooldown: float = 3.0
    basket: tuple = (-14.0, 12.0, 20.0) # drop point over a basket at the car side-front (rear deck is NOT reachable at low height - see GUIDE). PLACEHOLDER until built


class Controller:
    def __init__(self, arm, cfg=Config()):
        self.arm, self.cfg = arm, cfg
        self.state = State.IDLE
        self.q_target = np.array(cfg.home_q, float)
        self.grip_target = arm.p.grip_open_deg
        self.reset_track()
        self.state_t0 = 0.0
        self.cool_until = 0.0
        self.grasp_pt = None
        self.log = []

    def reset_track(self):
        self.est = None
        self.last_valid_t = -1e9
        self.valid_run = 0
        self.anchor = None
        self.anchor_t = 0.0
        self.unreachable = False
        self.need_closer = False        # hint for the rover: object is beyond the arm's grasp envelope

    # ------------------------------------------------------------ helpers
    def _go(self, s, t):
        self.state = s
        self.state_t0 = t
        self.log.append((t, s.name))

    def _settled(self, q_meas):
        return np.max(np.abs(np.asarray(q_meas) - self.q_target)) < self.cfg.settle_tol

    def _aim(self, point, q_meas):
        q, _ = self.arm.ik_clamped(point, q_ref=q_meas)
        if q is not None:
            self.q_target = q
        return q is not None

    def _ingest(self, frame, q_capture, t):
        """vision -> world estimate (gated EMA). returns True if a valid sample was used."""
        if not frame.found:
            self.valid_run = 0
            return False
        p_w = self.arm.cam_to_world(q_capture, (frame.x_cm, frame.y_cm, frame.z_cm))
        if self.est is not None and np.linalg.norm(p_w - self.est) > self.cfg.gate_cm and \
                (t - self.last_valid_t) < self.cfg.lost_timeout:
            return False                                    # outlier: ignore
        self.est = p_w if self.est is None else self.est + self.cfg.alpha * (p_w - self.est)
        self.last_valid_t = t
        self.valid_run += 1
        if self.anchor is None or np.linalg.norm(self.est - self.anchor) > self.cfg.still_tol:
            self.anchor, self.anchor_t = self.est.copy(), t
        return True

    # ------------------------------------------------------------ main step
    def step(self, t, frame, q_capture, q_meas):
        """Returns (q_target[3], grip_target_deg, state)."""
        c = self.cfg
        used = self._ingest(frame, q_capture, t) if self.state in (State.IDLE, State.TRACK) else False

        if self.state == State.IDLE:
            self.q_target = np.array(c.home_q, float)
            self.grip_target = self.arm.p.grip_open_deg
            if t >= self.cool_until and self.valid_run >= c.acquire_frames:
                self._go(State.TRACK, t)

        elif self.state == State.TRACK:
            if t - self.last_valid_t > c.lost_timeout:
                self.reset_track()
                self._go(State.IDLE, t)
            elif self.est is not None:
                tgt = np.array([self.est[0], self.est[1], c.obj_center_z])
                reachable = self.arm.ik(tgt, q_meas) is not None
                self.need_closer = not reachable
                if reachable:
                    self._aim(self.est + [0, 0, c.hover_h], q_meas)
                else:                                   # do not chase: keep the object in view, ask the rover to close in
                    self.q_target = np.array(c.home_q, float)
                if reachable and t - self.anchor_t >= c.dwell_s:
                    self.grasp_pt = tgt
                    self._go(State.APPROACH, t)

        elif self.state == State.APPROACH:
            self._aim(self.grasp_pt + [0, 0, c.approach_h], q_meas)
            if self._settled(q_meas):
                self._go(State.DESCEND, t)

        elif self.state == State.DESCEND:
            self._aim(self.grasp_pt, q_meas)
            if self._settled(q_meas):
                self._go(State.CLOSE, t)

        elif self.state == State.CLOSE:
            self.grip_target = self.arm.p.grip_closed_deg
            if t - self.state_t0 >= c.close_time:
                self._go(State.TRANSFER, t)

        elif self.state == State.TRANSFER:
            self._aim(np.array(c.basket), q_meas)
            if self._settled(q_meas):
                self._go(State.RELEASE, t)

        elif self.state == State.RELEASE:
            self.grip_target = self.arm.p.grip_open_deg
            if t - self.state_t0 >= c.release_time:
                self._go(State.HOME, t)

        elif self.state == State.HOME:
            self.q_target = np.array(c.home_q, float)
            if self._settled(q_meas):
                self.reset_track()
                self.cool_until = t + c.cooldown
                self._go(State.IDLE, t)

        return self.q_target.copy(), self.grip_target, self.state
