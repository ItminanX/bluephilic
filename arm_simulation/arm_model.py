"""
arm_model.py - geometry, forward kinematics (FK), analytical inverse kinematics (IK)
and camera model for the cardboard 3-joint + gripper arm.

CONVENTIONS (every one of these is an ASSUMPTION to verify on hardware - see SPEC.md)
  World frame : origin on the ground under the M1 axis. +X right, +Y forward, +Z up. cm / degrees.
  pan   (M1)  : 0 = straight ahead (+Y), +90 = right (+X), -90 = left. Range -90..+90.
  th2   (M2)  : world elevation of the M2 arm measured from horizontal-forward (+Y):
                0 = forward horizontal, 90 = straight up, 180 = backward horizontal.
  th3   (M3)  : servo angle, 90 = mean position. Arm angle relative to the M2 arm is
                rel = m3_dir*(th3-90) + m3_rel_at_mean. With defaults, M2=90/M3=90 puts the
                M3 arm 60 deg from vertical (30 deg above horizontal) as the builder confirmed.
  Everything lives in ONE vertical plane that rotates with M1 (the "30 deg inclined plane"
  is modelled as a fixed angular offset inside that plane, not a sideways tilt).
  TCP         : tool centre point = middle of the ideal grip zone, on the M3 arm line.
"""
from dataclasses import dataclass
import numpy as np


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class ArmParams:
    # ---- hand-measured (about +-0.5 cm) ----
    base_height: float = 10.0        # ground -> M1 axis
    m2_rise: float = 3.6             # M1 axis -> M2 pivot (vertical)
    L2: float = 15.6                 # M2 pivot -> M3 pivot (measured 15.4-15.8)
    L3: float = 11.3                 # M3 pivot -> S1 axis (measured 11.1-11.5)
    grip_reach: float = 8.5          # S1 axis -> centre of grip zone (measured 7-10)
    cam_along: float = 7.6           # camera offset along M3 arm from M3 pivot (7.4-7.7)
    cam_perp: float = 9.3            # camera offset perpendicular to M3 arm (9-9.6, ~10 from M3)
    cam_pitch_deg: float = 60.0      # optical axis angle to the M3 arm, tilted toward the arm. Bracket is 90 deg to the arm;
                                     # the OV2640 tilt on the devkit gives the 60 deg and cannot be changed (builder, confirmed)
    m3_rel_at_mean_deg: float = -60.0
    m3_dir: float = 1.0              # +1: increasing th3 swings the arm upward. UNVERIFIED
    pan_lim: tuple = (-90.0, 90.0)
    th2_lim: tuple = (0.0, 180.0)
    th3_lim: tuple = (0.0, 180.0)
    grip_open_deg: float = 30.0
    grip_closed_deg: float = 160.0
    min_joint_z: float = 1.0         # keep M3 pivot and S1 axis above the ground
    # rover body keep-out box (x0, x1, y0, y1, top_z) in cm. PLACEHOLDER - replace with the real chassis outline
    chassis: tuple = (-9.0, 9.0, -22.0, 6.0, 5.0)

    @property
    def z_m2(self):
        return self.base_height + self.m2_rise

    @property
    def tool_len(self):
        return self.L3 + self.grip_reach


@dataclass(frozen=True)
class Intrinsics:
    """From calibration.npz (RMS 3.92 px, 320x240)."""
    fx: float = 202.8
    fy: float = 210.9
    cx: float = 156.9
    cy: float = 112.3
    w: int = 320
    h: int = 240


class Arm:
    def __init__(self, p: ArmParams = ArmParams(), K: Intrinsics = Intrinsics()):
        self.p, self.K = p, K

    # ------------------------------------------------------------------ FK
    @staticmethod
    def _plane_vec(phi, alpha):
        """unit vector in the arm's vertical plane at world elevation alpha."""
        u = np.array([np.sin(phi), np.cos(phi), 0.0])
        return np.cos(alpha) * u + np.sin(alpha) * np.array([0, 0, 1.0])

    def fk(self, q):
        """q = (pan, th2, th3) degrees -> dict of joint points, TCP, camera pose."""
        p = self.p
        pan, th2, th3 = q
        phi = np.radians(pan)
        a2 = np.radians(th2)
        a3 = a2 + np.radians(p.m3_dir * (th3 - 90.0) + p.m3_rel_at_mean_deg)
        m1 = np.array([0, 0, p.base_height])
        m2 = np.array([0, 0, p.z_m2])
        m3 = m2 + p.L2 * self._plane_vec(phi, a2)
        arm_dir = self._plane_vec(phi, a3)
        s1 = m3 + p.L3 * arm_dir
        tcp = m3 + p.tool_len * arm_dir
        n_dir = self._plane_vec(phi, a3 + np.pi / 2)            # perpendicular, "above" the arm
        cam_pos = m3 + p.cam_along * arm_dir + p.cam_perp * n_dir
        pitch = np.radians(p.cam_pitch_deg)
        fwd = np.cos(pitch) * arm_dir - np.sin(pitch) * n_dir   # optical axis
        up_img = np.sin(pitch) * arm_dir + np.cos(pitch) * n_dir
        right = np.array([np.cos(phi), -np.sin(phi), 0.0])
        R = np.column_stack([right, -up_img, fwd])              # camera(X right,Y down,Z fwd) -> world
        return dict(m1=m1, m2=m2, m3=m3, s1=s1, tcp=tcp, cam_pos=cam_pos, R_wc=R,
                    arm_dir=arm_dir, alpha3_deg=np.degrees(a3))

    def cam_to_world(self, q, p_cam):
        f = self.fk(q)
        return f["cam_pos"] + f["R_wc"] @ np.asarray(p_cam, float)

    def world_to_cam(self, q, p_world):
        f = self.fk(q)
        return f["R_wc"].T @ (np.asarray(p_world, float) - f["cam_pos"])

    def project(self, q, p_world):
        """pixel (u,v) and camera-frame point of a world point; None if behind the camera."""
        pc = self.world_to_cam(q, p_world)
        if pc[2] <= 1e-6:
            return None, pc
        K = self.K
        return (K.fx * pc[0] / pc[2] + K.cx, K.fy * pc[1] / pc[2] + K.cy), pc

    # ------------------------------------------------------------------ IK
    def collides(self, q, margin=0.5):
        """True if any link / the camera passes through the chassis keep-out box."""
        x0, x1, y0, y1, zt = self.p.chassis
        f = self.fk(q)
        chain = [f["m2"], f["m3"], f["s1"], f["tcp"]]
        pts = [f["cam_pos"]]
        for a, b in zip(chain[:-1], chain[1:]):
            pts += [a + (b - a) * u for u in np.linspace(0, 1, 12)]
        return any(x0 - margin <= p[0] <= x1 + margin and y0 - margin <= p[1] <= y1 + margin and p[2] <= zt + margin for p in pts)

    def within_limits(self, q, tol=1e-6):
        p = self.p
        return (p.pan_lim[0] - tol <= q[0] <= p.pan_lim[1] + tol and
                p.th2_lim[0] - tol <= q[1] <= p.th2_lim[1] + tol and
                p.th3_lim[0] - tol <= q[2] <= p.th3_lim[1] + tol)

    def ik_all(self, target):
        """All joint solutions (pan, th2, th3) that put the TCP on `target` and respect limits
        and the ground-clearance rule. Closed form: pan from atan2, then a planar 2-link solve."""
        p = self.p
        x, y, z = target
        phi0 = np.degrees(np.arctan2(x, y))
        rad = np.hypot(x, y)
        if abs(phi0) <= 90.0:
            pan, r = phi0, rad
        else:                                   # target behind the base: use the opposite pan, negative radius
            pan, r = phi0 - np.sign(phi0) * 180.0, -rad
        dz = z - p.z_m2
        D = np.hypot(r, dz)
        L1, Lt = p.L2, p.tool_len
        if D > L1 + Lt - 1e-9 or D < abs(L1 - Lt) + 1e-9 or D < 1e-9:
            return []
        c = np.clip((D * D - L1 * L1 - Lt * Lt) / (2 * L1 * Lt), -1, 1)
        sols = []
        for rel in (np.arccos(c), -np.arccos(c)):
            a2 = np.arctan2(dz, r) - np.arctan2(Lt * np.sin(rel), L1 + Lt * np.cos(rel))
            th2 = wrap180(np.degrees(a2))
            th3 = (np.degrees(rel) - p.m3_rel_at_mean_deg) / p.m3_dir + 90.0
            q = np.array([pan, th2, th3])
            if not self.within_limits(q):
                continue
            f = self.fk(q)
            if f["m3"][2] < p.min_joint_z or f["s1"][2] < p.min_joint_z or self.collides(q):
                continue
            sols.append(q)
        return sols

    def ik(self, target, q_ref=None):
        sols = self.ik_all(target)
        if not sols:
            return None
        q_ref = np.array([0.0, 90.0, 90.0]) if q_ref is None else np.asarray(q_ref, float)
        return min(sols, key=lambda q: np.linalg.norm(q - q_ref))

    def ik_clamped(self, target, q_ref=None, steps=25):
        """IK with graceful fallback: shrink the horizontal distance until reachable."""
        for s in np.linspace(1.0, 0.3, steps):
            t = np.array([target[0] * s, target[1] * s, target[2]])
            q = self.ik(t, q_ref)
            if q is not None:
                return q, s
        return None, 0.0

    # ------------------------------------------------------------------ servo commands
    def to_servo_deg(self, q, grip_deg):
        """model angles -> the 0..180 numbers you write to each servo (M1 is shifted by +90)."""
        return dict(M1=q[0] + 90.0, M2=q[1], M3=q[2], S1=grip_deg)
