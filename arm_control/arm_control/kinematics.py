"""Pure-python (numpy only) kinematics for a serial arm described by a URDF.

No ROS imports here so it can be unit-tested without ROS.
Quaternions are (x, y, z, w) like ROS.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


# ------------------------------------------------------------------ math utils
def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_to_mat(axis, angle):
    a = np.asarray(axis, dtype=float)
    n = np.linalg.norm(a)
    if n < 1e-12:
        return np.eye(3)
    x, y, z = a / n
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def quat_to_mat(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R):
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def slerp(q0, q1, u):
    q0 = np.asarray(q0, float) / np.linalg.norm(q0)
    q1 = np.asarray(q1, float) / np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - u) * th) * q0 + math.sin(u * th) * q1) / math.sin(th)


def rot_log(R):
    """Rotation matrix -> rotation vector (axis * angle)."""
    c = (np.trace(R) - 1.0) / 2.0
    ang = math.acos(max(-1.0, min(1.0, c)))
    if ang < 1e-9:
        return np.zeros(3)
    if math.pi - ang < 1e-5:  # near 180 deg
        M = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(M), 0.0))
        k = int(np.argmax(axis))
        if axis[k] < 1e-9:
            return np.zeros(3)
        for i in range(3):
            if i != k:
                axis[i] = math.copysign(axis[i], M[k, i])
        axis /= np.linalg.norm(axis)
        return axis * ang
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2.0 * math.sin(ang)) * ang


def pose_to_mat(p, q):
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(q)
    T[:3, 3] = p
    return T


def mat_to_pose(T):
    return T[:3, 3].copy(), mat_to_quat(T[:3, :3])


# ------------------------------------------------------------------ chain
class Chain:
    """Serial chain base_link -> tip_link built from URDF xml text."""

    def __init__(self, urdf_xml, base_link, tip_link, tcp_offset=(0.0, 0.0, 0.0)):
        if isinstance(urdf_xml, str):
            urdf_xml = urdf_xml.encode("utf-8")
        root = ET.fromstring(urdf_xml)
        by_child = {}
        for j in root.findall("joint"):
            by_child[j.find("child").get("link")] = j

        path = []
        link = tip_link
        while link != base_link:
            if link not in by_child:
                raise ValueError(f"No path from '{base_link}' to '{tip_link}' in URDF (stuck at '{link}')")
            j = by_child[link]
            path.append(j)
            link = j.find("parent").get("link")
        path.reverse()

        self.joints = []
        for j in path:
            jtype = j.get("type")
            org = j.find("origin")
            xyz = [float(v) for v in (org.get("xyz", "0 0 0").split())] if org is not None else [0, 0, 0]
            rpy = [float(v) for v in (org.get("rpy", "0 0 0").split())] if org is not None else [0, 0, 0]
            T = np.eye(4)
            T[:3, :3] = rpy_to_mat(*rpy)
            T[:3, 3] = xyz
            ax = j.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.array([1.0, 0, 0])
            lim = j.find("limit")
            lower, upper, vel = -math.inf, math.inf, 1.0
            if lim is not None:
                if lim.get("lower") is not None:
                    lower = float(lim.get("lower"))
                if lim.get("upper") is not None:
                    upper = float(lim.get("upper"))
                if lim.get("velocity") is not None:
                    vel = float(lim.get("velocity"))
            self.joints.append(dict(name=j.get("name"), type=jtype, T=T, axis=axis,
                                    lower=lower, upper=upper, vel=vel))

        self.movable = [i for i, j in enumerate(self.joints) if j["type"] in ("revolute", "continuous")]
        self.joint_names = [self.joints[i]["name"] for i in self.movable]
        self.n = len(self.movable)
        self.lower = np.array([self.joints[i]["lower"] for i in self.movable])
        self.upper = np.array([self.joints[i]["upper"] for i in self.movable])
        self.vmax = np.array([self.joints[i]["vel"] for i in self.movable])
        self.T_tcp = np.eye(4)
        self.T_tcp[:3, 3] = tcp_offset

    # forward kinematics + geometric jacobian in the base frame
    def fk_jac(self, q, want_jac=True):
        T = np.eye(4)
        zs, ps = [], []
        k = 0
        for j in self.joints:
            T = T @ j["T"]
            if j["type"] in ("revolute", "continuous"):
                zs.append(T[:3, :3] @ j["axis"])
                ps.append(T[:3, 3].copy())
                R = np.eye(4)
                R[:3, :3] = axis_angle_to_mat(j["axis"], q[k])
                T = T @ R
                k += 1
        T = T @ self.T_tcp
        if not want_jac:
            return T, None
        p_tip = T[:3, 3]
        J = np.zeros((6, self.n))
        for i in range(self.n):
            J[:3, i] = np.cross(zs[i], p_tip - ps[i])
            J[3:, i] = zs[i]
        return T, J

    def fk(self, q):
        return self.fk_jac(np.asarray(q, float), want_jac=False)[0]

    def within_limits(self, q, tol=1e-6):
        q = np.asarray(q)
        return bool(np.all(q >= self.lower - tol) and np.all(q <= self.upper + tol))

    def ik(self, T_target, q_seed, max_iter=200, pos_tol=1e-4, rot_tol=1e-3,
           damping=0.05, null_gain=0.5, max_step=0.15, margin=1e-3):
        """Damped-least-squares IK with null-space pull toward the seed (keeps motion small)."""
        q_seed = np.asarray(q_seed, float)
        q = q_seed.copy()
        lo, hi = self.lower + margin, self.upper - margin
        I = np.eye(self.n)
        for _ in range(max_iter):
            T, J = self.fk_jac(q)
            e_p = T_target[:3, 3] - T[:3, 3]
            e_r = rot_log(T_target[:3, :3] @ T[:3, :3].T)
            if np.linalg.norm(e_p) < pos_tol and np.linalg.norm(e_r) < rot_tol:
                return q, True
            e = np.concatenate([e_p, e_r])
            Jp = J.T @ np.linalg.inv(J @ J.T + (damping ** 2) * np.eye(6))
            # exact (undamped) null-space projector so the posture term cannot disturb the task
            N = I - np.linalg.pinv(J, rcond=1e-3) @ J
            dq = Jp @ e + N @ (null_gain * (q_seed - q))
            m = np.max(np.abs(dq))
            if m > max_step:
                dq *= max_step / m
            q = np.clip(q + dq, lo, hi)
        return q, False
