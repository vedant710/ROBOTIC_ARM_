"""Trajectory generation (no ROS imports): joint-space and Cartesian-line moves.

Every trajectory is eased (cosine ramp) so it starts/stops smoothly, and its duration is
stretched if needed so no joint exceeds  speed_scale * URDF velocity limit.
"""
import math

import numpy as np

from .kinematics import mat_to_quat, slerp, pose_to_mat, rot_log


class PlanError(Exception):
    pass


def _finalize(q_path, T_total, vmax, speed_scale):
    """q_path: (M+1, n) including the start. Returns (pos (M,n), vel (M,n), t (M,))."""
    q_path = np.asarray(q_path, float)
    M = len(q_path) - 1
    u = np.arange(1, M + 1) / M
    base_t = np.arccos(np.clip(1.0 - 2.0 * u, -1.0, 1.0)) / math.pi   # in [0,1]
    T_total = max(T_total, 1e-3)
    t_prev = np.concatenate([[0.0], base_t[:-1]])
    dt_unit = base_t - t_prev                                          # per-step time for T=1
    dq = np.abs(np.diff(q_path, axis=0))
    ratio = float(np.max(dq / (dt_unit[:, None] * T_total) / (vmax * speed_scale)[None, :]))
    if ratio > 1.0:
        T_total *= ratio
    t = base_t * T_total

    full_t = np.concatenate([[0.0], t])
    vel = np.zeros((M, q_path.shape[1]))
    for i in range(1, M):  # central difference around path point i
        vel[i - 1] = (q_path[i + 1] - q_path[i - 1]) / (full_t[i + 1] - full_t[i - 1])
    return q_path[1:], vel, t


def plan_joint(chain, q0, q_goal, speed_scale=0.3, min_duration=0.5):
    q0 = np.asarray(q0, float)
    q_goal = np.asarray(q_goal, float)
    if not chain.within_limits(q_goal):
        bad = [chain.joint_names[i] for i in range(chain.n)
               if not (chain.lower[i] - 1e-6 <= q_goal[i] <= chain.upper[i] + 1e-6)]
        raise PlanError(f"joint target outside limits: {bad}")
    dmax = float(np.max(np.abs(q_goal - q0)))
    if dmax < 1e-4:
        raise PlanError("already at joint target")
    M = int(min(200, max(10, math.ceil(dmax / 0.05))))
    path = np.array([q0 + (q_goal - q0) * (i / M) for i in range(M + 1)])
    T0 = max(min_duration, dmax / (np.max(chain.vmax) * speed_scale))
    return _finalize(path, T0, chain.vmax, speed_scale)


def plan_cartesian(chain, q0, T_goal, lin_speed=0.10, ang_speed=0.5, lin_step=0.01, ang_step=0.05,
                   speed_scale=0.3, min_duration=0.5, max_joint_jump=0.5):
    """Straight line (and slerp) in Cartesian space from FK(q0) to T_goal; IK every step."""
    q0 = np.asarray(q0, float)
    T0 = chain.fk(q0)
    p0, p1 = T0[:3, 3], T_goal[:3, 3]
    dist = float(np.linalg.norm(p1 - p0))
    ang = float(np.linalg.norm(rot_log(T_goal[:3, :3] @ T0[:3, :3].T)))
    if dist < 1e-4 and ang < 1e-3:
        raise PlanError("already at TCP target")
    M = int(min(400, max(2, math.ceil(max(dist / lin_step, ang / ang_step)))))
    qa, qb = mat_to_quat(T0[:3, :3]), mat_to_quat(T_goal[:3, :3])

    path = [q0]
    for i in range(1, M + 1):
        u = i / M
        Ti = pose_to_mat(p0 + u * (p1 - p0), slerp(qa, qb, u))
        qi, ok = chain.ik(Ti, path[-1])
        if not ok:
            raise PlanError(f"IK failed at step {i}/{M} (target probably unreachable or near a singularity)")
        jump = float(np.max(np.abs(qi - path[-1])))
        if jump > max_joint_jump:
            raise PlanError(f"IK solution jumped {jump:.2f} rad at step {i}/{M} "
                            "(configuration flip), refusing to move")
        path.append(qi)
    T_total = max(min_duration, dist / lin_speed, ang / ang_speed)
    return _finalize(np.array(path), T_total, chain.vmax, speed_scale)
