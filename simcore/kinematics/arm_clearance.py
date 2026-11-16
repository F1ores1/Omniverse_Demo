"""UR10 clearance against box obstacles: the arm's link tubes as spheres, distance to axis-aligned boxes.

Pure numpy. The kinematics object (cuMotion `Kinematics`) is passed in, so the same check runs inside Kit (robot.py)
and headless (scripts/robot_layout_study.py). Hard-coded to the UR10 and to box obstacles (our proxy machines).

Fidelity: each tube is a row of spheres of radius ARM_R along its centre line (UR10 tubes are ~0.06-0.075 m radius).
The carried load is one sphere under the tool. Arm-to-arm contact is not checked here.
"""
from __future__ import annotations

import numpy as np

ARM_R = 0.08       # m, sphere radius standing in for a UR10 link tube
LOAD_R = 0.10      # m, sphere standing in for the carried load
LOAD_BELOW = 0.06  # m, load centre below the tool flange (tool target = node + 0.16, load centre = node + 0.10)

# Tube centre lines at the zero pose, in the arm's base frame, from the UR10 URDF frame origins.
# (frame that carries the tube, start, end, number of spheres)
TUBES = [
    ("shoulder_link", (0.0, 0.0, 0.127), (0.0, 0.221, 0.127), 3),
    ("upper_arm_link", (0.0, 0.221, 0.127), (0.612, 0.221, 0.127), 9),
    ("forearm_link", (0.612, 0.221, 0.127), (0.612, 0.049, 0.127), 3),   # elbow housing, on the elbow axis
    ("forearm_link", (0.612, 0.049, 0.127), (1.184, 0.049, 0.127), 9),
    ("wrist_1_link", (1.184, 0.049, 0.127), (1.184, 0.164, 0.127), 2),
    ("wrist_2_link", (1.184, 0.164, 0.127), (1.184, 0.164, 0.012), 2),
    ("wrist_3_link", (1.184, 0.164, 0.012), (1.184, 0.256, 0.012), 2),
]
TOOL = "tool0"


def _pose(kin, q, frame):
    p = kin.pose(q, frame)
    return np.asarray(p.rotation.matrix(), dtype=np.float64), np.asarray(p.translation, dtype=np.float64).ravel()


class ArmBody:
    """Sphere centres of the arm for any joint vector, in the arm's base frame."""

    def __init__(self, kin) -> None:
        self.kin = kin
        q0 = np.zeros(kin.num_cspace_coords())
        self.local: list[tuple[str, np.ndarray]] = []
        for frame, a, b, n in TUBES:
            r, t = _pose(kin, q0, frame)
            pts = np.linspace(a, b, n)
            self.local.append((frame, (pts - t) @ r))  # R^T (p - t), row-wise
        self.load_local = np.array([0.0, 0.0, LOAD_BELOW])  # tool z points along the approach (down when gripping)

    def points(self, q: np.ndarray) -> np.ndarray:
        out, cache = [], {}
        for frame, local in self.local:
            if frame not in cache:
                cache[frame] = _pose(self.kin, q, frame)
            r, t = cache[frame]
            out.append(local @ r.T + t)
        return np.vstack(out)

    def load_point(self, q: np.ndarray) -> np.ndarray:
        r, t = _pose(self.kin, q, TOOL)
        return r @ self.load_local + t


def box_distance(points: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """(N points, B boxes) distance from each point to each box; 0 inside."""
    p = points[:, None, :]
    return np.linalg.norm(np.maximum(np.maximum(lo[None] - p, 0.0), p - hi[None]), axis=-1)


def clearance(points: np.ndarray, lo: np.ndarray, hi: np.ndarray, radius: float = ARM_R) -> tuple[float, int]:
    """Smallest gap between the spheres and the boxes (negative = overlap), and the box it is to."""
    if len(lo) == 0:
        return float("inf"), -1
    d = box_distance(points, lo, hi) - radius
    k = int(np.argmin(d))
    return float(d.flat[k]), k % d.shape[1]
