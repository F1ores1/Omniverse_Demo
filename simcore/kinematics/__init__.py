"""Closed-form motion and clearance checks. Pure Python: used by the DES and by the 3D playback.

Fidelity choice: axis moves follow a trapezoidal velocity profile (accelerate, cruise, decelerate).
Collision checks use axis-aligned bounding boxes, which is enough to stop "robot through the CNC".
"""
from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

Vec3 = tuple[float, float, float]


def trapezoid_time(distance: float, v_max: float, a_max: float) -> float:
    """Time to move `distance` from rest to rest. Short moves never reach v_max (triangular profile)."""
    d = abs(distance)
    if d == 0:
        return 0.0
    if d >= v_max * v_max / a_max:
        return d / v_max + v_max / a_max
    return 2.0 * sqrt(d / a_max)


def trapezoid_pos(distance: float, v_max: float, a_max: float, t: float) -> float:
    """Distance covered at time `t` on the same profile. Clamped to [0, distance]."""
    d, total = abs(distance), trapezoid_time(distance, v_max, a_max)
    if t <= 0 or d == 0:
        return 0.0
    if t >= total:
        return d
    v_peak = min(v_max, sqrt(d * a_max))
    t_acc = v_peak / a_max
    if t < t_acc:
        return 0.5 * a_max * t * t
    if t > total - t_acc:
        return d - 0.5 * a_max * (total - t) ** 2
    return 0.5 * a_max * t_acc * t_acc + v_peak * (t - t_acc)


def pick_place_time(travel: float, lift: float, v_max: float, a_max: float, grip_time: float = 0.0) -> float:
    """One full transfer cycle: down, grip, up, across, down, release, up, and the empty return."""
    return (4 * trapezoid_time(lift, v_max, a_max)
            + 2 * trapezoid_time(travel, v_max, a_max)
            + 2 * grip_time)


@dataclass(frozen=True)
class Box:
    """Axis-aligned bounding box in world space (metres)."""

    lo: Vec3
    hi: Vec3

    def grown(self, margin: float) -> "Box":
        return Box(tuple(x - margin for x in self.lo), tuple(x + margin for x in self.hi))


def boxes_overlap(a: Box, b: Box, clearance: float = 0.0) -> bool:
    """True when the boxes intersect or come closer than `clearance`. Touching faces do not count."""
    return all(a.lo[i] < b.hi[i] + clearance and b.lo[i] < a.hi[i] + clearance for i in range(3))


def segment_hits_box(p0: Vec3, p1: Vec3, box: Box) -> bool:
    """True when the straight tool path p0 -> p1 passes through the box (slab test)."""
    t0, t1 = 0.0, 1.0
    for i in range(3):
        d = p1[i] - p0[i]
        if abs(d) < 1e-12:
            if not box.lo[i] < p0[i] < box.hi[i]:
                return False
            continue
        a, b = (box.lo[i] - p0[i]) / d, (box.hi[i] - p0[i]) / d
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
        if t0 >= t1:
            return False
    return True
