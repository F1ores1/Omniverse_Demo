"""Straight-line conveyor track: loads move along one axis with a motion profile limited by simple physics.

Pure Python (no Kit), so it is testable and its times can feed DES tables. The 3D conveyor draws it.

Physics that changes the timing, and nothing more:
- Belt speed ramps up at `accel`, but never faster than the drive force can push the belt plus everything on it
  (`drive_force / (belt_mass + load mass)`): a heavily loaded belt starts slower.
- A load can only speed up or slow down as fast as friction allows (`friction * g`); if the belt changes speed
  faster than that, the load slips and lags.
- Loads stop at the end stop or behind the load in front, keeping `gap` between them. They never touch.
Ownership is explicit: a load is on the track from `accept()` until `release()`.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

G = 9.81


@dataclass
class TrackLoad:
    id: str
    mass: float
    s: float = 0.0   # m from the inlet end to the load's centre
    v: float = 0.0   # m/s


class Track:
    def __init__(self, length: float, load_len: float, speed: float, accel: float, friction: float,
                 drive_force: float, capacity: int, belt_mass: float = 20.0, gap: float = 0.05) -> None:
        self.length, self.load_len, self.gap = length, load_len, gap
        self.speed, self.accel, self.friction = speed, accel, friction
        self.drive_force, self.belt_mass, self.capacity = drive_force, belt_mass, capacity
        self.vb = 0.0                      # belt speed now
        self.departing = False             # a picked load is still lifting off the end: it keeps blocking
        self.loads: list[TrackLoad] = []   # front (nearest the outlet) first

    @property
    def inlet_s(self) -> float:
        return self.load_len / 2

    @property
    def end_s(self) -> float:
        return self.length - self.load_len / 2

    def can_accept(self) -> bool:
        """Room under the WIP limit, and the inlet is clear of the last load (plus the gap)."""
        if len(self.loads) >= self.capacity:
            return False
        return not self.loads or self.loads[-1].s - self.load_len - self.gap >= self.inlet_s

    def accept(self, load_id: str, mass: float) -> TrackLoad:
        load = TrackLoad(load_id, mass, s=self.inlet_s, v=0.0)
        self.loads.append(load)
        return load

    def ready_at_out(self) -> bool:
        """The front load is stopped at the end stop, waiting to be picked or handed on."""
        return (not self.departing and bool(self.loads) and self.loads[0].s >= self.end_s - 1e-4
                and self.loads[0].v < 1e-4)

    def release(self) -> TrackLoad:
        """Hand the front load over. Its end slot stays blocked until `cleared()` (the picker has lifted it away)."""
        self.departing = True
        return self.loads.pop(0)

    def cleared(self) -> None:
        self.departing = False

    def step(self, dt: float) -> None:
        mass = self.belt_mass + sum(ld.mass for ld in self.loads)
        a_belt = min(self.accel, self.drive_force / mass)
        self.vb = min(self.speed, self.vb + a_belt * dt)
        a_load = self.friction * G   # most friction can accelerate or brake a load
        limit = self.end_s           # how far the front load may go
        if self.departing:           # the picked load still occupies the end
            limit = self.end_s - self.load_len - self.gap
        for ld in self.loads:
            dist = max(0.0, limit - ld.s)
            v_stop = sqrt(2 * a_load * dist)   # fastest speed that can still stop at the limit
            ld.v = max(0.0, min(self.vb, ld.v + a_load * dt, v_stop))
            ld.s = min(ld.s + ld.v * dt, limit)
            if ld.s >= limit - 1e-6:
                ld.s, ld.v = limit, 0.0
            limit = ld.s - self.load_len - self.gap

    # ---- for the floor status rectangle ----
    def state(self, blocked_out: bool) -> str:
        if not self.loads:
            return "STARVED"
        if blocked_out and len(self.loads) >= self.capacity:
            return "BLOCKED"
        return "PROCESSING"
