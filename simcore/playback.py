"""Playback: turn the DES event log into "where is each load, and what state is each unit" at time t.

Pure Python. The 3D view is a client of this: it never computes timing itself, so what you see is
exactly what the DES decided.
"""
from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from math import dist

Vec3 = tuple[float, float, float]


@dataclass
class Stay:
    """One load on one unit: arrives at t0, is finished at t_work, leaves at t1 (later if blocked)."""

    station: str
    unit: int
    t0: float
    t_work: float
    t1: float


def part_stays(events: list[tuple], end: float) -> dict[int, list[Stay]]:
    """Events are (t, station, unit, state, part id). Returns each load's visits in order."""
    stays: dict[int, list[Stay]] = {}
    current: dict[tuple[str, int], tuple[int, Stay]] = {}
    for t, station, unit, state, part in events:
        key = (station, unit)
        held = current.get(key)
        if part is not None and (held is None or held[0] != part):
            stay = Stay(station, unit, t, end, end)
            stays.setdefault(part, []).append(stay)
            current[key] = (part, stay)
        elif held is not None:
            stay = held[1]
            if state == "BLOCKED":
                stay.t_work = t
            elif part is None:  # the unit released the load
                stay.t1 = t
                stay.t_work = min(stay.t_work, t)
                del current[key]
    return stays


class StateTrack:
    """State of every unit over time, for the floor status rectangles."""

    PRIORITY = ("DOWN", "PROCESSING", "SETUP", "BLOCKED", "STARVED", "IDLE")

    def __init__(self, events: list[tuple]) -> None:
        self._tracks: dict[str, dict[int, tuple[list[float], list[str]]]] = {}
        for t, station, unit, state, _ in events:
            times, states = self._tracks.setdefault(station, {}).setdefault(unit, ([], []))
            times.append(t)
            states.append(state)

    def station_state(self, station: str, t: float) -> str:
        """One state per station: the most informative one among its units."""
        found = set()
        for times, states in self._tracks.get(station, {}).values():
            i = bisect_right(times, t) - 1
            found.add(states[i] if i >= 0 else "IDLE")
        return next((s for s in self.PRIORITY if s in found), "IDLE")


class LoadIndex:
    """Which loads are in the cell at time t."""

    def __init__(self, stays: dict[int, list[Stay]]) -> None:
        self.stays = stays
        self._order = sorted(stays, key=lambda p: stays[p][0].t0)
        self._starts = [stays[p][0].t0 for p in self._order]
        self.max_wip = 400  # look-back window; far above any cell this demo builds

    def active(self, t: float) -> list[int]:
        hi = bisect_right(self._starts, t)
        return [p for p in self._order[max(0, hi - self.max_wip):hi] if self.stays[p][-1].t1 > t]


def pick_place_point(pick: Vec3, place: Vec3, lift: float, s: float) -> Vec3:
    """Point on the path up -> across -> down at fraction s (0..1) of its length."""
    a, b = (pick[0], pick[1], pick[2] + lift), (place[0], place[1], place[2] + lift)
    pts = [pick, a, b, place]
    lengths = [dist(p, q) for p, q in zip(pts, pts[1:])]
    target = max(0.0, min(1.0, s)) * sum(lengths)
    for p, q, length in zip(pts, pts[1:], lengths):
        if target <= length or q is place:
            k = target / length if length else 1.0
            return tuple(p[i] + (q[i] - p[i]) * min(1.0, k) for i in range(3))
        target -= length
    return place


def lerp(a: Vec3, b: Vec3, k: float) -> Vec3:
    k = max(0.0, min(1.0, k))
    return tuple(a[i] + (b[i] - a[i]) * k for i in range(3))
