"""3D station: a machine (CNC, press, inspection, black box) or buffer that holds loads as WIP.

`count` units work at once, each for one sampled cycle time; up to `buffer` more wait in front. Loads sit at the
work position (queued ones lined up behind it). Same hand-over as every item: can_accept / accept / ready_at_out /
release. Times come from the same parameters the DES uses.
"""
from __future__ import annotations

import random

from pxr import Gf, Usd, UsdGeom

from simcore import usd as husd
from simcore.catalog import parse_dist

from . import loads


def sample(dist, rng: random.Random) -> float:
    if not isinstance(dist, dict):
        return float(dist)
    kind = dist["dist"]
    if kind == "exp":
        return rng.expovariate(1.0 / dist["mean"])
    if kind == "normal":
        return max(0.0, rng.gauss(dist["mean"], dist["sd"]))
    if kind == "triangular":
        return rng.triangular(dist["low"], dist["high"], dist["mode"])
    return rng.uniform(dist["low"], dist["high"])


class Station3D:
    """`count` identical units side by side, each with its own work spot; up to `buffer` more loads wait in a queue
    in front of the item's centre. A load stays on its unit from the moment it is put down until it is picked."""

    def __init__(self, prim: Usd.Prim, rng: random.Random) -> None:
        self.prim, self.id, self.rng = prim, prim.GetName(), rng
        p = husd.read_params(prim)
        self.kind = husd.item_of(prim).type
        self.cycle = parse_dist(p.get("cycle_time", 0.0)) if self.kind != "buffer" else 0.0
        offsets = husd.unit_offsets(prim)
        self.units = len(offsets)
        buffer = int(prim.GetAttribute("factory:buffer").Get() or 0)
        extra = int(p.get("slots", 0)) if self.kind == "buffer" else (10_000 if buffer < 0 else buffer)
        self.capacity = self.units + extra
        work = Gf.Vec3d(husd.world_port(prim, "in_lowered")) + Gf.Vec3d(0, 0, loads.LOAD_SIZE / 2 + 0.002)
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        side = Gf.Vec3d(xf.TransformDir(Gf.Vec3d(0, 1, 0))).GetNormalized()
        self.spots = [work + side * off for off in offsets]      # one work spot per unit
        self.queue_start = work + Gf.Vec3d(xf.TransformDir(Gf.Vec3d(-1, 0, 0))).GetNormalized() * (
            husd.size_of(prim)[0] / 2 + loads.LOAD_SIZE)         # waiting loads line up in front of the item
        self.back = Gf.Vec3d(xf.TransformDir(Gf.Vec3d(-1, 0, 0))).GetNormalized()
        self.on_unit: list[dict | None] = [None] * self.units    # {"prim", "done"} per unit
        self.queue: list[Usd.Prim] = []
        self.departing: set[int] = set()  # units whose picked load is still lifting off
        self.t = 0.0

    # ---- hand-over -------------------------------------------------------------------
    def _free_unit(self) -> int | None:
        return next((k for k, u in enumerate(self.on_unit) if u is None and k not in self.departing), None)

    def can_accept(self) -> bool:
        busy = sum(u is not None for u in self.on_unit) + len(self.departing) + len(self.queue)
        return busy < self.capacity

    def drop_point(self) -> Gf.Vec3d:
        k = self._free_unit()
        return self.spots[k] if k is not None else self._queue_spot(len(self.queue))

    def accept(self, prim: Usd.Prim) -> None:
        loads.to_kinematic(prim)
        k = self._free_unit()
        if k is not None:
            self.on_unit[k] = {"prim": prim, "done": None}
            loads.set_world_position(prim, self.spots[k])
        else:
            self.queue.append(prim)
            self._layout_queue()

    def _finished(self) -> int | None:
        """The unit whose load finished first, if any."""
        done = [(u["done"], k) for k, u in enumerate(self.on_unit) if u and u["done"] is not None and u["done"] <= self.t]
        return min(done)[1] if done else None

    def ready_at_out(self) -> bool:
        return self._finished() is not None

    def front(self) -> Usd.Prim | None:
        k = self._finished()
        return None if k is None else self.on_unit[k]["prim"]

    def release(self) -> Usd.Prim:
        k = self._finished()
        prim = self.on_unit[k]["prim"]
        self.on_unit[k] = None
        self.departing.add(k)  # the next load moves onto this unit only once it is cleared
        self._releasing = k
        return prim

    def cleared(self) -> None:
        self.departing.discard(getattr(self, "_releasing", None))

    def step(self, t: float) -> None:
        self.t = t
        k = self._free_unit()
        while k is not None and self.queue:  # a waiting load moves onto a free unit
            prim = self.queue.pop(0)
            self.on_unit[k] = {"prim": prim, "done": None}
            loads.set_world_position(prim, self.spots[k])
            self._layout_queue()
            k = self._free_unit()
        for u in self.on_unit:
            if u is not None and u["done"] is None:
                u["done"] = t + sample(self.cycle, self.rng)

    def _queue_spot(self, i: int) -> Gf.Vec3d:
        return self.queue_start + self.back * (i * (loads.LOAD_SIZE + 0.05))

    def _layout_queue(self) -> None:
        for i, prim in enumerate(self.queue):
            loads.set_world_position(prim, self._queue_spot(i))

    def unit_states(self) -> list[str]:
        """Per unit: STARVED (no load), PROCESSING (working), BLOCKED (finished, waiting to be picked)."""
        return ["STARVED" if u is None else "BLOCKED" if u["done"] is not None and u["done"] <= self.t else "PROCESSING"
                for u in self.on_unit]

    def state(self, blocked_out: bool) -> str:
        if not any(self.on_unit) and not self.queue:
            return "STARVED"
        return "BLOCKED" if blocked_out else "PROCESSING"
