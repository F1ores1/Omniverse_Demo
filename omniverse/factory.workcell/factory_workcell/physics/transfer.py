"""3D transfer: moves one load between two items as a kinematic motion profile.

Raise from the pick point, move across to above the place point, lower; each leg is a trapezoid (accelerate,
cruise, decelerate). The load becomes WIP of the next item only once it is put down (`accept`).
Used for the Transfer item, as the default between two items with no transfer in between, and for a robot whose
moves could not be solved (robots normally use RobotTransfer3D).
"""
from __future__ import annotations

import carb
from pxr import Gf, Usd

from simcore import usd as husd
from simcore.catalog import DEFAULT_TRANSFER
from simcore.kinematics import trapezoid_pos, trapezoid_time

from . import loads


class Transfer3D:
    def __init__(self, src, dst, prim: Usd.Prim | None = None) -> None:
        self.src, self.dst, self.prim = src, dst, prim  # prim is None for a default (implicit) transfer
        self.id = prim.GetName() if prim else f"{getattr(src, 'id', '?')}->{getattr(dst, 'id', '?')}"
        p = {**DEFAULT_TRANSFER, **(husd.read_params(prim) if prim else {})}
        self.v, self.a, self.lift = float(p["v_max"]), float(p["a_max"]), float(p["lift"])
        self.grip = float(p.get("grip_time", 0.0))
        self.load: Usd.Prim | None = None
        self.t = 0.0
        self.legs: list[tuple[Gf.Vec3d, Gf.Vec3d, float]] = []
        self.done = False  # load is above/at the place point, waiting for room
        self.lifted = False
        self.fault: str | None = None  # set when a load is not on the pick node; the transfer stops (DOWN)

    def step(self, dt: float) -> list[tuple[str, str, str]]:
        """Advance one step. Returns hand-over events (load, item, 'leave' / 'enter')."""
        events = []
        if self.fault:
            return events
        if self.load is None:
            if self.src.ready_at_out() and self.dst.can_accept():
                node = husd.world_port(self.src.prim, "out_lowered")  # grip only at the source's pick node
                miss = loads.off_node(self.src.front(), node)
                if miss > loads.NODE_TOL:
                    self.fault = f"{self.id}: load on {self.src.id} is {miss * 100:.0f} cm off the pick node, stopped"
                    carb.log_error(self.fault)
                    return events
                self.load = self.src.release()
                loads.to_kinematic(self.load)
                self._plan(Gf.Vec3d(node) + Gf.Vec3d(0, 0, loads.LOAD_SIZE / 2 + 0.002), self.dst.drop_point())
                self.t, self.done, self.lifted = 0.0, False, False
                events.append((self.load.GetName(), self.src.id, "leave"))
            return events
        if not self.done:
            self.t += dt
            self._place_at(self.t)
            self.done = self.t >= self.total
            if not self.lifted and self.t >= self.grip + self.legs[0][2]:  # at the top of the lift: source is clear
                self.lifted = True
                self.src.cleared()
        if self.done and self.dst.can_accept():
            name = self.load.GetName()
            self.dst.accept(self.load)
            self.load = None
            events.append((name, self.dst.id, "enter"))
        return events

    def _plan(self, pick: Gf.Vec3d, place: Gf.Vec3d) -> None:
        up = Gf.Vec3d(0, 0, self.lift)
        pts = [pick, pick + up, place + up, place]
        self.legs = [(a, b, trapezoid_time((b - a).GetLength(), self.v, self.a)) for a, b in zip(pts, pts[1:])]
        self.total = self.grip + sum(t for _, _, t in self.legs) + self.grip

    def _place_at(self, t: float) -> None:
        t -= self.grip
        for a, b, leg in self.legs:
            if t <= leg:
                d = (b - a).GetLength()
                frac = trapezoid_pos(d, self.v, self.a, max(t, 0.0)) / d if d > 0 else 1.0
                loads.set_world_position(self.load, a + (b - a) * frac)
                return
            t -= leg
        loads.set_world_position(self.load, self.legs[-1][1])

    def state(self) -> str:
        if self.fault:
            return "DOWN"
        if self.load is None:
            return "STARVED"
        return "BLOCKED" if self.done else "PROCESSING"
