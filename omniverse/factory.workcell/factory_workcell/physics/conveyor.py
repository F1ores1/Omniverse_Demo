"""3D conveyor: draws a `simcore.kinematics.track.Track` with kinematic loads locked to the belt's centre line.

No contact physics on the belt: loads cannot drift, rotate or stack. Friction and weight still shape the motion
through the track's profile (friction-limited acceleration, drive force shared over the mass on the belt).
Hand-over is explicit: can_accept / accept / ready_at_out / release.
"""
from __future__ import annotations

from pxr import Gf, Usd, UsdGeom

from simcore import usd as husd
from simcore.kinematics.track import Track

from . import loads


class Conveyor3D:
    def __init__(self, prim: Usd.Prim) -> None:
        self.prim, self.stage, self.id = prim, prim.GetStage(), prim.GetName()
        p = husd.read_params(prim)
        self.size = husd.size_of(prim)
        slots, gap = int(p.get("slots", 4)), float(p.get("gap", 0.05))
        if husd.item_of(prim).type == "buffer" and slots > 1:  # loads stop on the drawn slots, one pitch apart
            from simcore.usd.proxies import slot_pitch

            gap = slot_pitch(self.size, slots) - loads.LOAD_SIZE
        self.track = Track(length=self.size[0], load_len=loads.LOAD_SIZE, speed=float(p.get("speed", 0.2)),
                           accel=float(p.get("accel", 0.5)), friction=float(p.get("friction", 1.0)),
                           drive_force=float(p.get("drive_force", 300.0)), capacity=slots, gap=gap)
        self._xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.prims: dict[str, Usd.Prim] = {}

    # ---- hand-over -------------------------------------------------------------------
    def can_accept(self) -> bool:
        return self.track.can_accept()

    def accept(self, prim: Usd.Prim) -> None:
        loads.to_kinematic(prim)
        self.prims[prim.GetName()] = prim
        self.track.accept(prim.GetName(), loads.LOAD_MASS)
        self._place(prim, self.track.inlet_s)

    def ready_at_out(self) -> bool:
        return self.track.ready_at_out()

    def front(self) -> Usd.Prim | None:
        """The load at the end stop (the out node), if one is waiting there."""
        return self.prims[self.track.loads[0].id] if self.ready_at_out() else None

    def release(self) -> Usd.Prim:
        return self.prims.pop(self.track.release().id)

    def cleared(self) -> None:
        """The picked load has been lifted clear: the loads behind may move up."""
        self.track.cleared()

    # ---- motion ----------------------------------------------------------------------
    def step(self, dt: float) -> None:
        self.track.step(dt)
        for ld in self.track.loads:
            self._place(self.prims[ld.id], ld.s)

    def _place(self, prim: Usd.Prim, s: float) -> None:
        """Put a load on the centre line, `s` metres from the inlet end, resting on the belt."""
        x, _, z = self.size
        world = self._xf.Transform(Gf.Vec3d(-x / 2 + s, 0.0, z + loads.LOAD_SIZE / 2 + 0.002))
        loads.set_world_position(prim, world)

    def drop_point(self) -> Gf.Vec3d:
        """Where a transfer puts a load down: centre of a load resting at the inlet."""
        return self.inlet_world() + Gf.Vec3d(0, 0, loads.LOAD_SIZE / 2 + 0.002)

    def state_now(self, blocked_out: bool) -> str:
        return self.track.state(blocked_out)

    def inlet_world(self) -> Gf.Vec3d:
        x, _, z = self.size
        return self._xf.Transform(Gf.Vec3d(-x / 2 + self.track.inlet_s, 0.0, z))
