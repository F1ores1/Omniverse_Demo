"""2D view of a DES run: floor status rectangles and loads, stepped event by event.

Nothing here decides timing. The clock jumps from one DES event time to the next (`speed` events per real second),
not through simulation time, so idle stretches are skipped and every frame is a state the DES actually reached.
Loads are drawn where the DES has them at that event (simcore.playback).
"""
from __future__ import annotations

from pxr import Gf, Usd, UsdGeom

from simcore import usd as husd
from simcore.playback import LoadIndex, StateTrack, lerp, part_stays, pick_place_point
from simcore.usd.proxies import slot_positions

LOADS = "/Plant/Loads"
LOAD = 0.16      # m, edge of a load
QUEUE_PITCH = 0.2


class Player:
    def __init__(self, stage: Usd.Stage, scn, events: list[tuple], arms: dict | None = None) -> None:
        self.stage, self.t, self.speed, self.playing = stage, 0.0, 4.0, False  # speed: events per real second
        self.arms = arms or {}       # robot station id -> solved Arm (cuMotion trajectories)
        self._posed: set[str] = set()
        self.end = scn.run.duration_h * 3600
        self.times = sorted({e[0] for e in events})  # one step per distinct event time
        self.i, self._acc = 0, 0.0
        self.stays = part_stays(events, self.end)
        self.index, self.track = LoadIndex(self.stays), StateTrack(events)
        self.flow = husd.get_flow(stage)
        self.prims = {p.GetName(): p for p in husd.equipment_prims(stage)}
        self.items = {sid: husd.item_of(p) for sid, p in self.prims.items()}
        self.ports = {sid: {n: tuple(husd.world_port(p, n)) for n in ("in_lowered", "out_lowered")}
                      for sid, p in self.prims.items() if not self.items[sid].transfer}
        self.home = {sid: tuple(husd.world_pos(p)) for sid, p in self.prims.items()}
        self.slots = {}
        for sid, prim in self.prims.items():
            if self.items[sid].type == "buffer":
                m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                size = husd.size_of(prim)
                self.slots[sid] = [tuple(m.Transform(Gf.Vec3d(x, y, size[2])))
                                   for x, y in slot_positions(size, int(husd.read_params(prim)["slots"]))]
        self.shape = "cylinder" if (stage.GetPrimAtPath(husd.PLANT).GetAttribute("factory:loadShape").Get() == "cylinder") else "box"
        UsdGeom.Xform.Define(stage, LOADS)
        self._pool: list = []      # (imageable, translate op)
        self._shown = 0
        self._tools: dict[str, tuple] = {}
        self.render()

    # ---- clock -----------------------------------------------------------------------
    def step(self, dt: float) -> None:
        self._acc += dt * self.speed
        if self._acc >= 1.0:
            n, self._acc = int(self._acc), self._acc % 1.0
            self.step_events(n)

    def step_events(self, n: int = 1) -> None:
        """Jump forward n event times."""
        if not self.times:
            return
        self.i = min(self.i + n, len(self.times) - 1)
        self.t = self.times[self.i]
        if self.i == len(self.times) - 1:
            self.playing = False
        self.render()

    # ---- where is a load -------------------------------------------------------------
    def _port(self, sid: str | None, name: str, fallback) -> tuple:
        return self.ports[sid][name] if sid in self.ports else fallback

    def _position(self, part: int, queued: dict[str, int], blocked: dict[str, int]) -> tuple:
        stays, t = self.stays[part], self.t
        i = max(k for k, s in enumerate(stays) if s.t0 <= t)
        s = stays[i]
        if t >= s.t1 and i + 1 < len(stays):  # left this unit, waiting in the next station's queue
            sid = stays[i + 1].station
            n = queued[sid] = queued.get(sid, 0) + 1
            x, y, z = self._port(sid, "in_lowered", self.home[sid])
            return (x - QUEUE_PITCH * n, y, z)
        sid, kind = s.station, self.items[s.station].type
        k = (t - s.t0) / (s.t_work - s.t0) if s.t_work > s.t0 else 1.0
        if self.items[sid].transfer:
            src = stays[i - 1].station if i else None
            dst = stays[i + 1].station if i + 1 < len(stays) else None
            pick = self._port(src, "out_lowered", self.home[sid])
            place = self._port(dst, "in_lowered", self.home[sid])
            move = self.arms[sid].moves.get((src, dst)) if sid in self.arms else None
            if move is not None:  # replay the solved UR10 trajectory; the load rides on the tool
                arm, cycle_time = self.arms[sid], min(k, 1.0) * move.cycle_s
                q = move.q(cycle_time)
                arm.pose(q)
                self._posed.add(sid)
                if cycle_time < move.t_pick:
                    return pick
                if move.holding(cycle_time):
                    tool = arm.tool_world(q)
                    return (tool[0], tool[1], tool[2] - LOAD)
                return place
            lift = husd.read_params(self.prims[sid]).get("lift", 0.25)
            pos = pick_place_point(pick, place, lift, k * 2)  # first half: carry; second half: empty return
            self._tools[sid] = pos if k <= 0.5 else pick_place_point(place, pick, lift, k * 2 - 1)
            return pos
        if kind == "conveyor":
            if k >= 1.0:  # accumulated at the end, waiting for downstream
                n = blocked[sid] = blocked.get(sid, 0) + 1
                x, y, z = self.ports[sid]["out_lowered"]
                a = self.ports[sid]["in_lowered"]
                back = min(1.0, QUEUE_PITCH * (n - 1) / max(1e-6, ((x - a[0]) ** 2 + (y - a[1]) ** 2) ** 0.5))
                return lerp((x, y, z), a, back)
            return lerp(self.ports[sid]["in_lowered"], self.ports[sid]["out_lowered"], k)
        if kind == "buffer":
            return self.slots[sid][s.unit % len(self.slots[sid])]
        x, y, z = self.ports[sid]["in_lowered"]
        return (x, y + 0.22 * s.unit, z)

    # ---- draw ------------------------------------------------------------------------
    def _load_prim(self, i: int):
        while len(self._pool) <= i:
            path = f"{LOADS}/load_{len(self._pool)}"
            if self.shape == "cylinder":
                geom = UsdGeom.Cylinder.Define(self.stage, path)
                geom.CreateRadiusAttr(LOAD / 2)
                geom.CreateHeightAttr(LOAD)
            else:
                geom = UsdGeom.Cube.Define(self.stage, path)
                geom.CreateSizeAttr(LOAD)
            geom.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.55, 0.10)])
            self._pool.append((UsdGeom.Imageable(geom), geom.AddTranslateOp()))
        return self._pool[i]

    def render(self) -> None:
        for sid, prim in self.prims.items():
            husd.set_state(prim, self.track.station_state(sid, self.t))
        queued, blocked = {}, {}
        self._tools.clear()
        was_posed, self._posed = self._posed, set()
        active = self.index.active(self.t)
        for i, part in enumerate(active):
            imageable, op = self._load_prim(i)
            x, y, z = self._position(part, queued, blocked)
            op.Set(Gf.Vec3d(x, y, z + LOAD / 2))
            if i >= self._shown:
                imageable.MakeVisible()
        for i in range(len(active), self._shown):
            self._pool[i][0].MakeInvisible()
        self._shown = len(active)
        for sid in was_posed - self._posed:  # a robot that just went idle waits above its pick point
            arm = self.arms[sid]
            arm.pose(next(iter(arm.moves.values())).q(0.0))

    def close(self) -> None:
        try:
            self.stage.RemovePrim(LOADS)
        except Exception:  # noqa: BLE001  the stage was closed (File > New / Open): nothing left to clean up
            return
        for prim in self.prims.values():
            if prim.IsValid():
                husd.set_state(prim, "IDLE")
