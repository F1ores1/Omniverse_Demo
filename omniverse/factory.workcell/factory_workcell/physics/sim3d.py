"""3D simulation loop. Every load move between items is explicit and kinematic:

- Load source -> next item: the load appears there directly (sources are the one exception).
- Conveyor -> conveyor: handed straight on, belt to belt.
- A Transfer / robot item in the flow moves loads from the item before it to the item after it.
- Any other two neighbours: a default transfer (lift, move, lower) is added, and Checks flags the gap.

Each item follows the same hand-over: can_accept / accept / ready_at_out / release. Driven by PhysX step events while
Kit's timeline plays. Logs (time, load, item, "enter" / "leave") for later 3D -> DES tables.

Fast forward: every load and item here is kinematic, so `speed` > 1 runs the same logic `speed` times per physics step
(same dt each time). Time in each state is summed per item for the bottleneck insights.
"""
from __future__ import annotations

import random

import omni.physx
from pxr import Gf, Sdf, Usd, UsdPhysics

from simcore import usd as husd
from simcore.catalog import SOURCE_TYPES

from . import loads
from .conveyor import Conveyor3D
from .robot3d import RobotTransfer3D
from .station import Station3D
from .transfer import Transfer3D

MOVERS = {"transfer", "robot"}  # items that move loads between their neighbours


class Source3D:
    """Makes loads at exponential intervals. Holds them (BLOCKED) while the next item has no room.

    Next item accepts directly -> the load appears there. Next item is a transfer -> the load waits on the pad."""

    def __init__(self, prim: Usd.Prim, rng: random.Random) -> None:
        self.prim, self.id, self.rng = prim, prim.GetName(), rng
        p = husd.read_params(prim)
        self.shape, self.mean = p.get("shape", "box"), float(p["arrival_mean"])
        self.next_t = rng.expovariate(1.0 / self.mean)
        self.waiting, self.count = 0, 0
        self.on_pad: Usd.Prim | None = None
        self.departing = False  # a picked load is still lifting off the pad
        self.pad_top = husd.world_pos(prim) + Gf.Vec3d(0, 0, husd.size_of(prim)[2] + loads.LOAD_SIZE / 2 + 0.002)

    def due(self, t: float) -> None:
        while t >= self.next_t:
            self.waiting += 1
            self.next_t += self.rng.expovariate(1.0 / self.mean)

    def make(self, stage: Usd.Stage, at: Gf.Vec3d, material) -> Usd.Prim:
        self.waiting -= 1
        self.count += 1
        prim = loads.spawn(stage, f"{self.id}_load_{self.count}", self.shape, at, material)
        loads.to_kinematic(prim)
        return prim

    # a source feeding a transfer behaves like an item with the load ready on its pad
    def ready_at_out(self) -> bool:
        return self.on_pad is not None

    def front(self) -> Usd.Prim | None:
        return self.on_pad

    def release(self) -> Usd.Prim:
        prim, self.on_pad = self.on_pad, None
        self.departing = True
        return prim

    def cleared(self) -> None:
        self.departing = False

    def state(self) -> str:
        return "BLOCKED" if self.waiting or self.on_pad is not None else "PROCESSING"


class Sim3D:
    def __init__(self, stage: Usd.Stage, seed: int = 1, arms: dict | None = None) -> None:
        self.stage, self.rng, self.t = stage, random.Random(seed), 0.0
        self.events: list[tuple[float, str, str, str]] = []
        self.speed = 1                                   # sub-steps per physics step (fast forward)
        self.time_in_state: dict[str, dict[str, float]] = {}  # item id -> state -> seconds (units averaged)
        self.exits = 0
        self.material = loads.physics_material(stage, 1.0)
        self._ensure_scene()
        prims = {p.GetName(): p for p in husd.equipment_prims(stage)}
        # flow steps: a step is one item, or a list of parallel items (e.g. two presses doing the same operation)
        steps = [s if isinstance(s, list) else [s] for s in husd.get_flow(stage)]
        self.order = [sid for step in steps for sid in step]
        kind = {sid: husd.item_of(prims[sid]).type for sid in self.order}
        self.items: dict[str, object] = {}
        for sid in self.order:
            if kind[sid] in SOURCE_TYPES:
                self.items[sid] = Source3D(prims[sid], self.rng)
            elif kind[sid] in ("conveyor", "buffer"):  # a buffer is an accumulating track: loads take time to the front
                self.items[sid] = Conveyor3D(prims[sid])
            elif kind[sid] not in MOVERS:
                self.items[sid] = Station3D(prims[sid], self.rng)
        # links between neighbours: ("direct", a, b) or ("move", Transfer3D / RobotTransfer3D)
        self.links: list[tuple] = []
        for i, step in enumerate(steps):
            if kind[step[0]] in MOVERS:  # a transfer item: moves from the step before to the step after
                sid = step[0]
                if 0 < i < len(steps) - 1:
                    srcs, dsts = [self.items[s] for s in steps[i - 1]], [self.items[s] for s in steps[i + 1]]
                    arm = (arms or {}).get(sid)
                    if kind[sid] == "robot" and arm is not None and all(
                            (a.id, b.id) in arm.moves for a in srcs for b in dsts):
                        self.links.append(("move", RobotTransfer3D(srcs, dsts, prims[sid], arm)))  # solved loops
                    else:  # lift-and-place: one source and one destination only
                        self.links.append(("move", Transfer3D(srcs[0], dsts[0], prims[sid])))
                continue
            if i + 1 >= len(steps) or kind[steps[i + 1][0]] in MOVERS:
                continue
            a, b = self.items[step[0]], self.items[steps[i + 1][0]]  # no transfer item: single items only
            if isinstance(a, Source3D) or (isinstance(a, Conveyor3D) and isinstance(b, Conveyor3D)):
                self.links.append(("direct", a, b))
            else:  # no transfer between them: default lift-and-place
                self.links.append(("move", Transfer3D(a, b)))
        self._shown: dict[str, str] = {}
        self._prims = prims
        self._sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)

    def _ensure_scene(self) -> None:  # for loads that switch to dynamic (drops); tracks and transfers do not need it
        if not any(p.IsA(UsdPhysics.Scene) for p in self.stage.Traverse()):
            scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path("/PhysicsScene"))
            scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
            scene.CreateGravityMagnitudeAttr(9.81)

    def _still_valid(self) -> bool:
        import omni.usd

        if omni.usd.get_context().get_stage() is not self.stage or not all(p.IsValid() for p in self._prims.values()):
            return False
        held = [l[1].load for l in self.links if l[0] == "move" and l[1].load is not None]
        held += [s.on_pad for s in self.items.values() if isinstance(s, Source3D) and s.on_pad is not None]
        return all(p.IsValid() for p in held)

    def _log(self, load: str, item: str, what: str) -> None:
        self.events.append((self.t, load, item, what))

    def _on_step(self, dt: float) -> None:
        for _ in range(max(1, int(self.speed))):
            if not self._still_valid():  # stage replaced (File > New / Open) or loads removed under us: stop quietly
                self._sub = None
                return
            self._advance(dt)

    def _advance(self, dt: float) -> None:
        self.t += dt
        for item in self.items.values():
            if isinstance(item, Source3D):
                item.due(self.t)
            elif isinstance(item, Station3D):
                item.step(self.t)
        blocked_out: dict[str, bool] = {}
        for link in reversed(self.links):  # downstream first, so room frees up before upstream hands on
            if link[0] == "move":
                for load, item, what in link[1].step(dt):
                    self._log(load, item, what)
                continue
            _, a, b = link
            if isinstance(a, Source3D):
                if a.waiting and hasattr(b, "accept") and b.can_accept():
                    prim = a.make(self.stage, b.drop_point(), self.material)
                    b.accept(prim)
                    self._log(prim.GetName(), b.id, "enter")
            elif a.ready_at_out() and b.can_accept():
                prim = a.release()
                a.cleared()  # belt to belt: no lift, the slot frees at once
                b.accept(prim)
                self._log(prim.GetName(), a.id, "leave")
                self._log(prim.GetName(), b.id, "enter")
        for src in (i for i in self.items.values() if isinstance(i, Source3D)):  # sources feeding a transfer
            if src.waiting and src.on_pad is None and not src.departing and not any(l[0] == "direct" and l[1] is src for l in self.links):
                src.on_pad = src.make(self.stage, src.pad_top - Gf.Vec3d(0, 0, loads.LOAD_SIZE / 2), self.material)
                loads.set_world_position(src.on_pad, src.pad_top)
        last = self.items.get(self.order[-1]) if self.order else None
        if isinstance(last, (Conveyor3D, Station3D)) and last.ready_at_out():  # end of the line: finished loads leave
            prim = last.release()
            last.cleared()  # leaves the line at once
            self._log(prim.GetName(), last.id, "exit")
            self.exits += 1
            self.stage.RemovePrim(prim.GetPath())
        for sid, item in self.items.items():
            if isinstance(item, Conveyor3D):
                blocked_out[sid] = item.ready_at_out()
                item.step(dt)
            elif isinstance(item, Station3D):
                blocked_out[sid] = item.ready_at_out()
        self._show_states(blocked_out, dt)

    def _account(self, sid: str, states: list[str], dt: float) -> None:
        tis = self.time_in_state.setdefault(sid, {})
        for state in states:
            tis[state] = tis.get(state, 0.0) + dt / len(states)

    def stations(self) -> dict[str, dict[str, float]]:
        """Share of elapsed 3D time per state, per item (load sources left out), in flow order."""
        out = {}
        for sid in self.order:
            tis = self.time_in_state.get(sid)
            if not tis or isinstance(self.items.get(sid), Source3D):
                continue
            span = sum(tis.values()) or 1.0
            out[sid] = {k: tis.get(k.upper(), 0.0) / span for k in ("processing", "blocked", "starved", "down")}
            out[sid]["utilization"] = out[sid].pop("processing")
        return out

    def _show_states(self, blocked_out: dict[str, bool], dt: float = 0.0) -> None:
        states = {}
        for sid, item in self.items.items():
            states[sid] = item.state() if isinstance(item, Source3D) else (
                item.state_now(blocked_out[sid]) if isinstance(item, Conveyor3D) else item.state(blocked_out.get(sid, False)))
        for link in self.links:
            if link[0] == "move" and link[1].prim is not None:
                states[link[1].id] = link[1].state()
        for sid, state in states.items():
            if isinstance(self.items.get(sid), Station3D) and self.items[sid].units > 1:
                self._account(sid, self.items[sid].unit_states(), dt)
            else:
                self._account(sid, [state], dt)
        for sid, item in self.items.items():  # machines: one colour per parallel unit
            if isinstance(item, Station3D) and item.units > 1:
                states.pop(sid, None)
                for k, state in enumerate(item.unit_states()):
                    if self._shown.get((sid, k)) != state:
                        husd.set_unit_state(self._prims[sid], k, state)
                        self._shown[(sid, k)] = state
        for sid, state in states.items():
            if self._shown.get(sid) != state and sid in self._prims:  # only touch USD when it changes
                husd.set_state(self._prims[sid], state)
                self._shown[sid] = state

    def reset(self) -> None:
        """Stop: drop the subscription, remove all loads, show every item idle."""
        import omni.usd

        self._sub = None
        if omni.usd.get_context().get_stage() is not self.stage:  # its file was closed: nothing left to clean up
            return
        for link in self.links:  # robots: drive targets back to their home pose
            if link[0] == "move" and hasattr(link[1], "reset"):
                link[1].reset()
        loads.remove_all(self.stage)
        for p in husd.equipment_prims(self.stage):
            husd.set_state(p, "IDLE")
