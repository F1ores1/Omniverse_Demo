"""Session: everything the window does, without any UI. Scripts and tests drive this directly."""
from __future__ import annotations

import time
from pathlib import Path

import omni.kit.app
import omni.usd
import yaml
from pxr import Gf, Tf, Usd, UsdGeom

from simcore import usd as husd
from simcore.catalog import CATALOG
from simcore.config import load_scenario
from simcore.config.schema import Scenario
from simcore.experiment import simulate
from simcore.metrics.insights import bottleneck_insights

from .player import Player

SNAP_RADIUS = 0.5   # m: how close a face must be to its snap target
SNAP_GAP = 0.7      # m: spacing left between machines for a robot
SETTLE_S = 0.35     # s without movement before a dragged item is snapped
THROUGH = {"conveyor", "buffer"}


class Session:
    def __init__(self, repo: Path) -> None:
        self.repo = repo
        self.status = "Add equipment from the catalog."
        self.problems: list[str] = []
        self.results: dict | None = None
        self.scenario_path: Path | None = None
        self.player: Player | None = None
        self.on_change = lambda: None  # the window sets this to refresh itself
        self.on_insights = lambda: None  # the window sets this to redraw the insights panel (live during 3D)
        self.mode: str | None = None     # "DES" or "3D": which run the insights panel shows
        self.speed_3d = 1
        self._last3d: dict | None = None  # 3D numbers kept after Stop resets the simulation
        self._next_insights = 0.0
        self.arms: dict = {}             # robot station id -> solved Arm
        self.robot_errors: list[str] = []
        self._moved: dict[str, float] = {}
        self._good: dict[str, tuple] = {}
        self._internal = False
        self._listener = None
        self._listened_stage = None
        self._update = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._tick, name="factory.workcell")

    def shutdown(self) -> None:
        self.stop()
        if self.sim3d is not None:  # a reload must not leave a 3D simulation running against a stale stage
            self.sim3d.reset()
            self.sim3d = None
        self._update = None
        if self._listener:
            self._listener.Revoke()

    # ---- stage -----------------------------------------------------------------------
    @property
    def stage(self) -> Usd.Stage:
        stage = omni.usd.get_context().get_stage()
        if stage is not self._listened_stage:
            if self._listener:
                self._listener.Revoke()
            self._listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_usd_change, stage)
            self._listened_stage = stage
            self.player = None  # a DES view of the previous file would keep writing to its expired prims
            self._moved.clear()
            self._good.clear()
        return stage

    def equipment(self) -> list[Usd.Prim]:
        return husd.equipment_prims(self.stage)

    def add(self, type_name: str, position=None, **kwargs) -> Usd.Prim:
        stage = self.stage
        first = husd.find_plant(stage) is None  # this add turns a fresh file into a plant (cm -> m)
        husd.plant(stage)
        before = husd.get_flow(stage)  # the flow is saved explicitly: once a line turns, X order means nothing
        self._internal = True
        try:
            prim = husd.add_equipment(stage, type_name, position or husd.auto_position(stage, type_name, SNAP_GAP), **kwargs)
            husd.set_flow(stage, before + [prim.GetName()])  # a new item joins the end of the flow
            if position is None:
                self._follow_last(prim)
        finally:
            self._internal = False
        if first:  # the camera is still where it was in cm, ~100x too far: frame the new item
            from omni.kit.viewport.utility import frame_viewport_prims, get_active_viewport

            frame_viewport_prims(get_active_viewport(), prims=[str(prim.GetPath())])
        self._good[str(prim.GetPath())] = self._pose(prim)
        self.status = f"Added {prim.GetName()}."
        if type_name == "robot":  # show the arm straight away; its moves are solved on Run
            try:
                from .robot import Arm

                self._internal = True
                arm = Arm(prim)
                arm.attach()
                arm.home(self._pick_point(prim))  # upright, facing the item it picks from
            except Exception as e:  # noqa: BLE001
                self.status = f"Added {prim.GetName()} without an arm: {e}"
            finally:
                self._internal = False
        self.check()
        return prim

    def _pick_point(self, prim: Usd.Prim):
        """Outlet of the item before this one in the flow (where a robot picks), or None."""
        before, _ = husd.neighbours(husd.get_flow(self.stage), prim.GetName())
        for sid in before:
            src = self.stage.GetPrimAtPath(f"{husd.PLANT}/{sid}")
            if src.IsValid() and not husd.item_of(src).transfer:
                return husd.world_port(src, "out_lowered")
        return None

    def selected(self) -> Usd.Prim | None:
        """The equipment item that owns the current viewport or stage selection."""
        for path in omni.usd.get_context().get_selection().get_selected_prim_paths():
            prim = self.stage.GetPrimAtPath(path)
            while prim and prim.IsValid() and not prim.IsPseudoRoot():
                if prim.HasAttribute("factory:equipmentType"):
                    return prim
                prim = prim.GetParent()
        return None

    def apply(self, prim: Usd.Prim, size=None, values: dict | None = None) -> None:
        """Write edited parameters back to the prim and rebuild its geometry."""
        self._internal = True
        try:
            if size is not None:
                prim.GetAttribute("factory:size").Set(Gf.Vec3d(*size))
            for name, value in (values or {}).items():
                prim.GetAttribute(name).Set(value)
            husd.rebuild(prim)
        finally:
            self._internal = False
        if "factory:count" in (values or {}):  # more or fewer units: wider, so re-snap the line
            self.relayout()
        self.check()

    def set_3d(self, on: bool, prims: list[Usd.Prim] | None = None) -> None:
        for prim in prims or self.equipment():
            self.apply(prim, values={"factory:show3d": on})
            arm = prim.GetChild("Arm")  # the UR10 sits beside Geo, so the 2D rebuild doesn't flatten it
            if arm.IsValid():
                self._internal = True
                try:
                    UsdGeom.Imageable(arm).MakeVisible() if on else UsdGeom.Imageable(arm).MakeInvisible()
                finally:
                    self._internal = False

    # ---- drag guard ------------------------------------------------------------------
    @staticmethod
    def _pose(prim: Usd.Prim) -> tuple:
        t, r, *_ = UsdGeom.XformCommonAPI(prim).GetXformVectors(Usd.TimeCode.Default())
        return (Gf.Vec3d(t), Gf.Vec3f(r))

    def _set_pose(self, prim: Usd.Prim, pose: tuple) -> None:
        self._internal = True
        try:
            api = UsdGeom.XformCommonAPI(prim)
            api.SetTranslate(pose[0])
            api.SetRotate(pose[1])
        finally:
            self._internal = False

    def _on_usd_change(self, notice, _stage) -> None:
        if self._internal:
            return
        for path in notice.GetChangedInfoOnlyPaths():
            if path.IsPropertyPath() and path.name.startswith("xformOp"):
                prim = self._listened_stage.GetPrimAtPath(path.GetPrimPath())
                if prim.HasAttribute("factory:equipmentType"):
                    self._moved[str(prim.GetPath())] = time.monotonic()

    def _tick(self, event) -> None:
        if self.player and self.player.stage is not omni.usd.get_context().get_stage():
            self.player = None  # its file was closed
        if self.player and self.player.playing:
            self.player.step(event.payload["dt"])
        if self.sim3d is not None and self.mode == "3D" and time.monotonic() > self._next_insights:
            self._next_insights = time.monotonic() + 1.0  # live 3D insights, once a second
            self.on_insights()
        if not self._moved:
            return
        now = time.monotonic()
        for path in [p for p, t in self._moved.items() if now - t > SETTLE_S]:
            del self._moved[path]
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                self.settle(prim)

    def settle(self, prim: Usd.Prim) -> None:
        """Called when an item stops moving: undo the move if it now overlaps something."""
        stage, name = self.stage, prim.GetName()
        clashes = husd.overlap_problems(stage, only=prim)
        if clashes and str(prim.GetPath()) in self._good:
            self._set_pose(prim, self._good[str(prim.GetPath())])
            self.status = f"Move refused: {clashes[0]}. {name} was put back."
        else:
            self._good[str(prim.GetPath())] = self._pose(prim)
            self.status = f"Moved {name}."
        self.check()

    # ---- layout: rotate and line up ---------------------------------------------------
    def rotate(self, prim: Usd.Prim, degrees: float) -> None:
        """Turn an item 90 degrees left (+90) or right (-90), then line every item up again."""
        self._internal = True
        try:
            husd.rotate(prim, degrees)
        finally:
            self._internal = False
        self.relayout()
        self.status = f"Rotated {prim.GetName()} {'left' if degrees > 0 else 'right'}; line re-snapped."

    def _follow_last(self, prim: Usd.Prim) -> None:
        """A new item takes the rotation of the last item in the line and snaps in front of it."""
        flow = husd.get_flow(self.stage)
        line = [s for s in flow if isinstance(s, str) and husd.item_of(self.stage.GetPrimAtPath(f"{husd.PLANT}/{s}")).type
                not in husd.MOVER_TYPES]
        name = prim.GetName()
        if husd.item_of(prim).type in husd.MOVER_TYPES:  # transfer / robot / gantry: in front of the last item
            if line:
                husd.place_mover(self.stage.GetPrimAtPath(f"{husd.PLANT}/{line[-1]}"), prim, SNAP_GAP)
            return
        if name not in line or line.index(name) == 0:
            return
        prev = self.stage.GetPrimAtPath(f"{husd.PLANT}/{line[line.index(name) - 1]}")
        UsdGeom.XformCommonAPI(prim).SetRotate(Gf.Vec3f(0.0, 0.0, husd.yaw_of(prev)))
        husd.layout_chain(self.stage, flow, start=line.index(name), gap=SNAP_GAP)

    def remove(self, sid: str) -> None:
        """Delete an item: out of the flow, off the stage, then re-snap the line."""
        self.stop()
        if self.sim3d is not None:
            self.stop_3d()
        flow = [s for s in husd.get_flow(self.stage) if s != sid]
        flow = [[x for x in s if x != sid] if isinstance(s, list) else s for s in flow]
        self._internal = True
        try:
            husd.set_flow(self.stage, [s[0] if isinstance(s, list) and len(s) == 1 else s for s in flow if s])
            self.stage.RemovePrim(f"{husd.PLANT}/{sid}")
        finally:
            self._internal = False
        self._good.pop(f"{husd.PLANT}/{sid}", None)
        self.relayout()
        self.status = f"Removed {sid}."
        self.on_change()

    def relayout(self) -> None:
        """Snap each item's inlet to the previous item's outlet, following the product flow. Rotations are kept."""
        self._internal = True
        try:
            husd.layout_chain(self.stage, husd.get_flow(self.stage), gap=SNAP_GAP)
        finally:
            self._internal = False
        for prim in self.equipment():
            self._good[str(prim.GetPath())] = self._pose(prim)
        self.check()

    def check(self) -> list[str]:
        """Overlaps, robot reach and tool paths through machines. Shown in the window."""
        stage = self.stage
        flow = husd.get_flow(stage)
        self.problems = (husd.overlap_problems(stage) + husd.reach_problems(stage, flow)
                         + husd.path_problems(stage, flow) + husd.transfer_problems(stage, flow) + self.robot_errors)
        self.on_change()
        return self.problems

    # ---- run -------------------------------------------------------------------------
    def build_scenario(self) -> Scenario:
        """Stage -> scenario YAML on disk -> validated scenario. The YAML file is what gets simulated."""
        raw = husd.stage_to_scenario(self.stage)
        Scenario.model_validate(raw)  # fail here, with a readable message, before writing anything
        out = self.repo / "output" / "layouts"
        out.mkdir(parents=True, exist_ok=True)
        self.scenario_path = out / f"{raw['name']}.yaml"
        self.scenario_path.write_text(
            "# Written by the plug-and-play layout editor. Runs headless: python -m simcore.run <this file>\n"
            + yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
        return load_scenario(self.scenario_path)

    # ---- robots ----------------------------------------------------------------------
    def robots(self) -> list[Usd.Prim]:
        flow = husd._ids(husd.get_flow(self.stage))
        return [p for p in self.equipment() if husd.item_of(p).type == "robot" and p.GetName() in flow]

    def solve_robots(self) -> dict | None:
        """Plan every robot move with cuMotion, show the arms, and save the move-time table for the DES.

        TODO: no button for this any more. It runs before every DES run; it should also run by itself whenever a robot
        is placed or moved (or the items it serves move), so the move table is always current."""
        from .robot import Arm, write_table

        stage, root = self.stage, husd.plant(self.stage)
        flow = husd.get_flow(stage)
        self.arms, self.robot_errors = {}, []
        self._internal = True
        try:
            for prim in self.robots():
                arm = Arm(prim)
                arm.attach()
                speed = float(husd.read_params(prim).get("speed_scale", 1.0))
                for src, dst, pick, place in husd.transfer_points(stage, flow, prim.GetName()):
                    arm.solve(src, dst, pick, place, lift=0.25, speed=speed)
                arm.home(self._pick_point(prim))  # upright, facing the pick
                arm.line_up()  # several loops (a parallel step): keep them on the same turn of the base joint
                if arm.moves:  # wait where the solved loop starts (above the pick node), so the first pick has no jump
                    arm.home_q = next(iter(arm.moves.values())).q(0.0)
                    arm.pose(arm.home_q)
                if arm.error:
                    self.robot_errors.append(arm.error)
                self.arms[prim.GetName()] = arm
            table = None
            if any(a.moves for a in self.arms.values()):
                name = root.GetAttribute("factory:plantName").Get()
                table = write_table(self.repo / "output" / "layouts" / "tables" / f"{name}.yaml", self.arms)
                husd._set(root, "factory:moveTable", f"tables/{name}.yaml")
            else:
                husd._set(root, "factory:moveTable", "")
        finally:
            self._internal = False
        return table

    def run(self, duration_h: float | None = None, replications: int | None = None) -> dict:
        """Run the DES on the current layout. Overrides are for the long 'run for weeks' button."""
        self.stop()
        if self.sim3d is not None:
            self.stop_3d()
        if self.robots():  # the layout may have changed, so robot moves are solved again before every run
            try:
                self.solve_robots()
            except Exception as e:  # noqa: BLE001  (no solver: fall back to the handling time on the robot)
                self.robot_errors = [f"Robot solver unavailable, using each robot's handling_time: {e}"]
                husd._set(husd.plant(self.stage), "factory:moveTable", "")
        scn = self.build_scenario()
        update = {k: v for k, v in (("duration_h", duration_h), ("replications", replications)) if v is not None}
        if update:
            scn = scn.model_copy(update={"run": scn.run.model_copy(update=update)})
        t0 = time.perf_counter()
        res = simulate(scn, record_events=True)
        self._events = res.pop("events")
        self._scn, self.results = scn, res
        res["wall_s"] = time.perf_counter() - t0
        k = res["kpis"]
        self.mode = "DES"
        self.status = (f"Ran {scn.run.duration_h:g} h x {scn.run.replications} in {res['wall_s']:.1f} s: "
                       f"{k['throughput_per_h']['mean']:.1f} parts/h, bottleneck {res['bottleneck']}. "
                       "Showing it in 2D, event by event.")
        self.play()
        self.on_change()
        return res

    # ---- bottleneck insights ---------------------------------------------------------
    def insights(self) -> dict | None:
        """Bottleneck insights for the last DES run or the current / last 3D run (whichever ran last)."""
        order = husd._ids(husd.get_flow(self.stage))
        if self.mode == "DES" and self.results is not None:
            res = self.results
            out = bottleneck_insights(res["stations"], order)
            k = res["kpis"]
            out.update(mode="DES", stations=res["stations"], throughput=k["throughput_per_h"]["mean"],
                       detail=f"{res['meta']['duration_h']:g} h x {res['meta']['replications']} replications, "
                              f"+/- {k['throughput_per_h']['ci95']:.1f} parts/h (95% CI), "
                              f"cycle time {k['cycle_time_s']['mean']:.0f} s")
            return out
        if self.mode == "3D":
            snap = self._snapshot_3d() if self.sim3d is not None else self._last3d
            if snap is None or not snap["stations"]:
                return None
            out = bottleneck_insights(snap["stations"], order)
            rate = snap["exits"] / snap["t"] * 3600 if snap["t"] > 0 else 0.0
            out.update(mode="3D", stations=snap["stations"], throughput=rate,
                       detail=f"{snap['t'] / 60:.1f} min of 3D time, {snap['exits']} loads out (warm-up included)")
            return out
        return None

    def _snapshot_3d(self) -> dict:
        return {"stations": self.sim3d.stations(), "t": self.sim3d.t, "exits": self.sim3d.exits}

    # ---- 3D playback -----------------------------------------------------------------
    def play(self, speed: float = 4.0) -> None:
        """2D event-by-event view of the last DES run (`speed` events per second). Every item is shown as a 2D tile."""
        if self.results is None:
            self.run()
            return
        if self.player is None:
            self.set_3d(False)
            self.player = Player(self.stage, self._scn, self._events, {})  # 2D: no arm poses
        self.player.speed, self.player.playing = speed, True

    def step_des(self) -> None:
        if self.player:
            self.player.playing = False
            self.player.step_events(1)

    def pause(self) -> None:
        if self.player:
            self.player.playing = False

    def stop(self) -> None:
        if self.player:
            self._internal = True
            try:
                self.player.close()
            finally:
                self._internal = False
            self.player = None

    # ---- 3D simulation (PhysX) ---------------------------------------------------------
    # 3D mode is its own simulation: PhysX + kinematics + motion profiles on the stage, driven by Kit's timeline.
    # It does not replay the DES. Its measured times become tables the DES can use later (not built yet).
    sim3d = None

    def play_3d(self) -> None:
        import omni.timeline

        from .physics import Sim3D

        self.stop()  # never mix the DES view with physics
        self.mode = "3D"
        timeline = omni.timeline.get_timeline_interface()
        if self.sim3d is not None and (self.sim3d.stage is not self.stage or self.sim3d._sub is None):
            self.sim3d = None  # left over from another file (File > New / Open): start fresh
        if self.sim3d is None:  # fresh start (not a resume after Pause)
            if self.robots():  # robots pick with their cuMotion trajectories; solve them for the current layout
                try:
                    self.solve_robots()
                except Exception as e:  # noqa: BLE001  (no solver: robots fall back to the lift-and-place transfer)
                    self.arms, self.robot_errors = {}, [f"Robot solver unavailable, robots use lift-and-place: {e}"]
            self.sim3d = Sim3D(self.stage, arms=self.arms)
            self._last3d = None
            timeline.set_end_time(1.0e6)  # don't loop back to 0 after a few seconds
            timeline.set_looping(False)
        self.sim3d.speed = self.speed_3d
        timeline.play()

    def set_speed_3d(self, speed: int) -> None:
        """Fast forward: sub-steps of the 3D logic per physics step."""
        self.speed_3d = speed
        if self.sim3d is not None:
            self.sim3d.speed = speed

    def pause_3d(self) -> None:
        import omni.timeline

        omni.timeline.get_timeline_interface().pause()

    def stop_3d(self) -> None:
        import omni.timeline

        omni.timeline.get_timeline_interface().stop()  # PhysX resets bodies to where they started
        if self.sim3d is not None:
            self._last3d = self._snapshot_3d()  # keep the insights after the reset
            self.sim3d.reset()
            self.sim3d = None


def catalog_labels() -> list[tuple[str, str]]:
    return [(item.type, item.label) for item in CATALOG.values()]
