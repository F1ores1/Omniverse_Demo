"""The Modular Work Cell window: catalog, selected item, flow, DES run (2D, event by event), bottleneck insights,
3D (PhysX) simulation."""
from __future__ import annotations

import asyncio

import omni.kit.app
import omni.ui as ui
import omni.usd
from pxr import UsdGeom

from simcore import usd as husd
from simcore.catalog import CHOICES, DIST_PARAMS, TIME_FORMS, autofill, time_form, time_text

from .session import Session, catalog_labels

RED, GREEN, GREY = 0xFF4040E0, 0xFF60C060, 0xFFA0A0A0
SPEEDS_3D = (1, 5, 10, 20)  # 3D fast forward: logic sub-steps per physics step
TITLE = "Modular Work Cell"
HIDDEN_AT_START = ("Render Settings", "Variant Presenter")
# Fields left out of section 2 per type to keep the demo simple (they keep their defaults).
HIDE_FIELDS = {"conveyor": {"accel", "drive_force", "friction", "gap", "factory:buffer"}}
LABELS = {"cycle_time": "Cycle time (s)", "handling_time": "Handling time (s)", "shape": "Load shape", "arrival_dist": "Arrival distribution", "arrival_mean": "Mean time between arrivals (s)",
          "accel": "Belt acceleration (m/s2)", "speed": "Belt speed (m/s)", "friction": "Friction (belt to load)",
          "drive_force": "Belt drive force (N)", "gap": "Gap between queued loads (m)", "slots": "Max loads on it (WIP)"}


async def _dock_with_stage(window: ui.Window) -> None:
    """Tab the window in with Stage once the app's layout exists (Stage appears a few frames after startup)."""
    for _ in range(600):
        await omni.kit.app.get_app().next_update_async()
        stage_win, me = ui.Workspace.get_window("Stage"), ui.Workspace.get_window(window.title)
        if stage_win and me and stage_win.visible and stage_win.docked:
            me.dock_in(stage_win, ui.DockPosition.SAME)  # the workspace handle docks; ui.Window.dock_in does not
            for _ in range(5):
                await omni.kit.app.get_app().next_update_async()
            if ui.Workspace.get_window(window.title).docked:
                for title in HIDDEN_AT_START:  # Composer opens these as tabs by default; reopen from the Window menu
                    other = ui.Workspace.get_window(title)
                    if other:
                        other.visible = False
                me.focus()
                return


class WorkcellWindow:
    def __init__(self, session: Session) -> None:
        self.s = session
        session.on_change = self.refresh
        session.on_insights = lambda: self._kpis.rebuild()
        self._window = ui.Window(TITLE, width=440, height=900)
        # Dock as a tab with Stage / Layer / Render Settings (right-hand panel), not as a floating window.
        asyncio.ensure_future(_dock_with_stage(self._window))
        self._fields: dict = {}
        with self._window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=6, height=0):
                    self._status = ui.Label(session.status, word_wrap=True, height=0)
                    with ui.CollapsableFrame("1. Catalog", height=0):
                        with ui.VStack(spacing=2, height=0):
                            for type_name, label in catalog_labels():
                                ui.Button(f"+  {label}", height=24, clicked_fn=lambda t=type_name: self.s.add(t))
                    with ui.CollapsableFrame("2. Selected equipment", height=0):
                        self._selected = ui.Frame(height=0)
                        self._selected.set_build_fn(self._build_selected)
                    with ui.CollapsableFrame("3. Product flow", height=0):
                        self._flow = ui.Frame(height=0)
                        self._flow.set_build_fn(self._build_flow)
                    with ui.CollapsableFrame("4. Run 3D (PhysX)", height=0):
                        self._sim3d = ui.Frame(height=0)
                        self._sim3d.set_build_fn(self._build_3d)
                    with ui.CollapsableFrame("5. Run DES", height=0, collapsed=True):
                        self._run = ui.Frame(height=0)
                        self._run.set_build_fn(self._build_run)
                    with ui.CollapsableFrame("6. Bottleneck insights", height=0):
                        self._kpis = ui.Frame(height=0)
                        self._kpis.set_build_fn(self._build_kpis)
        self._events = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="factory.workcell.window")

    def destroy(self) -> None:
        self._events = None
        self._window.destroy()

    def _on_stage_event(self, event) -> None:
        if event.type in (int(omni.usd.StageEventType.SELECTION_CHANGED), int(omni.usd.StageEventType.OPENED)):
            self._selected.rebuild()
            if event.type == int(omni.usd.StageEventType.OPENED):
                self.refresh()

    def refresh(self) -> None:
        self._status.text = self.s.status
        frames = (self._selected, self._flow, self._kpis, self._sim3d)
        for frame in frames + ((self._run,) if not hasattr(self, "_run_fields") else ()):  # run form: build once
            frame.rebuild()

    def _no_stage(self) -> bool:
        """True while the app has no stage yet (startup). Draws a placeholder instead of failing."""
        if omni.usd.get_context().get_stage() is not None:
            return False
        ui.Label("Waiting for a stage...", height=20)
        return True

    # ---- 2. selected equipment -------------------------------------------------------
    def _build_selected(self) -> None:
        if self._no_stage():
            return
        prim = self.s.selected()
        with ui.VStack(spacing=3, height=0):
            if prim is None:
                ui.Label("Click an item in the viewport.", height=20)
                return
            item, size = husd.item_of(prim), husd.size_of(prim)
            ui.Label(f"{prim.GetName()}   ({item.label})", height=20)
            fields: dict = {}
            with ui.HStack(height=22):
                ui.Label("Size X / Y / Z (m)", width=150)
                fields["size"] = []
                for value, editable in zip(size, item.resizable):
                    f = ui.FloatField(enabled=editable)
                    f.model.set_value(value)
                    fields["size"].append(f)
            hide = HIDE_FIELDS.get(item.type, set())
            for attr, label in (("factory:count", "Parallel units"), ("factory:buffer", "Queue in front (-1 = unlimited)")):
                if attr in hide:
                    continue
                with ui.HStack(height=22):
                    ui.Label(label, width=220)
                    fields[attr] = ui.IntField()
                    fields[attr].model.set_value(prim.GetAttribute(attr).Get())
            for name, value in husd.read_params(prim).items():
                if name in hide:
                    continue
                with ui.HStack(height=22):
                    ui.Label(LABELS.get(name, name), width=220 if name in LABELS else 150)
                    if name in DIST_PARAMS:  # a time: pick its form, then fill in its numbers
                        fields[husd.PARAM + name] = self._time_row(value)
                        continue
                    if name in CHOICES:  # pick from a list
                        options = CHOICES[name]
                        combo = ui.ComboBox(options.index(value) if value in options else 0, *options)
                        fields[husd.PARAM + name] = ("choice", combo, options)
                        continue
                    f = ui.StringField() if isinstance(value, str) else ui.FloatField()
                    f.model.set_value(value)
                    fields[husd.PARAM + name] = f
            with ui.HStack(height=24, spacing=4):
                ui.Button("Rotate left 90", clicked_fn=lambda: self._rotate(prim, 90.0))
                ui.Button("Rotate right 90", clicked_fn=lambda: self._rotate(prim, -90.0))
            with ui.HStack(height=24, spacing=4):
                ui.Button("Apply", clicked_fn=lambda: self._apply(prim, fields))
                on = prim.GetAttribute("factory:show3d").Get()
                ui.Button("Show as 2D box" if on else "Open in 3D", clicked_fn=lambda: self.s.set_3d(not on, [prim]))

    def _time_row(self, value) -> tuple:
        """Dropdown of time forms (Fixed / Min-max / Min-average-max) and one number field per argument.
        Switching the form refills the numbers around the current typical time."""
        form, args = time_form(value)
        keys = [k for k, _, _ in TIME_FORMS]
        state = {"form": form, "fields": []}
        combo = ui.ComboBox(keys.index(form), *[label for _, label, _ in TIME_FORMS], width=150)
        nums = ui.Frame()

        def draw(values):
            state["fields"] = []
            with nums:
                with ui.HStack(spacing=3):
                    names = dict((k, n) for k, _, n in TIME_FORMS)[state["form"]]
                    for n, v in zip(names, values):
                        f = ui.FloatField(tooltip=n)
                        f.model.set_value(float(v))
                        state["fields"].append(f)

        def switched(model, _item):
            current = [f.model.get_value_as_float() for f in state["fields"]]
            centre = current[0] if state["form"] == "fixed" else (
                sum(current) / 2 if state["form"] == "uniform" else current[1])
            state["form"] = keys[model.get_item_value_model().get_value_as_int()]
            draw(autofill(state["form"], centre))

        draw(args)
        combo.model.add_item_changed_fn(switched)
        state["combo"] = combo
        return ("time", state)

    def _rotate(self, prim, degrees: float) -> None:
        self.s.rotate(prim, degrees)
        self.refresh()

    def _apply(self, prim, fields: dict) -> None:
        values = {}
        for attr, field in fields.items():
            if attr == "size":
                continue
            if isinstance(field, tuple) and field[0] == "time":
                state = field[1]
                values[attr] = time_text(state["form"], [f.model.get_value_as_float() for f in state["fields"]])
                continue
            if isinstance(field, tuple):  # dropdown: (kind, combo, options)
                _, combo, options = field
                values[attr] = options[combo.model.get_item_value_model().get_value_as_int()]
                continue
            old = prim.GetAttribute(attr).Get()
            m = field.model
            values[attr] = (m.get_value_as_string() if isinstance(old, str)
                            else m.get_value_as_int() if isinstance(old, int) and not isinstance(old, bool)
                            else m.get_value_as_float())
        self.s.apply(prim, size=[f.model.get_value_as_float() for f in fields["size"]], values=values)
        self.s.status = f"Updated {prim.GetName()}."
        self.refresh()

    # ---- 3. flow ---------------------------------------------------------------------
    def _build_flow(self) -> None:
        if self._no_stage():
            return
        stage = self.s.stage
        flow = husd.get_flow(stage)
        with ui.VStack(spacing=2, height=0):
            if not flow:
                ui.Label("No equipment yet.", height=20)
                return
            for i, step in enumerate(flow):
                with ui.HStack(height=22, spacing=3):
                    ui.Label(f"{i + 1}.", width=22)
                    ui.Label(step if isinstance(step, str) else "  ||  ".join(step))
                    ui.Button("Up", width=34, enabled=i > 0, clicked_fn=lambda i=i: self._move(i, -1))
                    ui.Button("Dn", width=34, enabled=i + 1 < len(flow), clicked_fn=lambda i=i: self._move(i, 1))
                    if isinstance(step, str) and husd.item_of(self.s.stage.GetPrimAtPath(f"{husd.PLANT}/{step}")).type                             not in ("load_source", "transfer", "robot"):
                        ui.Button("Parallel", width=62, tooltip="Add an identical unit beside it (same fields)",
                                  clicked_fn=lambda sid=step: self._add_unit(sid))
                    elif not isinstance(step, str):
                        ui.Button("Split", width=62, clicked_fn=lambda i=i: self._split(i))
                    if isinstance(step, str):
                        ui.Button("Remove", width=62, tooltip="Delete this item from the stage and the flow",
                                  clicked_fn=lambda sid=step: self.s.remove(sid))
            ui.Button("Reset to layout order (left to right)", height=22, clicked_fn=lambda: self._set_flow(""))

    def _set_flow(self, flow) -> None:
        if flow == "":
            husd.plant(self.s.stage).GetAttribute("factory:flow").Set("")
        else:
            husd.set_flow(self.s.stage, flow)
        self.s.status = "Flow updated."
        self.s.check()

    def _move(self, i: int, d: int) -> None:
        """Reorder the flow, then move the equipment so the line follows the new order (rotations kept)."""
        flow = husd.get_flow(self.s.stage)
        flow[i], flow[i + d] = flow[i + d], flow[i]
        husd.set_flow(self.s.stage, flow)
        self.s.relayout()
        self.s.status = "Flow updated; line re-snapped."
        self.refresh()

    def _add_unit(self, sid: str) -> None:
        """One more identical unit of this item, side by side (raises Parallel units by one)."""
        prim = self.s.stage.GetPrimAtPath(f"{husd.PLANT}/{sid}")
        n = int(prim.GetAttribute("factory:count").Get() or 1) + 1
        self.s.apply(prim, values={"factory:count": n})
        self.s.status = f"{sid}: {n} parallel units."
        self.refresh()

    def _merge(self, i: int) -> None:
        flow = husd.get_flow(self.s.stage)
        above = flow[i - 1] if isinstance(flow[i - 1], list) else [flow[i - 1]]
        flow[i - 1:i + 1] = [above + [flow[i]]]
        self._set_flow(flow)

    def _split(self, i: int) -> None:
        flow = husd.get_flow(self.s.stage)
        flow[i:i + 1] = list(flow[i])
        self._set_flow(flow)

    # ---- 5. run DES -----------------------------------------------------------------
    def _build_run(self) -> None:
        if self._no_stage():
            return
        root = husd.find_plant(self.s.stage)  # read-only: drawing the panel must not change the stage
        self._run_fields = {}
        with ui.VStack(spacing=3, height=0):
            ui.Label("Discrete-event run: fast, statistical KPIs. Then shown on the floor in 2D, one event per step "
                     "(jumps from event to event, not through simulation time).", height=0,
                     style={"color": GREY}, word_wrap=True)
            rows = (("factory:arrival", "Arrivals if no Load source (saturated, exp(60))", ui.StringField),
                    ("factory:durationH", "Run length (hours)", ui.FloatField),
                    ("factory:replications", "Replications", ui.IntField))
            for attr, label, kind in rows:
                with ui.HStack(height=22):
                    ui.Label(label, width=230)
                    f = kind()
                    key = attr.split(":", 1)[1]
                    value = root.GetAttribute(attr).Get() if root and root.HasAttribute(attr) else husd.PLANT_DEFAULTS[key]
                    f.model.set_value(value)
                    self._run_fields[attr] = (f, type(value))
            with ui.HStack(height=28, spacing=4):
                ui.Button("Run DES", clicked_fn=lambda: self._do_run())
                ui.Button("Run 4 weeks", tooltip="672 hours, one replication",
                          clicked_fn=lambda: self._do_run(duration_h=672.0, replications=1))
            with ui.HStack(height=24, spacing=4):
                ui.Button("Pause", clicked_fn=self.s.pause)
                ui.Button("Next event", clicked_fn=self.s.step_des)
                ui.Button("Resume", clicked_fn=lambda: self.s.player and self.s.play(self.s.player.speed))
                ui.Button("Stop view", clicked_fn=self.s.stop)
            with ui.HStack(height=24, spacing=4):
                ui.Label("Events / s", width=90)
                for speed in (1, 4, 20, 100):
                    ui.Button(str(speed), clicked_fn=lambda v=speed: self.s.player and self.s.play(v))

    # ---- 4. run 3D (PhysX) -----------------------------------------------------------
    def _build_3d(self) -> None:
        if self._no_stage():
            return
        with ui.VStack(spacing=3, height=0):
            ui.Label("Physics, kinematics and motion profiles on the stage. Separate from the DES; its measured "
                     "times will become tables the DES can reuse.", height=0, style={"color": GREY}, word_wrap=True)
            with ui.HStack(height=28, spacing=4):
                ui.Button("Play", clicked_fn=self._play_3d)
                ui.Button("Pause", clicked_fn=self.s.pause_3d)
                ui.Button("Stop", clicked_fn=self._stop_3d)
            with ui.HStack(height=24, spacing=4):
                ui.Label("Fast forward", width=90)
                for speed in SPEEDS_3D:
                    ui.Button(f"x{speed}", style={"color": GREEN} if speed == self.s.speed_3d else {},
                              clicked_fn=lambda v=speed: self._speed_3d(v))
            with ui.HStack(height=24, spacing=4):
                ui.Button("All 3D", clicked_fn=lambda: self.s.set_3d(True))
                ui.Button("All 2D black boxes", clicked_fn=lambda: self.s.set_3d(False))
            ui.Button("Toggle Equipment Status", height=24, tooltip="Show or hide every floor status rectangle",
                      clicked_fn=self._toggle_status)

    def _save_run_fields(self) -> None:
        root = husd.plant(self.s.stage)
        for attr, (field, kind) in self._run_fields.items():
            m = field.model
            value = m.get_value_as_string() if kind is str else m.get_value_as_int() if kind is int else m.get_value_as_float()
            husd._set(root, attr, value)

    def _toggle_status(self) -> None:
        """Hide every floor status rectangle if any is showing, otherwise show them all."""
        from pxr import Usd

        rects = [UsdGeom.Imageable(q) for p in husd.equipment_prims(self.s.stage)
                 for q in Usd.PrimRange(p.GetPrimAtPath("Geo")) if q.GetName() == "Status"]
        show = not any(r.ComputeVisibility() != UsdGeom.Tokens.invisible for r in rects)
        for r in rects:
            r.MakeVisible() if show else r.MakeInvisible()

    def _do_run(self, **overrides) -> None:
        self._save_run_fields()
        try:
            self.s.run(**overrides)
        except Exception as e:  # noqa: BLE001  (a config error should be readable in the window)
            self.s.status = f"Cannot run: {e}"
            self.refresh()

    def _speed_3d(self, speed: int) -> None:
        self.s.set_speed_3d(speed)
        self.s.status = f"3D fast forward x{speed}."
        self.refresh()

    def _play_3d(self) -> None:
        try:
            self.s.play_3d()
            self.s.status = "3D simulation playing (PhysX)."
        except Exception as e:  # noqa: BLE001
            self.s.status = f"Cannot play 3D: {e}"
        self.refresh()

    def _stop_3d(self) -> None:
        self.s.stop_3d()
        self.s.status = "3D simulation stopped and reset."
        self.refresh()

    # ---- 6. bottleneck insights (DES or 3D) ------------------------------------------
    def _build_kpis(self) -> None:
        if self._no_stage():
            return
        ins = self.s.insights()
        with ui.VStack(spacing=2, height=0):
            if ins is None:
                ui.Label("Run the DES or play 3D to see where the line is held up.", height=20, word_wrap=True)
                return
            ui.Label(f"From {ins['mode']} mode:  {ins['throughput']:.1f} parts/h", height=20)
            for n, line in enumerate(ins["lines"]):
                ui.Label(line, height=0, word_wrap=True, style={"color": RED} if n == 0 else {})
            with ui.HStack(height=18):
                for text, width in (("station", 110), ("work", 50), ("blocked", 60), ("starved", 60), ("down", 50)):
                    ui.Label(text, width=width, style={"color": GREY})
            for sid in ins["ranking"]:
                st = ins["stations"][sid]
                with ui.HStack(height=18):
                    ui.Label(sid, width=110, style={"color": RED} if sid == ins["bottleneck"] else {})
                    for key, width in (("utilization", 50), ("blocked", 60), ("starved", 60), ("down", 50)):
                        ui.Label(f"{100 * st.get(key, 0.0):.0f}%", width=width)
            ui.Label(ins["detail"], height=18, style={"color": GREY}, word_wrap=True)
            if ins["mode"] == "DES":
                m = self.s.results["meta"]
                ui.Label(f"config {m['config_hash']}  seed {m['seed']}  engine {m['engine_version']}",
                         height=18, style={"color": GREY}, word_wrap=True)
