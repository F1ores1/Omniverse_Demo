"""Kit extension entry point. Omniverse is a client of the headless core, never the other way round."""
from __future__ import annotations

import sys
from pathlib import Path

import carb
import omni.ext
import omni.kit.app

_session = None
TEMPLATE = "Modular Work Cell"


def _new_plant_stage(rootname: str, context_name: str = "") -> None:
    """New-file template: start in Z-up metres with the plant, floor and lights already there. Because the units are
    right from the start, adding equipment never switches them (no 'Stage units changed' warning, no camera jump)."""
    import omni.usd
    from pxr import UsdGeom

    from simcore import usd as husd

    stage = omni.usd.get_context(context_name).get_stage()
    # Composer's units display warns when a non-empty stage changes units, and it already sees /World here. Forget its
    # remembered units for this brand-new file so it just picks up metres. Best effort: if this fails, the notice shows.
    try:
        import gc

        from omni.metrics.assembler.ui.metricsAssemblerUnitsView import MetricsAssemblerUnitsView

        for view in (o for o in gc.get_objects() if isinstance(o, MetricsAssemblerUnitsView)):
            view._last_known_mpu = None
    except Exception:  # noqa: BLE001
        pass
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    husd.new_plant(stage)
    import asyncio

    asyncio.ensure_future(_camera_to_metres())


async def _camera_to_metres() -> None:
    """Kit places the new file's camera for centimetres (hundreds of units out). Put it ~10 m from the origin."""
    from omni.kit.viewport.utility import get_active_viewport
    from omni.kit.viewport.utility.camera_state import ViewportCameraState
    from pxr import Gf, UsdGeom

    for _ in range(3):  # let the viewport pick up the new stage first
        await omni.kit.app.get_app().next_update_async()
    viewport = get_active_viewport()
    if viewport is None:
        return
    cam = ViewportCameraState(viewport.camera_path, viewport)
    cam.set_position_world(Gf.Vec3d(7.0, -7.0, 5.0), True)
    cam.set_target_world(Gf.Vec3d(0.0, 0.0, 0.5), True)
    prim = viewport.stage.GetPrimAtPath(viewport.camera_path) if viewport.stage else None
    if prim and prim.IsA(UsdGeom.Camera):
        UsdGeom.Camera(prim).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))  # metres: 1 cm to 10 km


def get_session():
    """The live session, for scripts and tests: `factory_workcell.get_session()`."""
    return _session


class FactoryWorkcellExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str) -> None:
        global _session
        ext_path = Path(omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id))
        repo = ext_path.resolve().parents[1]  # <repo>/omniverse/factory.workcell
        extra = carb.settings.get_settings().get("/exts/factory.workcell/isaac_site_packages") or ""
        for path in (str(repo), extra):
            if path and path not in sys.path:
                sys.path.append(path)
        carb.settings.get_settings().set("/persistent/app/viewport/pickingMode", "kind:model.ALL")

        from .session import Session
        from .window import WorkcellWindow

        _session = Session(repo)
        self._window = WorkcellWindow(_session)
        try:  # File > New starts from our template (Composer's own templates stay in the File > New From Stage Template menu)
            from omni.kit.stage_template.core import register_template

            register_template(TEMPLATE, _new_plant_stage)
            carb.settings.get_settings().set("/persistent/app/newStage/defaultTemplate", TEMPLATE)
        except ImportError:
            pass

    def on_shutdown(self) -> None:
        global _session
        try:
            from omni.kit.stage_template.core import unregister_template

            unregister_template(TEMPLATE)
        except ImportError:
            pass
        if _session:
            _session.shutdown()
        self._window.destroy()
        self._window = _session = None
