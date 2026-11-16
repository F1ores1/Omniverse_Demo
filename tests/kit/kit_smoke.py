"""In-app smoke test, meant to be watched.  Run:  .\\scripts\\run_composer.ps1 -Smoke

Drives the same Session API the window uses, slowly enough to follow by eye. Each check prints a
[SMOKE] PASS/FAIL line in the terminal and in the window's status line. The app stays open at the
end so the result can be inspected. Results also go to output/kit/kit_smoke.json.
"""
import asyncio
import json
import traceback
from pathlib import Path

import carb
import omni.kit.app
import omni.usd

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "output" / "kit"
checks: list[dict] = []


async def frames(n: int) -> None:
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


def say(session, text: str) -> None:
    carb.log_warn(f"[SMOKE] {text}")
    print(f"[SMOKE] {text}", flush=True)
    if session:
        session.status = f"SMOKE TEST: {text}"
        session.on_change()


def check(session, name: str, ok: bool, detail: str = "") -> None:
    checks.append({"check": name, "pass": bool(ok), "detail": detail})
    say(session, f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")


async def look(session) -> None:
    from omni.kit.viewport.utility import frame_viewport_prims, get_active_viewport

    frame_viewport_prims(get_active_viewport(), prims=["/Plant"])
    await frames(45)


async def main() -> None:
    s = None
    await frames(120)
    try:
        import factory_workcell
        from pxr import Gf, UsdGeom

        from simcore import usd as husd

        s = factory_workcell.get_session()
        await omni.usd.get_context().new_stage_async()
        await frames(10)
        husd.new_plant(s.stage, "smoke")

        say(s, "Step 1/7: building a line from the catalog, one item every second")
        for type_name, params in (("conveyor", {}), ("robot", {}), ("cnc", {"cycle_time": "normal(60, 3)"}),
                                  ("robot", {}), ("inspection", {}), ("buffer", {}), ("gantry", {}), ("conveyor", {})):
            s.add(type_name, params=params)
            await look(s)
        flow = husd.get_flow(s.stage)
        check(s, "catalog items placed in flow order", flow == [
            "conveyor_1", "robot_1", "cnc_1", "robot_2", "inspection_1", "buffer_1", "gantry_1", "conveyor_2"], str(flow))
        check(s, "default layout has no overlap, reach or path problems", not s.problems, "; ".join(s.problems))
        await frames(90)

        say(s, "Step 2/7: running the DES (4 h x 2 replications)")
        r = s.run(duration_h=4.0, replications=2)
        tp = r["kpis"]["throughput_per_h"]["mean"]
        check(s, "DES ran from the written YAML", s.scenario_path.exists() and tp > 0,
              f"{tp:.1f} parts/h, bottleneck {r['bottleneck']}, config {r['meta']['config_hash']}")
        check(s, "bottleneck is the CNC (60 s cycle)", r["bottleneck"] == "cnc_1" and abs(tp - 60) < 4, f"{tp:.1f} parts/h")
        await frames(120)

        say(s, "Step 3/7: 3D playback at x10 - orange loads should move along the line, floor rectangles change color")
        s.play(speed=10.0)
        await frames(480)
        loads = s.stage.GetPrimAtPath("/Plant/Loads").GetChildren()
        visible = sum(UsdGeom.Imageable(p).ComputeVisibility() != "invisible" for p in loads)
        states = {sid: s.player.track.station_state(sid, s.player.t) for sid in s.player.prims}
        check(s, "loads are on stage during playback", visible > 0, f"{visible} loads at t={s.player.t:.0f} s")
        check(s, "CNC floor status shows PROCESSING", states["cnc_1"] == "PROCESSING", str(states))
        say(s, "Fast forward x60")
        t_before = s.player.t
        s.play(speed=60.0)
        await frames(300)
        check(s, "fast forward advances the clock", s.player.t > t_before + 150, f"t={s.player.t:.0f} s (was {t_before:.0f})")
        s.pause()
        try:
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

            await look(s)
            capture_viewport_to_file(get_active_viewport(), str(OUT / "kit_smoke.png"))
            await frames(30)
        except Exception:  # noqa: BLE001
            say(s, "screenshot failed: " + traceback.format_exc(limit=1))
        s.stage.GetRootLayer().Export(str(OUT / "kit_smoke.usda"))
        s.stop()
        await frames(60)

        say(s, "Step 4/7: SnapMode - a conveyor dropped 0.3 m off the end should jump onto the line")
        line_end = max(husd.footprint(p).hi[0] for p in s.equipment() if not husd.item_of(p).transfer)
        extra = s.add("conveyor", position=(line_end + 1.5 + 0.3, 0.25, 0.0))
        await look(s)
        await frames(90)
        s.settle(extra)
        gap = husd.footprint(extra).lo[0] - line_end
        check(s, "snap closes the gap between conveyors", abs(gap) < 1e-3, f"gap {gap:.4f} m; {s.status}")
        await frames(120)

        say(s, "Step 5/7: dragging that conveyor into the CNC should be refused and undone")
        before = tuple(husd.world_pos(extra))
        UsdGeom.XformCommonAPI(extra).SetTranslate(Gf.Vec3d(*husd.world_pos(s.stage.GetPrimAtPath("/Plant/cnc_1"))))
        await frames(120)  # no explicit settle: the stage-change listener must catch this on its own
        check(s, "overlapping move is reverted", tuple(husd.world_pos(extra)) == before and "refused" in s.status, s.status)
        await frames(120)

        say(s, "Step 6/7: a gantry loading a CNC through its roof should be flagged in '6. Checks'")
        cnc = s.add("cnc")
        s.add("gantry")
        s.add("buffer")
        await look(s)
        flagged = [p for p in s.check() if "passes through" in p]
        check(s, "gantry through the CNC roof is flagged, and nothing else",
              len(flagged) == 1 and "gantry_2" in flagged[0] and "cnc_2" in flagged[0], "; ".join(flagged))
        await frames(180)

        say(s, "Step 7/7: switching every item to a 2D black box, then running again")
        s.set_3d(False)
        await look(s)
        tiles = [c.GetName() for c in cnc.GetPrimAtPath("Geo").GetChildren()]
        tp2 = s.run(duration_h=4.0, replications=2)["kpis"]["throughput_per_h"]["mean"]
        check(s, "2D black boxes still simulate", "Tile" in tiles and tp2 > 0, f"{tp2:.1f} parts/h")
        await frames(120)
        s.set_3d(True)
        await look(s)
    except Exception:  # noqa: BLE001
        check(s, "smoke test ran to the end", False, traceback.format_exc())
    failed = [c for c in checks if not c["pass"]]
    (OUT / "kit_smoke.json").write_text(json.dumps(checks, indent=1), encoding="utf-8")
    say(s, f"RESULT: {'ALL PASS' if not failed else 'FAILED'} - {len(checks) - len(failed)}/{len(checks)} checks passed. "
           "The app stays open; close it when you are done.")


asyncio.ensure_future(main())
