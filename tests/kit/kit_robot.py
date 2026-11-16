"""In-app UR10 test, meant to be watched.  Run:  .\\scripts\\run_composer.ps1 -Exec tests\\kit\\kit_robot.py

Builds conveyor -> UR10 -> CNC -> UR10 -> CMM -> conveyor, solves the robot moves with cuMotion,
runs the DES on the solved move times, and plays it back slowly. Prints [ROBOT] PASS/FAIL lines.
The app stays open. Results also go to output/kit/kit_robot.json.
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
    carb.log_warn(f"[ROBOT] {text}")
    print(f"[ROBOT] {text}", flush=True)
    if session:
        session.status = f"ROBOT TEST: {text}"
        session.on_change()


def check(session, name: str, ok: bool, detail: str = "") -> None:
    checks.append({"check": name, "pass": bool(ok), "detail": detail})
    say(session, f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")


async def look(prims) -> None:
    from omni.kit.viewport.utility import frame_viewport_prims, get_active_viewport

    frame_viewport_prims(get_active_viewport(), prims=prims)
    await frames(45)


async def close_up(eye, target) -> None:
    """Fixed camera, because framing on the arm asset's bounds zooms far out."""
    from omni.kit.viewport.utility import get_active_viewport
    from omni.kit.viewport.utility.camera_state import ViewportCameraState
    from pxr import Gf

    cam = ViewportCameraState(get_active_viewport().camera_path, get_active_viewport())
    cam.set_position_world(Gf.Vec3d(*eye), True)
    cam.set_target_world(Gf.Vec3d(*target), True)
    await frames(45)


async def main() -> None:
    s = None
    await frames(120)
    try:
        import factory_workcell
        import numpy as np
        import yaml

        from simcore import usd as husd

        s = factory_workcell.get_session()
        await omni.usd.get_context().new_stage_async()
        await frames(10)
        husd.new_plant(s.stage, "robot_cell")

        say(s, "Step 1/5: building conveyor -> UR10 -> CNC -> UR10 -> CMM -> conveyor (first robot loads cuMotion, ~10 s)")
        for type_name, params in (("conveyor", {}), ("robot", {}), ("cnc", {"cycle_time": "20"}),
                                  ("robot", {}), ("inspection", {"cycle_time": "10"}), ("conveyor", {})):
            s.add(type_name, params=params)
            await look(["/Plant"])
        await frames(240)  # let the UR10 asset finish loading
        say(s, "Step 2/5: solving robot moves with cuMotion - both arms should stand on their pedestals, tool above the pick point")
        table = s.solve_robots()
        s.check()
        base = husd.world_pos(s.stage.GetPrimAtPath("/Plant/robot_1"))
        view = ((base[0] + 0.6, base[1] - 4.0, 2.6), (base[0] + 0.3, base[1] + 0.5, 1.0))
        await close_up(*view)
        arm = s.arms.get("robot_1")
        from pxr import Gf, Usd, UsdGeom
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        far = {}
        for link in arm._links.values() if arm else ():
            box = cache.ComputeWorldBound(link.GetPrim()).ComputeAlignedRange()
            if not box.IsEmpty():
                far[link.GetPrim().GetName()] = round(max((box.GetMax() - box.GetMin())[i] for i in range(3)), 2)
        check(s, "every arm link is link-sized (under 1.5 m)", bool(far) and max(far.values()) < 1.5, json.dumps(far))
        links = sorted(arm._links) if arm else []
        check(s, "UR10 asset loaded with link prims matching the URDF", len(links) >= 6, ", ".join(links))
        check(s, "cuMotion solved every move", bool(table) and set(table["stations"]) == {"robot_1", "robot_2"}
              and not s.robot_errors, "; ".join(s.robot_errors) or json.dumps(table["stations"]) if table else "no table")
        move = arm.moves.get(("conveyor_1", "cnc_1"))
        if move is None:
            raise RuntimeError("robot_1 has no solved move conveyor_1 -> cnc_1, stopping here")
        check(s, "trajectory touches pick before place", 0 < move.t_pick < move.t_place < move.duration,
              f"motion {move.duration:.2f} s, pick at {move.t_pick:.2f} s, place at {move.t_place:.2f} s, cycle {move.cycle_s:.2f} s")
        pick = np.array(husd.world_port(s.stage.GetPrimAtPath("/Plant/conveyor_1"), "out_lowered")) + [0, 0, 0.16]
        miss = float(np.linalg.norm(np.array(arm.tool_world(move.q(move.t_pick))) - pick))
        check(s, "tool reaches the pick point", miss < 0.02, f"{1000 * miss:.1f} mm off")
        # The drawn arm must follow the solved one: wrist mesh at the tool, forearm mesh between elbow and wrist.
        q_pick = move.q(move.t_pick)
        arm.pose(q_pick)
        await frames(5)
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        centre = lambda n: np.array(cache.ComputeWorldBound(arm._links[n].GetPrim()).ComputeAlignedRange().GetMidpoint())  # noqa: E731
        frame = lambda n: np.array(arm.base.Transform(Gf.Vec3d(*[float(v) for v in arm.kin.position(q_pick, n)])))  # noqa: E731
        gaps = {"wrist_3_link to tool": float(np.linalg.norm(centre("wrist_3_link") - np.array(arm.tool_world(q_pick)))),
                "forearm_link to elbow-wrist midpoint": float(np.linalg.norm(
                    centre("forearm_link") - (frame("forearm_link") + frame("wrist_1_link")) / 2)),
                "upper_arm_link to shoulder-elbow midpoint": float(np.linalg.norm(
                    centre("upper_arm_link") - (frame("upper_arm_link") + frame("forearm_link")) / 2))}
        check(s, "drawn arm links are connected and follow the solved pose", max(gaps.values()) < 0.2,
              ", ".join(f"{k} {1000 * v:.0f} mm" for k, v in gaps.items()))
        await frames(180)

        say(s, "Step 3/5: running the DES on the solved move times")
        r = s.run(duration_h=2.0, replications=1)
        scn = yaml.safe_load(s.scenario_path.read_text(encoding="utf-8"))
        used = s._scn.cell.stations[1].params["handling_time"]
        check(s, "DES uses the cuMotion cycle as the robot handling time",
              scn.get("move_table") == f"tables/{s.scenario_path.stem}.yaml" and abs(used - table["stations"]["robot_1"]["cycle_s"]) < 1e-6,
              f"robot_1 handling_time {used} s, {r['kpis']['throughput_per_h']['mean']:.1f} parts/h, bottleneck {r['bottleneck']}")
        await frames(120)

        say(s, "Step 4/5: playback at x1 - watch robot_1 pick from the conveyor end and place on the CNC table")
        s.play(speed=1.0)
        await frames(900)
        say(s, "Step 5/5: playback at x5")
        s.play(speed=5.0)
        await frames(900)
        s.pause()
        try:
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

            await close_up(*view)
            capture_viewport_to_file(get_active_viewport(), str(OUT / "kit_robot.png"))
            await frames(30)
        except Exception:  # noqa: BLE001
            say(s, "screenshot failed")
        check(s, "playback ran with the arms", s.player.t > 60, f"t={s.player.t:.0f} s")
    except Exception:  # noqa: BLE001
        check(s, "robot test ran to the end", False, traceback.format_exc())
    failed = [c for c in checks if not c["pass"]]
    (OUT / "kit_robot.json").write_text(json.dumps(checks, indent=1), encoding="utf-8")
    say(s, f"RESULT: {'ALL PASS' if not failed else 'FAILED'} - {len(checks) - len(failed)}/{len(checks)} checks passed. "
           "Playback is paused; the app stays open.")


asyncio.ensure_future(main())
