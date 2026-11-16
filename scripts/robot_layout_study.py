"""Where should each robot pedestal stand, and what are its poses?  Headless: no Kit.

Run with Isaac Sim's Python (it has the cuMotion wheel):
    C:\\isaacsim\\env\\Scripts\\python.exe scripts\\robot_layout_study.py [output\\kit\\layout_dump.json]

Input: a layout dump from Composer (every machine's collider boxes and pick/place nodes).
For each robot (between the item before and after it in the flow) it sweeps the pedestal position and solves the
loop with simcore.kinematics.ur10_loop, the same solver the Kit extension uses: cuMotion IK + time-optimal joint
trajectories, the arm's link tubes (and the load in the air) checked for clearance against every machine box.

Output: output/reports/robot_study.json: every candidate's clearance, closest box and cycle time; for the best one
per robot, the joint values and link points at each waypoint.

    --check   no sweep: score each robot where it stands, for every move it makes (all source x destination pairs of
              a parallel step). Output: output/reports/robot_check.json.
"""
from __future__ import annotations

import importlib.util
import json
import os
import pathlib
import sys
import time

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
PB = r"C:\isaacsim\env\Lib\site-packages\isaacsim\exts\isaacsim.robot_motion.cumotion\pip_prebundle"
sys.path.insert(0, PB)
os.add_dll_directory(PB)
import cumotion  # noqa: E402

from simcore.kinematics.ur10_loop import NAMES, NEED, LoopSolver  # noqa: E402

PEDESTAL = (0.4, 0.4, 0.9)  # catalog robot size: the arm's base sits on top


def load_ur10():
    spec = importlib.util.spec_from_file_location("norm", PB + r"\isaacsim\robot_motion\cumotion\impl\urdf_normalize.py")
    norm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(norm)
    d = pathlib.Path(PB) / "isaacsim/robot_motion/cumotion/robot_configurations/ur10"
    desc = cumotion.load_robot_from_memory((d / "robot.xrdf").read_text(),
                                           norm.normalize_urdf_for_urdfdom((d / "robot.urdf").read_text()))
    return desc.kinematics(), desc.tool_frame_names()[0]


def check(dump_path: pathlib.Path) -> None:
    """Every robot where it stands, every move it makes."""
    kin, tool = load_ur10()
    solver = LoopSolver(cumotion, kin, tool)
    dump = json.loads(dump_path.read_text())
    items = {it["id"]: it for it in dump["items"]}
    steps = [s if isinstance(s, list) else [s] for s in dump["flow"]]
    boxes = [(it["id"], b["name"], b["lo"], b["hi"]) for it in dump["items"] for b in it["boxes"]]
    boxes.append(("floor", "floor", [-1e3, -1e3, -1.0], [1e3, 1e3, 0.0]))
    out, all_ok = {}, True
    for i, step in enumerate(steps):
        sid = step[0]
        if items[sid]["type"] != "robot":
            continue
        keep = [k for k, b in enumerate(boxes) if b[0] != sid]  # not its own pedestal
        lo, hi = np.array([boxes[k][2] for k in keep]), np.array([boxes[k][3] for k in keep])
        base = np.array(items[sid]["pos"]) + np.array([0, 0, PEDESTAL[2]])
        for a in steps[i - 1]:
            for b in steps[i + 1]:
                pick, place = np.array(items[a]["ports"]["out_lowered"]), np.array(items[b]["ports"]["in_lowered"])
                reach = [round(float(np.linalg.norm((n - base)[:2])), 3) for n in (pick, place)]
                loop = solver.solve(base, pick, place, lo, hi)
                key = f"{sid} {a}->{b}"
                if loop is None:
                    out[key] = {"result": "unreachable", "reach_m": reach}
                    all_ok = False
                else:
                    out[key] = {"ok": loop.ok, "clearance": round(loop.clearance, 3),
                                "closest": "/".join(boxes[keep[loop.closest_box]][:2]),
                                "worst_leg": f"{NAMES[loop.worst_leg]} -> {NAMES[loop.worst_leg + 1]}",
                                "cycle_s": round(loop.cycle_s, 2), "approach_up": loop.approach_up,
                                "wait_back_m": round(loop.wait_back, 2), "reach_m": reach}
                    all_ok &= loop.ok
                print(key, out[key], flush=True)
    path = REPO / "output" / "reports" / "robot_check.json"
    path.write_text(json.dumps(out, indent=1))
    print("ALL CLEAR" if all_ok else "NOT ALL CLEAR", "->", path)


def main(dump_path: pathlib.Path) -> None:
    kin, tool = load_ur10()
    solver = LoopSolver(cumotion, kin, tool)
    dump = json.loads(dump_path.read_text())
    items = {it["id"]: it for it in dump["items"]}
    flow = [s for s in dump["flow"] if isinstance(s, str)]
    boxes = [(it["id"], b["name"], b["lo"], b["hi"]) for it in dump["items"] if it["type"] != "robot" for b in it["boxes"]]
    boxes.append(("floor", "floor", [-1e3, -1e3, -1.0], [1e3, 1e3, 0.0]))
    lo, hi = np.array([b[2] for b in boxes]), np.array([b[3] for b in boxes])
    out = {"note": __doc__.split("\n")[0], "need_clearance_m": NEED, "robots": {}}
    t0 = time.time()
    for i, sid in enumerate(flow):
        if items[sid]["type"] != "robot":
            continue
        src, dst = items[flow[i - 1]], items[flow[i + 1]]
        pick, place = np.array(src["ports"]["out_lowered"]), np.array(dst["ports"]["in_lowered"])
        mid_x = (pick[0] + place[0]) / 2
        gap_x = (max(b[3][0] for b in boxes if b[0] == src["id"]) + min(b[2][0] for b in boxes if b[0] == dst["id"])) / 2
        cands = []
        for x in sorted({round(mid_x + d, 2) for d in np.arange(-0.4, 0.401, 0.1)} | {round(gap_x, 2)}):
            for y in np.arange(-0.5, -1.501, -0.1):
                ped_lo = np.array([x - PEDESTAL[0] / 2, y - PEDESTAL[1] / 2, 0.0])
                ped_hi = np.array([x + PEDESTAL[0] / 2, y + PEDESTAL[1] / 2, PEDESTAL[2]])
                if np.any(np.all((ped_lo[None] < hi - 1e-6) & (ped_hi[None] > lo + 1e-6), axis=1)[:-1]):
                    continue  # pedestal stands inside a machine
                base = np.array([x, y, PEDESTAL[2]])
                loop = solver.solve(base, pick, place, lo, hi)
                row = {"x": x, "y": round(float(y), 2), "dx_from_node_mid": round(x - mid_x, 2),
                       "dx_from_gap_centre": round(x - gap_x, 2)}
                if loop is None:
                    row["result"] = "unreachable"
                else:
                    row.update({"clearance": round(loop.clearance, 3), "closest": "/".join(boxes[loop.closest_box][:2]),
                                "worst_leg": f"{NAMES[loop.worst_leg]} -> {NAMES[loop.worst_leg + 1]}",
                                "cycle_s": round(loop.cycle_s, 3), "approach_up": loop.approach_up,
                                "wait_back_m": round(loop.wait_back, 2), "_loop": loop, "_base": base})
                cands.append(row)
        ok = [c for c in cands if "_loop" in c and c["_loop"].ok]
        best = max(ok, key=lambda c: (min(c["clearance"], 0.3), -c["cycle_s"])) if ok else None
        if best:  # joints and link points at each waypoint of the chosen loop
            loop, base = best["_loop"], best["_base"]
            best["poses"] = {n: {"joints_deg": np.degrees(q).round(1).tolist(),
                                 "links": (solver.body.points(q) + base).round(3).tolist()}
                             for n, q in zip(NAMES, loop.waypoints)}
        for c in cands:
            c.pop("_loop", None)
            c.pop("_base", None)
        out["robots"][sid] = {"from": src["id"], "to": dst["id"], "pick": pick.tolist(), "place": place.tolist(),
                              "current": items[sid]["pos"], "gap_centre_x": gap_x, "best": best,
                              "candidates": sorted(cands, key=lambda c: -c.get("clearance", -9))}
        b = best or {}
        print(f"{sid} {src['id']}->{dst['id']}: {len(ok)}/{len(cands)} clear; best x={b.get('x')} y={b.get('y')} "
              f"(gap centre {b.get('dx_from_gap_centre')}) clearance={b.get('clearance')} m cycle={b.get('cycle_s')} s "
              f"closest={b.get('closest')} worst leg={b.get('worst_leg')}  [{time.time() - t0:.0f}s]", flush=True)
    path = REPO / "output" / "reports" / "robot_study.json"
    path.write_text(json.dumps(out, indent=1))
    print("wrote", path)


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    dump = pathlib.Path(args[0]) if args else REPO / "output" / "kit" / "layout_dump.json"
    check(dump) if "--check" in sys.argv else main(dump)
