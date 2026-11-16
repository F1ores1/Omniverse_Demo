"""USD layer: a stage built from the catalog round-trips into a scenario the DES can run.

Needs `pxr`. Skipped in a plain Python environment; run with Isaac Sim's Python:
    C:\\isaacsim\\env\\Scripts\\python.exe -m pytest tests/unit/test_usd.py
"""
from __future__ import annotations

from math import hypot

import pytest

pytest.importorskip("pxr")

from pxr import Gf, Usd, UsdGeom  # noqa: E402

from simcore import usd as husd  # noqa: E402
from simcore.catalog import CATALOG, parse_dist  # noqa: E402
from simcore.config.schema import Scenario  # noqa: E402
from simcore.experiment import simulate  # noqa: E402
from simcore.kinematics import boxes_overlap, pick_place_time, segment_hits_box  # noqa: E402


def cell() -> Usd.Stage:
    stage = Usd.Stage.CreateInMemory()
    husd.new_plant(stage, "demo")
    husd.add_equipment(stage, "conveyor", (0, 0, 0), station_id="infeed")
    husd.add_equipment(stage, "robot", (2.2, -0.9, 0), station_id="robot")
    husd.add_equipment(stage, "cnc", (3.4, 0.6, 0), station_id="cnc", params={"cycle_time": "normal(60, 3)"})
    husd.add_equipment(stage, "transfer", (5.6, 0, 0), station_id="gantry")
    husd.add_equipment(stage, "inspection", (7.6, 0, 0), station_id="inspect")
    return stage


def test_every_catalog_item_builds_with_ports_and_attributes():
    stage = Usd.Stage.CreateInMemory()
    for i, name in enumerate(CATALOG):
        prim = husd.add_equipment(stage, name, (i * 4.0, 0, 0))
        assert prim.GetAttribute("factory:equipmentType").Get() == name
        assert prim.GetAttribute("factory:configRef").Get() == f"cell.stations.{prim.GetName()}"
        assert prim.GetPrimAtPath("Geo/Status").IsValid()
        assert prim.GetPrimAtPath("Ports/in_raised").IsValid() != CATALOG[name].transfer


def test_resizing_is_parametric():
    stage = Usd.Stage.CreateInMemory()
    prim = husd.add_equipment(stage, "cnc")
    prim.GetAttribute("factory:size").Set(Gf.Vec3d(4.0, 3.0, 2.5))
    husd.rebuild(prim)
    box = husd.footprint(prim)
    assert box.hi[0] - box.lo[0] == pytest.approx(4.0, abs=0.05)
    assert box.hi[2] == pytest.approx(2.5, abs=0.01)  # roof


def test_stage_becomes_a_scenario_that_runs():
    stage = cell()
    raw = husd.stage_to_scenario(stage)
    assert raw["part_types"][0]["routing"] == ["infeed", "robot", "cnc", "gantry", "inspect"]  # layout order along +X
    scn = Scenario.model_validate(raw)
    by_id = {s.id: s for s in scn.cell.stations}
    assert by_id["infeed"].params["transit_time"] == pytest.approx(3.0 / 0.2)  # belt length / speed
    assert by_id["cnc"].params["cycle_time"] == {"dist": "normal", "mean": 60.0, "sd": 3.0}
    assert by_id["gantry"].params["travel"] == pytest.approx(hypot(7.6 - 3.4, 0.6))  # CNC table -> CMM table
    res = simulate(scn.model_copy(update={"run": scn.run.model_copy(update={"replications": 1})}))
    assert res["bottleneck"] == "cnc"
    assert res["kpis"]["throughput_per_h"]["mean"] == pytest.approx(60.0, rel=0.03)


def test_moving_equipment_changes_the_gantry_time():
    stage = cell()
    near = Scenario.model_validate(husd.stage_to_scenario(stage)).cell.stations[3].params
    UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/Plant/inspect")).SetTranslate(Gf.Vec3d(9.6, 0, 0))
    far = Scenario.model_validate(husd.stage_to_scenario(stage)).cell.stations[3].params
    t = lambda p: pick_place_time(p["travel"], p["lift"], p["v_max"], p["a_max"], p["grip_time"])  # noqa: E731
    assert far["travel"] == pytest.approx(hypot(9.6 - 3.4, 0.6))
    assert t(far) > t(near)


def test_explicit_flow_with_parallel_stations():
    stage = cell()
    husd.add_equipment(stage, "cnc", (3.4, 3.0, 0), station_id="cnc_b", params={"cycle_time": "60"})
    husd.set_flow(stage, ["infeed", "robot", ["cnc", "cnc_b"], "gantry", "inspect"])
    raw = husd.stage_to_scenario(stage)
    assert raw["part_types"][0]["routing"][2] == ["cnc", "cnc_b"]
    assert len(husd.transfer_points(stage, husd.get_flow(stage), "robot")) == 2  # one pick, two place points
    Scenario.model_validate(raw)
    stage.RemovePrim("/Plant/cnc_b")  # deleting a station heals the flow
    assert husd.get_flow(stage)[2] == "cnc"


def test_overlap_and_tool_path_checks_use_the_stage_geometry():
    stage = cell()
    cnc, robot = stage.GetPrimAtPath("/Plant/cnc"), stage.GetPrimAtPath("/Plant/robot")
    assert not boxes_overlap(husd.footprint(cnc), husd.footprint(robot))
    clash = husd.add_equipment(stage, "buffer", (3.4, 0.6, 0))
    assert boxes_overlap(husd.footprint(cnc), husd.footprint(clash))
    table = tuple(husd.world_port(cnc, "in_raised"))
    front, behind = (table[0], table[1] - 2.0, table[2]), (table[0], table[1] + 2.0, table[2])
    hits = lambda a, b: any(segment_hits_box(a, b, box) for box in husd.obstacles(cnc))  # noqa: E731
    assert not hits(front, table)  # loading through the open front is clear
    assert hits(behind, table)     # reaching through the back wall is flagged


def test_state_colors_the_floor_status():
    stage = cell()
    prim = stage.GetPrimAtPath("/Plant/cnc")
    husd.set_state(prim, "DOWN")
    assert UsdGeom.Gprim(prim.GetPrimAtPath("Geo/Status")).GetDisplayColorAttr().Get()[0] == Gf.Vec3f(*husd.STATE_COLORS["IDLE"])


def test_dist_text():
    assert parse_dist("20") == 20.0
    assert parse_dist("tri(20, 25, 35)") == {"dist": "triangular", "low": 20.0, "mode": 25.0, "high": 35.0}
    with pytest.raises(ValueError):
        parse_dist("normal(5)")
