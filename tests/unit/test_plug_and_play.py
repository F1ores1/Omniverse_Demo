"""Plug-and-play features: parallel routing, gantry, buffer, move-time tables, plant roll-up, clearance."""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from simcore.config import load_scenario
from simcore.config.schema import Scenario
from simcore.experiment import simulate
from simcore.kinematics import Box, boxes_overlap, pick_place_time, segment_hits_box, trapezoid_pos, trapezoid_time
from simcore.plant import rollup, simulate_plant

EXP = lambda mean: {"dist": "exp", "mean": mean}  # noqa: E731


def scenario(stations, routing, interarrival=None, duration_h=20, warmup_h=1, reps=1) -> Scenario:
    return Scenario.model_validate({
        "name": "test",
        "cell": {"name": "c", "stations": stations},
        "part_types": [{"name": "p", "routing": routing}],
        "source": {"interarrival": interarrival},
        "run": {"duration_h": duration_h, "warmup_h": warmup_h, "seed": 11, "replications": reps},
    })


# ---- parallel routing ----------------------------------------------------------------
def test_parallel_stations_match_erlang_c():
    """One queue feeding two different stations in parallel is M/M/2: W = 2.778 s at lambda = 1.6, mu = 1."""
    scn = scenario(
        [{"id": "q", "type": "buffer", "buffer": None, "params": {"slots": 1}},
         {"id": "a", "type": "cnc", "params": {"cycle_time": EXP(1.0)}},
         {"id": "b", "type": "cnc", "params": {"cycle_time": EXP(1.0)}}],
        routing=["q", ["a", "b"]], interarrival=EXP(1 / 1.6), duration_h=40, warmup_h=2, reps=3,
    )
    res = simulate(scn)
    assert res["kpis"]["cycle_time_s"]["mean"] == pytest.approx(2.778, rel=0.05)
    assert (res["stations"]["a"]["utilization"] + res["stations"]["b"]["utilization"]) / 2 == pytest.approx(0.8, rel=0.02)


def test_parallel_stations_double_a_deterministic_bottleneck():
    scn = scenario(
        [{"id": "load", "type": "robot", "params": {"handling_time": 10}},
         {"id": "cut_a", "type": "cnc", "params": {"cycle_time": 60}},
         {"id": "cut_b", "type": "cnc", "params": {"cycle_time": 60}},
         {"id": "check", "type": "inspection", "params": {"cycle_time": 20}}],
        routing=["load", ["cut_a", "cut_b"], "check"],
    )
    res = simulate(scn)
    assert res["kpis"]["throughput_per_h"]["mean"] == pytest.approx(120.0, abs=0.2)
    assert res["stations"]["cut_a"]["utilization"] == pytest.approx(1.0, abs=1e-3)
    assert res["stations"]["cut_b"]["utilization"] == pytest.approx(1.0, abs=1e-3)


def test_first_step_cannot_be_parallel():
    with pytest.raises(ValueError, match="first step"):
        scenario([{"id": "a", "type": "cnc", "buffer": 1, "params": {"cycle_time": 1}},
                  {"id": "b", "type": "cnc", "buffer": 1, "params": {"cycle_time": 1}}], routing=[["a", "b"]])


# ---- gantry and trapezoid profile ----------------------------------------------------
def test_trapezoid_time_matches_closed_form():
    assert trapezoid_time(2.0, 1.0, 2.0) == pytest.approx(2.0 / 1.0 + 1.0 / 2.0)  # reaches v_max: d/v + v/a
    assert trapezoid_time(0.1, 1.0, 2.0) == pytest.approx(2 * (0.1 / 2.0) ** 0.5)  # triangular: 2*sqrt(d/a)
    assert trapezoid_time(0.0, 1.0, 2.0) == 0.0


@pytest.mark.parametrize("d", [0.1, 0.5, 2.0])
def test_trapezoid_position_is_continuous_and_ends_at_distance(d):
    total = trapezoid_time(d, 1.0, 2.0)
    xs = [trapezoid_pos(d, 1.0, 2.0, total * i / 200) for i in range(201)]
    assert xs[0] == 0.0 and xs[-1] == pytest.approx(d)
    assert all(0 <= b - a <= 1.0 * total / 200 + 1e-9 for a, b in zip(xs, xs[1:]))  # never faster than v_max


def test_gantry_throughput_follows_its_velocity():
    def rate(v):
        scn = scenario([{"id": "g", "type": "gantry",
                         "params": {"travel": 2.0, "lift": 0.3, "v_max": v, "a_max": 2.0, "grip_time": 0.5}}],
                       routing=["g"])
        return simulate(scn)["kpis"]["throughput_per_h"]["mean"]

    for v in (0.5, 1.0):
        assert rate(v) == pytest.approx(3600 / pick_place_time(2.0, 0.3, v, 2.0, 0.5), rel=1e-3)
    assert rate(1.0) > rate(0.5)


# ---- buffer --------------------------------------------------------------------------
def test_buffer_decouples_without_adding_time():
    """A zero-dwell buffer between two fixed stations does not change the line rate."""
    stations = [{"id": "a", "type": "cnc", "params": {"cycle_time": 30}},
                {"id": "buf", "type": "buffer", "params": {"slots": 5}},
                {"id": "b", "type": "cnc", "params": {"cycle_time": 60}}]
    res = simulate(scenario(stations, routing=["a", "buf", "b"]))
    assert res["kpis"]["throughput_per_h"]["mean"] == pytest.approx(60.0, abs=0.1)
    assert res["bottleneck"] == "b"


# ---- move-time table: solved in 3D, used headless ------------------------------------
def test_move_table_sets_the_robot_handling_time(tmp_path: Path):
    (tmp_path / "tables").mkdir()
    (tmp_path / "tables" / "cell.yaml").write_text(yaml.safe_dump({
        "solver": "test", "stations": {"robot": {"cycle_s": 12.0, "moves": [
            {"from": "in", "to": "cut", "speed": 1.0, "seconds": 6.0},
            {"from": "cut", "to": "in", "speed": 1.0, "seconds": 6.0}]}},
    }), encoding="utf-8")
    cell = {
        "name": "cell", "move_table": "tables/cell.yaml",
        "cell": {"name": "c", "stations": [
            {"id": "robot", "type": "robot", "params": {"handling_time": 99}},
            {"id": "cut", "type": "cnc", "params": {"cycle_time": 5}}]},
        "part_types": [{"name": "p", "routing": ["robot", "cut"]}],
        "run": {"duration_h": 10, "warmup_h": 1},
    }
    (tmp_path / "cell.yaml").write_text(yaml.safe_dump(cell), encoding="utf-8")
    scn = load_scenario(tmp_path / "cell.yaml")
    assert scn.cell.stations[0].params["handling_time"] == 12.0
    assert simulate(scn)["kpis"]["throughput_per_h"]["mean"] == pytest.approx(300.0, abs=0.5)


# ---- plant roll-up -------------------------------------------------------------------
def test_plant_rolls_up_from_its_cells(tmp_path: Path):
    for name, cycle in (("fast", 30), ("slow", 60)):
        (tmp_path / f"{name}.yaml").write_text(yaml.safe_dump({
            "name": name,
            "cell": {"name": name, "stations": [{"id": "m", "type": "cnc", "params": {"cycle_time": cycle}},
                                                {"id": "i", "type": "inspection", "params": {"cycle_time": 15}}]},
            "part_types": [{"name": "p", "routing": ["m", "i"]}],
            "run": {"duration_h": 10, "warmup_h": 1},
        }), encoding="utf-8")
    (tmp_path / "plant.yaml").write_text(yaml.safe_dump({"name": "p1", "cells": ["fast.yaml", "slow.yaml"]}), encoding="utf-8")
    res = simulate_plant(tmp_path / "plant.yaml")
    assert res["throughput_per_h"] == pytest.approx(120 + 60, abs=0.2)
    assert res["throughput_per_h"] == pytest.approx(sum(c["throughput_per_h"] for c in res["cells"].values()))
    assert res["mean_utilization"] == pytest.approx((1.0 + 0.5 + 1.0 + 0.25) / 4, abs=1e-3)
    assert res["bottleneck"]["station"] == "m"
    assert rollup("p1", res["results"])["throughput_per_h"] == res["throughput_per_h"]


# ---- playback from the event log -----------------------------------------------------
def test_playback_reconstructs_load_visits_from_the_event_log():
    from simcore.engine.cell import Cell
    from simcore.playback import LoadIndex, StateTrack, part_stays, pick_place_point

    scn = scenario([{"id": "load", "type": "robot", "params": {"handling_time": 10}},
                    {"id": "cut", "type": "cnc", "params": {"cycle_time": 60}}],
                   routing=["load", "cut"], duration_h=1, warmup_h=0)
    cell = Cell(scn, seed=1, record_events=True)
    cell.run()
    stays = part_stays(cell.metrics.events, end=3600.0)
    first = stays[1]
    assert [s.station for s in first] == ["load", "cut"]
    assert (first[0].t0, first[0].t_work, first[0].t1) == (0.0, 10.0, 10.0)   # nothing downstream yet
    assert (first[1].t0, first[1].t1) == (10.0, 70.0)
    second = stays[2]
    assert (second[0].t_work, second[0].t1) == (20.0, 70.0)                    # done at 20, blocked until 70
    track = StateTrack(cell.metrics.events)
    assert track.station_state("load", 30.0) == "BLOCKED"
    assert track.station_state("cut", 30.0) == "PROCESSING"
    assert LoadIndex(stays).active(30.0) == [1, 2]
    assert pick_place_point((0, 0, 1), (2, 0, 1), 0.5, 0.5) == pytest.approx((1.0, 0.0, 1.5))


# ---- clearance -----------------------------------------------------------------------
def test_bounding_box_clearance():
    cnc = Box((0, 0, 0), (2, 2, 2))
    assert boxes_overlap(cnc, Box((1.5, 0, 0), (3, 1, 1)))
    assert not boxes_overlap(cnc, Box((2, 0, 0), (3, 1, 1)))               # touching faces are allowed
    assert boxes_overlap(cnc, Box((2.05, 0, 0), (3, 1, 1)), clearance=0.1)  # but not inside the clearance


def test_tool_path_through_a_machine_is_flagged():
    cnc = Box((0, 0, 0), (2, 2, 2))
    assert segment_hits_box((-1, 1, 1), (3, 1, 1), cnc)       # straight through
    assert not segment_hits_box((-1, 1, 2.5), (3, 1, 2.5), cnc)  # over the top
    assert not segment_hits_box((-1, 1, 1), (-0.5, 1, 1), cnc)   # stops short
