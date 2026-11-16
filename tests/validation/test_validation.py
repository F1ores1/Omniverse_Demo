"""Analytical validation: the simulator must reproduce results that have a closed-form answer."""
from __future__ import annotations

import pytest

from simcore.config.schema import Scenario
from simcore.experiment import simulate

EXP = lambda mean: {"dist": "exp", "mean": mean}  # noqa: E731


def scenario(stations, interarrival=None, duration_h=50, warmup_h=2, reps=1, seed=11) -> Scenario:
    return Scenario.model_validate({
        "name": "test",
        "cell": {"name": "test-cell", "stations": stations},
        "part_types": [{"name": "p", "routing": [s["id"] for s in stations]}],
        "source": {"interarrival": interarrival},
        "run": {"duration_h": duration_h, "warmup_h": warmup_h, "seed": seed, "replications": reps},
    })


def test_mm1_queue_matches_theory():
    """M/M/1 with rho = 0.8: W = 1/(mu - lambda) = 5 s, L = rho/(1 - rho) = 4, utilization = rho."""
    scn = scenario(
        [{"id": "m", "type": "cnc", "buffer": None, "params": {"cycle_time": EXP(1.0)}}],
        interarrival=EXP(1.25), duration_h=60, reps=3,
    )
    res = simulate(scn)
    assert res["kpis"]["cycle_time_s"]["mean"] == pytest.approx(5.0, rel=0.05)
    assert res["kpis"]["wip"]["mean"] == pytest.approx(4.0, rel=0.05)
    assert res["stations"]["m"]["utilization"] == pytest.approx(0.8, rel=0.02)


def test_mm2_queue_matches_erlang_c():
    """M/M/2 with lambda = 1.6, mu = 1: Erlang C gives W = 2.778 s. Checks parallel units."""
    scn = scenario(
        [{"id": "m", "type": "cnc", "count": 2, "buffer": None, "params": {"cycle_time": EXP(1.0)}}],
        interarrival=EXP(1 / 1.6), duration_h=40, reps=3,
    )
    res = simulate(scn)
    assert res["kpis"]["cycle_time_s"]["mean"] == pytest.approx(2.778, rel=0.05)
    assert res["stations"]["m"]["utilization"] == pytest.approx(0.8, rel=0.02)


def test_deterministic_line_runs_at_the_slowest_station():
    """Fixed times 10/60/20 s, saturated: 60 parts/h, upstream blocked 5/6, downstream starved 2/3."""
    scn = scenario([
        {"id": "load", "type": "robot", "params": {"handling_time": 10}},
        {"id": "cut", "type": "cnc", "params": {"cycle_time": 60}},
        {"id": "check", "type": "inspection", "params": {"cycle_time": 20}},
    ], duration_h=20, warmup_h=1)
    res = simulate(scn)
    assert res["kpis"]["throughput_per_h"]["mean"] == pytest.approx(60.0, abs=0.1)
    assert res["bottleneck"] == "cut"
    assert res["stations"]["cut"]["utilization"] == pytest.approx(1.0, abs=1e-3)
    assert res["stations"]["load"]["blocked"] == pytest.approx(5 / 6, abs=1e-3)
    assert res["stations"]["check"]["starved"] == pytest.approx(2 / 3, abs=1e-3)


def test_failures_cut_capacity_by_availability():
    """Throughput = (1/cycle) * MTBF/(MTBF + MTTR) = 36 * 60/70 = 30.86 parts/h."""
    scn = scenario(
        [{"id": "m", "type": "cnc", "params": {"cycle_time": 100, "mtbf_h": 1, "mttr_min": 10}}],
        duration_h=400, warmup_h=10, reps=3,
    )
    res = simulate(scn)
    assert res["kpis"]["throughput_per_h"]["mean"] == pytest.approx(36 * 60 / 70, rel=0.03)
    assert res["stations"]["m"]["down"] == pytest.approx(10 / 70, rel=0.06)


def test_parallel_units_double_a_deterministic_bottleneck():
    stations = [
        {"id": "load", "type": "robot", "params": {"handling_time": 10}},
        {"id": "cut", "type": "cnc", "count": 2, "params": {"cycle_time": 60}},
    ]
    res = simulate(scenario(stations, duration_h=20, warmup_h=1))
    assert res["kpis"]["throughput_per_h"]["mean"] == pytest.approx(120.0, abs=0.2)
