"""Platform behavior: config is the source of truth, equipment plugs in, runs are reproducible."""
from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import ValidationError

from simcore.config import config_hash, load_scenario
from simcore.config.schema import Dist, Scenario, Strict
from simcore.engine.cell import Cell
from simcore.engine.entities import IllegalTransition, State
from simcore.equipment import Equipment, register
from simcore.experiment import simulate

SCENARIOS = Path(__file__).parents[2] / "scenarios"


def short(scn: Scenario) -> Scenario:
    return scn.model_copy(update={"run": scn.run.model_copy(update={"duration_h": 10, "warmup_h": 1, "replications": 1})})


def test_every_scenario_file_loads_and_runs():
    for path in sorted(SCENARIOS.glob("*.yaml")):
        res = simulate(short(load_scenario(path)))
        assert res["kpis"]["throughput_per_h"]["mean"] > 0, path.name


def test_time_in_state_adds_up():
    res = simulate(short(load_scenario(SCENARIOS / "baseline.yaml")))
    for sid, s in res["stations"].items():
        assert s["active"] + s["blocked"] + s["starved"] == pytest.approx(1.0, abs=1e-9), sid


def test_variant_is_a_config_change_with_its_own_hash():
    base, variant = load_scenario(SCENARIOS / "baseline.yaml"), load_scenario(SCENARIOS / "second_cnc.yaml")
    assert [s.count for s in variant.cell.stations if s.id == "cnc"] == [2]
    assert config_hash(base) != config_hash(variant)
    assert config_hash(base) == config_hash(load_scenario(SCENARIOS / "baseline.yaml"))


def test_same_config_and_seed_give_identical_results():
    scn = short(load_scenario(SCENARIOS / "baseline.yaml"))
    assert simulate(scn) == simulate(scn)


def test_bad_config_is_rejected():
    raw = load_scenario(SCENARIOS / "baseline.yaml").model_dump()
    raw["cell"]["stations"][2]["type"] = "laser"
    with pytest.raises(ValidationError, match="unknown equipment type"):
        Scenario.model_validate(raw)
    raw = load_scenario(SCENARIOS / "baseline.yaml").model_dump()
    raw["cell"]["stations"][2]["params"]["cycle_tmie"] = 200  # typo
    with pytest.raises(ValidationError):
        Scenario.model_validate(raw)


def test_illegal_state_transition_raises():
    cell = Cell(short(load_scenario(SCENARIOS / "baseline.yaml")), seed=1)
    cnc = cell.stations["cnc"]
    cnc.dispatch()
    assert cnc.units[0].state is State.STARVED
    with pytest.raises(IllegalTransition):
        cnc._set(cnc.units[0], State.BLOCKED)


def test_new_equipment_type_plugs_in_without_engine_changes():
    @register("wash")
    class Wash(Equipment):
        class Params(Strict):
            soak_time: Dist

        def process_time(self, part):
            return self.p.soak_time.sample(self.rng)

    raw = load_scenario(SCENARIOS / "baseline.yaml").model_dump()
    raw["cell"]["stations"].insert(3, {"id": "wash", "type": "wash", "buffer": 1, "params": {"soak_time": 40}})
    raw["part_types"][0]["routing"].insert(3, "wash")
    res = simulate(short(Scenario.model_validate(raw)))
    assert res["stations"]["wash"]["utilization"] > 0
