"""Plant: a set of cells, with KPIs rolled up from its children.

Fidelity choice: cells are independent (no shared resources), so plant throughput is the sum of the
cells. The plant bottleneck is the most active station anywhere in the plant.
"""
from __future__ import annotations

from pathlib import Path
from statistics import mean

import yaml

from simcore.config.schema import Strict, load_scenario
from simcore.experiment import simulate


class PlantCfg(Strict):
    name: str
    cells: list[str]  # scenario files, relative to the plant file


def load_plant(path: str | Path) -> tuple[PlantCfg, Path]:
    path = Path(path)
    return PlantCfg.model_validate(yaml.safe_load(path.read_text(encoding="utf-8"))), path.parent


def rollup(name: str, cells: dict[str, dict]) -> dict:
    """Plant KPIs from finished cell results (the dicts returned by `simulate`)."""
    stations = [(c, sid, s) for c, res in cells.items() for sid, s in res["stations"].items()]
    cell, station, worst = max(stations, key=lambda x: x[2]["active"])
    return {
        "plant": name,
        "throughput_per_h": sum(r["kpis"]["throughput_per_h"]["mean"] for r in cells.values()),
        "mean_utilization": mean(s["utilization"] for _, _, s in stations),
        "bottleneck": {"cell": cell, "station": station, "active": worst["active"]},
        "cells": {c: {"throughput_per_h": r["kpis"]["throughput_per_h"]["mean"],
                      "mean_utilization": mean(s["utilization"] for s in r["stations"].values()),
                      "bottleneck": r["bottleneck"],
                      "config_hash": r["meta"]["config_hash"]} for c, r in cells.items()},
    }


def simulate_plant(path: str | Path) -> dict:
    plant, base = load_plant(path)
    cells = {}
    for ref in plant.cells:
        res = simulate(load_scenario(base / ref))
        res.pop("events")
        cells[res["meta"]["scenario"]] = res
    return {**rollup(plant.name, cells), "results": cells}
