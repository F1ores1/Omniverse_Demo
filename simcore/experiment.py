"""Experiment: run replications of a scenario and report means with confidence intervals."""
from __future__ import annotations

from collections import Counter
from statistics import mean, stdev

import simcore
from simcore.config.schema import Scenario, config_hash
from simcore.engine.cell import Cell

# Two-sided 95% Student t values by degrees of freedom.
_T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262}


def _ci(values: list[float]) -> dict:
    n = len(values)
    half = _T95.get(n - 1, 1.96) * stdev(values) / n ** 0.5 if n > 1 else 0.0
    return {"mean": mean(values), "ci95": half}


def simulate(scn: Scenario, record_events: bool = False) -> dict:
    """Run every replication. Returns KPIs plus the metadata needed to reproduce them."""
    runs, events = [], None
    for i in range(scn.run.replications):
        cell = Cell(scn, seed=scn.run.seed + i, record_events=record_events and i == 0)
        runs.append(cell.run())
        if i == 0:
            events = cell.metrics.events
    station_ids = list(runs[0]["stations"])
    fields = ("utilization", "active", "blocked", "starved", "down", "setup", "mean_active_period_s")
    stations = {
        sid: {"type": runs[0]["stations"][sid]["type"],
              **{f: mean(r["stations"][sid][f] for r in runs) for f in fields}}
        for sid in station_ids
    }
    votes = Counter(r["bottleneck"] for r in runs)
    return {
        "meta": {
            "scenario": scn.name,
            "config_hash": config_hash(scn),
            "schema_version": scn.schema_version,
            "engine_version": simcore.__version__,
            "seed": scn.run.seed,
            "replications": scn.run.replications,
            "duration_h": scn.run.duration_h,
            "warmup_h": scn.run.warmup_h,
        },
        "kpis": {k: _ci([r[k] for r in runs])
                 for k in ("throughput_per_h", "cycle_time_s", "cycle_time_p95_s", "wip")},
        "yield": sum(r["good"] for r in runs) / max(1, sum(r["good"] + r["scrapped"] for r in runs)),
        "stations": stations,
        "bottleneck": max(stations, key=lambda s: stations[s]["active"]),
        "bottleneck_agreement": votes.most_common(1)[0][1] / len(runs),
        "events": events,
    }
