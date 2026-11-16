"""CLI: run scenarios headless and compare them.

    python -m simcore.run scenarios/baseline.yaml scenarios/second_cnc.yaml
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from simcore.config import load_scenario
from simcore.experiment import simulate


def _pct(x: float) -> str:
    return f"{100 * x:5.1f}%"


def report(res: dict) -> str:
    m, k = res["meta"], res["kpis"]
    lines = [
        f"== {m['scenario']}  (config {m['config_hash']}, seed {m['seed']}, {m['replications']} reps, "
        f"{m['duration_h']:g} h, engine {m['engine_version']})",
        f"   throughput  {k['throughput_per_h']['mean']:6.2f} +/- {k['throughput_per_h']['ci95']:.2f} good parts/h"
        f"   yield {_pct(res['yield'])}",
        f"   cycle time  {k['cycle_time_s']['mean'] / 60:6.1f} min avg, {k['cycle_time_p95_s']['mean'] / 60:.1f} min p95"
        f"   WIP {k['wip']['mean']:.1f} parts",
        f"   {'station':<10}{'type':<12}{'process':>8}{'setup':>8}{'down':>8}{'blocked':>9}{'starved':>9}{'active':>8}",
    ]
    for sid, s in res["stations"].items():
        mark = "  <- bottleneck" if sid == res["bottleneck"] else ""
        lines.append(
            f"   {sid:<10}{s['type']:<12}{_pct(s['utilization']):>8}{_pct(s['setup']):>8}{_pct(s['down']):>8}"
            f"{_pct(s['blocked']):>9}{_pct(s['starved']):>9}{_pct(s['active']):>8}{mark}"
        )
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description="Run work cell scenarios headless.")
    ap.add_argument("scenarios", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=Path("output/reports"))
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    results = []
    for path in args.scenarios:
        res = simulate(load_scenario(path), record_events=True)
        events = res.pop("events")
        name = res["meta"]["scenario"]
        (args.out / f"{name}.json").write_text(json.dumps(res, indent=2), encoding="utf-8")
        with (args.out / f"{name}.events.jsonl").open("w", encoding="utf-8") as f:  # replication 0, for playback
            for t, station, unit, state, part in events:
                f.write(json.dumps({"t": t, "station": station, "unit": unit, "state": state, "part": part}) + "\n")
        results.append(res)
        print(report(res), "\n")

    if len(results) > 1:
        base = results[0]["kpis"]["throughput_per_h"]["mean"]
        print(f"{'scenario':<28}{'parts/h':>9}{'vs base':>9}{'cycle min':>11}{'WIP':>6}  bottleneck")
        for r in results:
            tp = r["kpis"]["throughput_per_h"]["mean"]
            print(f"{r['meta']['scenario']:<28}{tp:>9.2f}{100 * (tp / base - 1):>+8.1f}%"
                  f"{r['kpis']['cycle_time_s']['mean'] / 60:>11.1f}{r['kpis']['wip']['mean']:>6.1f}  {r['bottleneck']}")


if __name__ == "__main__":
    main()
