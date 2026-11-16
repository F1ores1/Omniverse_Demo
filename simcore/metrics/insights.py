"""Bottleneck insights: per-station state shares (from the DES or from 3D mode) -> a ranked bottleneck and plain lines.

Same rule as the DES collector: the bottleneck is the station that is working (not blocked, not starved) the most.
The lines then explain it with blocked/starved analysis: stations before it wait on it (blocked), stations after it
wait for it (starved).
"""
from __future__ import annotations


def bottleneck_insights(stations: dict[str, dict[str, float]], order: list[str] | None = None) -> dict:
    """`stations`: id -> {"utilization", "blocked", "starved", "down"} shares (0..1), in flow order.
    Returns {"bottleneck", "ranking", "lines"}."""
    if not stations:
        return {"bottleneck": None, "ranking": [], "lines": ["No station data yet."]}
    order = [s for s in (order or list(stations)) if s in stations]
    active = {s: v["utilization"] + v.get("down", 0.0) for s, v in stations.items()}
    ranking = sorted(stations, key=lambda s: -active[s])
    b = ranking[0]
    lines = [f"Bottleneck: {b}, working {100 * stations[b]['utilization']:.0f}% of the time"
             + (f" and down {100 * stations[b]['down']:.0f}%" if stations[b].get("down", 0) > 0.005 else "") + "."]
    if len(ranking) > 1:
        gap = active[b] - active[ranking[1]]
        lines.append(f"Next is {ranking[1]} at {100 * stations[ranking[1]]['utilization']:.0f}%"
                     + (" - close: speeding up the bottleneck alone will soon move it there." if gap < 0.1
                        else f" ({100 * gap:.0f} points lower)."))
    i = order.index(b) if b in order else -1
    upstream = [s for s in order[:i] if stations[s]["blocked"] > 0.05]
    downstream = [s for s in order[i + 1:] if stations[s]["starved"] > 0.05]
    if upstream:
        lines.append("Waiting on it (blocked): " + ", ".join(f"{s} {100 * stations[s]['blocked']:.0f}%" for s in upstream) + ".")
    if downstream:
        lines.append("Waiting for it (starved): " + ", ".join(f"{s} {100 * stations[s]['starved']:.0f}%" for s in downstream) + ".")
    idle = [s for s in order if stations[s]["utilization"] < 0.3 and s != b]
    if idle:
        lines.append("Spare capacity (under 30% working): " + ", ".join(idle) + ".")
    return {"bottleneck": b, "ranking": ranking, "lines": lines}
