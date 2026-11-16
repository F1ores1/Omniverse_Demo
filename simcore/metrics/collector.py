"""Metrics: time-in-state logs -> cycle time, throughput, utilization, WIP, bottleneck."""
from __future__ import annotations

from collections import defaultdict
from statistics import mean
from typing import TYPE_CHECKING

from simcore.engine.entities import ACTIVE, Part, State, Unit

if TYPE_CHECKING:
    from simcore.engine.engine import Engine


class MetricsCollector:
    def __init__(self, engine: "Engine", record_events: bool = False) -> None:
        self.engine = engine
        self.units: list[Unit] = []
        self.events: list[tuple] | None = [] if record_events else None
        self.reset()

    def register(self, unit: Unit) -> None:
        self.units.append(unit)

    def reset(self) -> None:
        """Start measuring from now. Called at the end of warm-up."""
        now = self.engine.now
        self.t0 = now
        self.time_in_state: dict[tuple, dict[State, float]] = defaultdict(lambda: defaultdict(float))
        self.periods: dict[tuple, list[float]] = defaultdict(list)  # closed active periods
        self._start: dict[tuple, float] = {}  # when the open active period began
        self._gap: dict[tuple, float] = {}    # when the unit last left the active set
        self.good = self.scrapped = 0
        self.cycle_times: list[float] = []
        self.wip, self._wip_area, self._wip_t = getattr(self, "wip", 0), 0.0, now
        for u in self.units:
            u.since = now
            if u.state in ACTIVE:
                self._start[u.key] = now

    # ---- hooks called by the simulation ------------------------------------------
    def on_state(self, u: Unit, new: State) -> None:
        now, key = self.engine.now, u.key
        self.time_in_state[key][u.state] += now - u.since
        u.since = now
        was, will = u.state in ACTIVE, new in ACTIVE
        if was and not will:
            self._gap[key] = now
        elif will and not was:
            gap = self._gap.pop(key, None)
            if gap is None or key not in self._start:
                self._start[key] = now
            elif now > gap:  # a real pause closes the period; back-to-back jobs extend it
                self.periods[key].append(gap - self._start[key])
                self._start[key] = now
        if self.events is not None:
            self.events.append((round(now, 3), u.station.id, u.index, new.value, u.part.id if u.part else None))

    def _wip_step(self, delta: int) -> None:
        now = self.engine.now
        self._wip_area += self.wip * (now - self._wip_t)
        self._wip_t, self.wip = now, self.wip + delta

    def on_release(self, part: Part) -> None:
        part.released = self.engine.now
        self._wip_step(+1)

    def on_exit(self, part: Part, good: bool) -> None:
        self._wip_step(-1)
        if not good:
            self.scrapped += 1
            return
        self.good += 1
        if part.released >= self.t0:
            self.cycle_times.append(self.engine.now - part.released)

    # ---- results -----------------------------------------------------------------
    def kpis(self) -> dict:
        now = self.engine.now
        span = now - self.t0
        self._wip_step(0)
        stations: dict[str, dict] = {}
        by_station: dict[str, list[Unit]] = defaultdict(list)
        for u in self.units:
            by_station[u.station.id].append(u)
        for sid, units in by_station.items():
            shares: dict[str, float] = defaultdict(float)
            periods: list[float] = []
            for u in units:
                tis = dict(self.time_in_state[u.key])
                tis[u.state] = tis.get(u.state, 0.0) + now - u.since  # close the open interval
                for state, t in tis.items():
                    shares[state.value] += t / span / len(units)
                periods += self.periods[u.key]
                if u.key in self._start:  # the still-open active period counts too
                    periods.append(self._gap.get(u.key, now) - self._start[u.key])
            stations[sid] = {
                "type": units[0].station.type_name,
                "utilization": shares["PROCESSING"],
                "active": sum(shares[s.value] for s in ACTIVE),
                "blocked": shares["BLOCKED"],
                "starved": shares["STARVED"],
                "down": shares["DOWN"],
                "setup": shares["SETUP"],
                "mean_active_period_s": mean(periods) if periods else 0.0,
            }
        ct = sorted(self.cycle_times)
        return {
            "throughput_per_h": self.good / span * 3600,
            "good": self.good,
            "scrapped": self.scrapped,
            "cycle_time_s": mean(ct) if ct else float("nan"),
            "cycle_time_p95_s": ct[int(0.95 * (len(ct) - 1))] if ct else float("nan"),
            "wip": self._wip_area / span,
            "stations": stations,
            # Bottleneck = the station that is active (not blocked, not starved) the most.
            "bottleneck": max(stations, key=lambda s: stations[s]["active"]),
        }
