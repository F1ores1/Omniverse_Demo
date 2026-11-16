"""Cell: builds equipment from config, releases parts and runs one replication."""
from __future__ import annotations

import random
from collections import deque

from simcore.config.schema import Scenario
from simcore.engine.engine import Engine
from simcore.engine.entities import Part
from simcore.equipment import REGISTRY, Equipment
from simcore.metrics import MetricsCollector


class Cell:
    def __init__(self, scn: Scenario, seed: int, record_events: bool = False) -> None:
        self.scn, self.seed = scn, seed
        self.engine = Engine()
        self.metrics = MetricsCollector(self.engine, record_events)
        self.stations: dict[str, Equipment] = {
            s.id: REGISTRY[s.type](s, self) for s in scn.cell.stations
        }
        # Source state
        self._rng = random.Random(f"{seed}:source")
        self._ids = 0
        self._pending: deque[Part] = deque()  # arrived, waiting for space in the cell
        for pt in scn.part_types:
            self.stations[pt.routing[0]].feed = self._push

    # ---- routing -----------------------------------------------------------------
    def next_stations(self, part: Part) -> tuple[Equipment, ...]:
        """The next routing step. A step that lists several stations means they run in parallel."""
        route = part.type.routing
        if part.step + 1 >= len(route):
            return ()
        step = route[part.step + 1]
        return tuple(self.stations[s] for s in ([step] if isinstance(step, str) else step))

    def exit(self, part: Part, good: bool) -> None:
        self.metrics.on_exit(part, good)

    # ---- source ------------------------------------------------------------------
    def _new_part(self) -> Part:
        types = self.scn.part_types
        self._ids += 1
        return Part(self._ids, self._rng.choices(types, [t.mix for t in types])[0])

    def _arrive(self) -> None:
        self._pending.append(self._new_part())
        self._push()
        self.engine.schedule(self.scn.source.interarrival.sample(self._rng), self._arrive)

    def _push(self) -> None:
        """Release parts into the cell while the first station has space."""
        saturated = self.scn.source.interarrival is None
        while True:
            if saturated and not self._pending:
                self._pending.append(self._new_part())
            if not self._pending:
                return
            first = self.stations[self._pending[0].type.routing[0]]
            if not first.can_accept():
                return
            part = self._pending.popleft()  # take it before accept(): accept can re-enter _push
            self.metrics.on_release(part)
            first.accept(part)

    # ---- run ---------------------------------------------------------------------
    def run(self) -> dict:
        run = self.scn.run
        for st in self.stations.values():
            st.dispatch()
        if self.scn.source.interarrival is None:
            self._push()
        else:
            self.engine.schedule(self.scn.source.interarrival.sample(self._rng), self._arrive)
        if run.warmup_h:
            self.engine.schedule(run.warmup_h * 3600, self.metrics.reset)
        self.engine.run(until=run.duration_h * 3600)
        return self.metrics.kpis()
