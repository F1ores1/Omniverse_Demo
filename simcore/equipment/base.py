"""Equipment base class: one shared state machine and material-flow logic.

A concrete type only answers four questions (the hooks below). Blocking,
starving, failures and hand-offs are handled here, once, for every type.
"""
from __future__ import annotations

import random
from collections import deque
from typing import TYPE_CHECKING, Callable

from pydantic import BaseModel

from simcore.engine.entities import TRANSITIONS, IllegalTransition, Part, State, Unit

if TYPE_CHECKING:
    from simcore.config.schema import StationCfg
    from simcore.engine.cell import Cell

REGISTRY: dict[str, type["Equipment"]] = {}


def register(name: str):
    """Register an equipment type under the name used in config (`type: cnc`)."""

    def deco(cls: type["Equipment"]) -> type["Equipment"]:
        cls.type_name = name
        REGISTRY[name] = cls
        return cls

    return deco


class Equipment:
    type_name = "equipment"
    Params: type[BaseModel]  # each type declares and validates its own parameters

    def __init__(self, cfg: "StationCfg", cell: "Cell") -> None:
        self.id, self.cfg, self.cell, self.engine = cfg.id, cfg, cell, cell.engine
        self.p = self.Params.model_validate(cfg.params)
        # One random stream per station: changing one station leaves the others' draws unchanged.
        self.rng = random.Random(f"{cell.seed}:{cfg.id}")
        self.queue: deque[Part] = deque()     # input buffer
        self.blocked: deque[Unit] = deque()   # upstream units waiting for space here
        self.feed: Callable[[], None] | None = None  # set by the source on first stations
        mtbf_h, mttr_min = getattr(self.p, "mtbf_h", None), getattr(self.p, "mttr_min", None)
        self.mtbf = mtbf_h * 3600 if mtbf_h else None
        self.mttr = (mttr_min or 0) * 60
        self.units = [Unit(self, i) for i in range(self.unit_count())]
        for u in self.units:
            if self.mtbf:
                u.ttf = self.rng.expovariate(1 / self.mtbf)
            cell.metrics.register(u)

    # ---- hooks a concrete equipment type overrides -------------------------------
    def unit_count(self) -> int:
        return self.cfg.count

    def buffer_cap(self) -> int | None:
        return self.cfg.buffer

    def process_time(self, part: Part) -> float:
        raise NotImplementedError

    def setup_time(self, unit: Unit, part: Part) -> float:
        return 0.0

    def passes(self, part: Part) -> bool:
        return True

    # ---- state machine -----------------------------------------------------------
    def _set(self, unit: Unit, new: State) -> None:
        if new not in TRANSITIONS[unit.state]:
            raise IllegalTransition(f"{self.id}[{unit.index}]: {unit.state.value} -> {new.value}")
        self.cell.metrics.on_state(unit, new)
        unit.state = new

    # ---- material flow -----------------------------------------------------------
    def can_accept(self) -> bool:
        cap = self.buffer_cap()
        free = sum(u.state in (State.IDLE, State.STARVED) for u in self.units)
        return cap is None or len(self.queue) < cap + free

    def load(self) -> float:
        """Parts here per unit. Used to choose between parallel alternatives."""
        return (len(self.queue) + sum(u.part is not None for u in self.units)) / len(self.units)

    def accept(self, part: Part) -> None:
        self.queue.append(part)
        self.dispatch()

    def dispatch(self) -> None:
        """Pair waiting parts with free units, then pull from whoever is blocked on us."""
        for u in self.units:
            if u.state not in (State.IDLE, State.STARVED):
                continue
            if self.queue:
                self._begin(u, self.queue.popleft())
            elif u.state is State.IDLE:
                self._set(u, State.STARVED)
        self._pull_upstream()

    def _begin(self, u: Unit, part: Part) -> None:
        u.part = part
        setup = self.setup_time(u, part)
        u.last_type = part.type.name
        if setup > 0:
            self._set(u, State.SETUP)
            self.engine.schedule(setup, self._process, u)
        else:
            self._process(u)

    def _process(self, u: Unit) -> None:
        self._set(u, State.PROCESSING)
        u.remaining = self.process_time(u.part)
        self._run(u)

    def _run(self, u: Unit) -> None:
        """Run the current job until it finishes or the unit fails (operation-dependent failures)."""
        if u.ttf > u.remaining:
            u.ttf -= u.remaining
            self.engine.schedule(u.remaining, self._finish, u)
        else:
            u.remaining -= u.ttf
            self.engine.schedule(u.ttf, self._fail, u)

    def _fail(self, u: Unit) -> None:
        self._set(u, State.DOWN)
        self.engine.schedule(self.rng.expovariate(1 / self.mttr) if self.mttr else 0.0, self._repair, u)

    def _repair(self, u: Unit) -> None:
        u.ttf = self.rng.expovariate(1 / self.mtbf)
        self._set(u, State.PROCESSING)
        self._run(u)  # resume the interrupted job

    def _finish(self, u: Unit) -> None:
        part = u.part
        u.parts_done += 1
        if not self.passes(part):
            self.cell.exit(part, good=False)
            return self._release(u)
        options = self.cell.next_stations(part)  # more than one = parallel alternatives
        open_ = [s for s in options if s.can_accept()]
        if not options:
            self.cell.exit(part, good=True)
            self._release(u)
        elif open_:
            part.step += 1
            min(open_, key=Equipment.load).accept(part)  # least loaded; ties go to the first listed
            self._release(u)
        else:  # blocking after service: hold the part until any alternative has space
            self._set(u, State.BLOCKED)
            u.waiting_on = options
            for s in options:
                s.blocked.append(u)

    def _release(self, u: Unit) -> None:
        u.part = None
        self._set(u, State.IDLE)
        self.dispatch()

    def _pull_upstream(self) -> None:
        while self.blocked and self.can_accept():
            u = self.blocked.popleft()  # first blocked, first served
            for other in u.waiting_on:  # it was also waiting on our parallel alternatives
                if other is not self:
                    other.blocked.remove(u)
            u.waiting_on = ()
            part = u.part
            part.step += 1
            self.accept(part)
            u.station._release(u)
        if self.feed:
            self.feed()
