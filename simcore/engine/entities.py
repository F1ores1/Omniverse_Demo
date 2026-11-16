"""Parts, equipment units and the equipment state machine."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from simcore.config.schema import PartTypeCfg
    from simcore.equipment.base import Equipment


class State(str, Enum):
    IDLE = "IDLE"              # zero-duration decision point between jobs
    STARVED = "STARVED"        # free, but no part is waiting
    SETUP = "SETUP"            # changeover or tool change
    PROCESSING = "PROCESSING"  # adding value
    DOWN = "DOWN"              # failed, under repair
    BLOCKED = "BLOCKED"        # finished, but downstream has no space


# A unit is "active" when it is the one holding the line up (bottleneck analysis).
ACTIVE = frozenset({State.SETUP, State.PROCESSING, State.DOWN})

# The only legal moves. Anything else is a modelling bug and raises.
TRANSITIONS: dict[State, frozenset[State]] = {
    State.IDLE: frozenset({State.STARVED, State.SETUP, State.PROCESSING}),
    State.STARVED: frozenset({State.SETUP, State.PROCESSING}),
    State.SETUP: frozenset({State.PROCESSING}),
    State.PROCESSING: frozenset({State.DOWN, State.BLOCKED, State.IDLE}),
    State.DOWN: frozenset({State.PROCESSING}),
    State.BLOCKED: frozenset({State.IDLE}),
}


class IllegalTransition(RuntimeError):
    pass


@dataclass
class Part:
    id: int
    type: "PartTypeCfg"
    released: float = 0.0  # time it entered the cell
    step: int = 0          # index into type.routing


@dataclass
class Unit:
    """One machine of a station. A station with count=2 has two units."""

    station: "Equipment"
    index: int
    state: State = State.IDLE
    part: Part | None = None
    since: float = 0.0             # when the current state began
    remaining: float = 0.0         # processing time left on the current part
    ttf: float = float("inf")      # operating time left until the next failure
    last_type: str | None = None   # part type of the previous job (changeovers)
    parts_done: int = 0
    waiting_on: tuple = ()         # stations this unit is blocked on (several if they are parallel)

    @property
    def key(self) -> tuple[str, int]:
        return (self.station.id, self.index)
