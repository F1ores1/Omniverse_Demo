"""Concrete equipment types. Each one is parameters plus a few hooks; the engine is untouched."""
from __future__ import annotations

from pydantic import Field

from simcore.config.schema import Dist, Strict
from simcore.engine.entities import Part, Unit
from simcore.equipment.base import Equipment, register
from simcore.kinematics import pick_place_time


class _Machine(Equipment):
    """Shared hooks for anything with a cycle time that may be overridden per part type."""

    def process_time(self, part: Part) -> float:
        dist = part.type.times.get(self.id, self.p.cycle_time)
        return dist.sample(self.rng)

    def passes(self, part: Part) -> bool:
        rate = getattr(self.p, "scrap_rate", 0.0)
        return rate == 0 or self.rng.random() >= rate


@register("conveyor")
class Conveyor(Equipment):
    """Accumulating conveyor, approximated as `slots` positions with a fixed transit time."""

    class Params(Strict):
        transit_time: Dist
        slots: int = Field(ge=1)

    def unit_count(self) -> int:
        return self.p.slots

    def buffer_cap(self) -> int:
        return 0

    def process_time(self, part: Part) -> float:
        return self.p.transit_time.sample(self.rng)


@register("robot")
class Robot(Equipment):
    """Handling robot, approximated as a pick-and-place time. Reach and collisions are a physics check."""

    class Params(Strict):
        handling_time: Dist

    def process_time(self, part: Part) -> float:
        return self.p.handling_time.sample(self.rng)


@register("gantry")
class Gantry(Equipment):
    """Gantry pick-and-place. Move time comes from travel, lift and a trapezoidal velocity profile."""

    class Params(Strict):
        travel: float = Field(gt=0)        # m, horizontal distance from pick point to place point
        lift: float = Field(0.3, ge=0)     # m, raised position to lowered position
        v_max: float = Field(gt=0)         # m/s
        a_max: float = Field(gt=0)         # m/s^2
        grip_time: float = Field(0.5, ge=0)
        handling_time: Dist | None = None  # if set (e.g. from a move-time table), replaces the estimate

    def process_time(self, part: Part) -> float:
        p = self.p
        if p.handling_time is not None:
            return p.handling_time.sample(self.rng)
        return pick_place_time(p.travel, p.lift, p.v_max, p.a_max, p.grip_time)


@register("buffer")
class Buffer(Equipment):
    """Physical buffer: `slots` positions, first in first out. A part leaves as soon as downstream has space."""

    class Params(Strict):
        slots: int = Field(ge=1)
        dwell_time: Dist = Dist(dist="fixed", mean=0.0)  # e.g. a cooling time

    def unit_count(self) -> int:
        return self.p.slots

    def buffer_cap(self) -> int | None:
        return self.cfg.buffer

    def process_time(self, part: Part) -> float:
        return self.p.dwell_time.sample(self.rng)


@register("cnc")
class CNC(_Machine):
    class Params(Strict):
        cycle_time: Dist
        mtbf_h: float | None = Field(None, gt=0)
        mttr_min: float | None = Field(None, ge=0)
        scrap_rate: float = Field(0.0, ge=0, lt=1)
        tool_change_every: int | None = Field(None, ge=1)  # parts between tool changes
        tool_change_time: float = 0.0

    def setup_time(self, unit: Unit, part: Part) -> float:
        n = self.p.tool_change_every
        return self.p.tool_change_time if n and unit.parts_done and unit.parts_done % n == 0 else 0.0


@register("forge_press")
class ForgePress(_Machine):
    class Params(Strict):
        cycle_time: Dist
        die_change_time: float = 0.0  # paid when the part type changes
        mtbf_h: float | None = Field(None, gt=0)
        mttr_min: float | None = Field(None, ge=0)
        scrap_rate: float = Field(0.0, ge=0, lt=1)

    def setup_time(self, unit: Unit, part: Part) -> float:
        changed = unit.last_type not in (None, part.type.name)
        return self.p.die_change_time if changed else 0.0


@register("inspection")
class Inspection(_Machine):
    class Params(Strict):
        cycle_time: Dist
        fail_rate: float = Field(0.0, ge=0, lt=1)  # failed parts are scrapped (rework loop deferred)

    def passes(self, part: Part) -> bool:
        return self.rng.random() >= self.p.fail_rate
