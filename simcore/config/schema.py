"""Config schema: cells and scenarios are versioned YAML, validated here.

Times are in seconds unless the field name says otherwise (`_h`, `_min`).
"""
from __future__ import annotations

import copy
import hashlib
import json
import random
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

SCHEMA_VERSION = "0.2"  # 0.2: parallel routing steps, move_table, gantry and buffer types


class Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")  # a typo in YAML is an error, not a silent default


class Dist(Strict):
    """A time distribution. A bare number in YAML means a fixed time."""

    dist: Literal["fixed", "exp", "normal", "triangular", "uniform"] = "fixed"
    mean: float | None = None
    sd: float | None = None
    low: float | None = None
    mode: float | None = None
    high: float | None = None

    @model_validator(mode="before")
    @classmethod
    def _from_number(cls, v: Any) -> Any:
        return {"dist": "fixed", "mean": v} if isinstance(v, (int, float)) else v

    @model_validator(mode="after")
    def _check(self) -> "Dist":
        need = {
            "fixed": ["mean"], "exp": ["mean"], "normal": ["mean", "sd"],
            "triangular": ["low", "mode", "high"], "uniform": ["low", "high"],
        }[self.dist]
        missing = [f for f in need if getattr(self, f) is None]
        if missing:
            raise ValueError(f"dist '{self.dist}' needs {missing}")
        return self

    def sample(self, rng: random.Random) -> float:
        if self.dist == "fixed":
            return self.mean
        if self.dist == "exp":
            return rng.expovariate(1.0 / self.mean)
        if self.dist == "normal":
            return max(0.0, rng.gauss(self.mean, self.sd))  # truncated at zero
        if self.dist == "triangular":
            return rng.triangular(self.low, self.high, self.mode)
        return rng.uniform(self.low, self.high)


class AssetRef(Strict):
    """Link from a config entity to its USD asset."""

    usd: str
    variant: str | None = None


class StationCfg(Strict):
    id: str
    type: str                        # a name in the equipment registry
    count: int = Field(1, ge=1)      # parallel units
    buffer: int | None = Field(0, ge=0)  # queue slots in front; null = unlimited
    params: dict[str, Any] = {}      # validated by the equipment type's own model
    asset: AssetRef | None = None


class CellCfg(Strict):
    name: str
    stations: list[StationCfg]
    usd_stage: str | None = None


class PartTypeCfg(Strict):
    name: str
    routing: list[str | list[str]]   # ordered station ids; a nested list = parallel alternatives

    @property
    def route_ids(self) -> list[str]:
        return [s for step in self.routing for s in ([step] if isinstance(step, str) else step)]
    mix: float = Field(1.0, gt=0)    # relative share of releases
    times: dict[str, Dist] = {}      # per-station process time overrides


class SourceCfg(Strict):
    interarrival: Dist | None = None  # null = saturated: raw stock is always available


class RunCfg(Strict):
    duration_h: float = Field(gt=0)
    warmup_h: float = Field(0.0, ge=0)
    seed: int = 1
    replications: int = Field(1, ge=1)


class Scenario(Strict):
    schema_version: str = SCHEMA_VERSION
    name: str
    description: str = ""
    cell: CellCfg
    part_types: list[PartTypeCfg]
    source: SourceCfg = SourceCfg()
    run: RunCfg
    move_table: str | None = None    # robot/gantry move times solved in 3D (see _apply_move_table)

    @model_validator(mode="after")
    def _check(self) -> "Scenario":
        from simcore.equipment import REGISTRY  # late import: equipment depends on this module

        ids = [s.id for s in self.cell.stations]
        if len(ids) != len(set(ids)):
            raise ValueError("station ids must be unique")
        by_id = {s.id: s for s in self.cell.stations}
        for s in self.cell.stations:
            if s.type not in REGISTRY:
                raise ValueError(f"station '{s.id}': unknown equipment type '{s.type}' (known: {sorted(REGISTRY)})")
            REGISTRY[s.type].Params.model_validate(s.params)
        for pt in self.part_types:
            unknown = [r for r in pt.route_ids + list(pt.times) if r not in by_id]
            if not pt.routing or unknown:
                raise ValueError(f"part type '{pt.name}': bad routing {unknown or '(empty)'}")
            if not isinstance(pt.routing[0], str) or [] in pt.routing:
                raise ValueError(f"part type '{pt.name}': the first step must be one station, and no step may be empty")
            if self.source.interarrival is None and by_id[pt.routing[0]].buffer is None:
                raise ValueError("a saturated source needs a finite buffer on the first station")
        return self


def _apply(raw: Any, path: str, value: Any) -> None:
    """Set a dotted path. Lists are addressed by item id/name: cell.stations.cnc.count"""
    node, keys = raw, path.split(".")
    for i, key in enumerate(keys):
        last = i == len(keys) - 1
        if isinstance(node, list):
            idx = next((j for j, x in enumerate(node)
                        if isinstance(x, dict) and key in (x.get("id"), x.get("name"))), None)
            if idx is None:
                raise KeyError(f"override '{path}': no item '{key}'")
            if last:
                node[idx] = value
            else:
                node = node[idx]
        elif last:
            node[key] = value
        else:
            node = node.setdefault(key, {})


def _apply_move_table(raw: dict, table_path: Path) -> None:
    """Use move times solved in 3D as the handling time of each listed robot or gantry.

    The table keeps every solved move (from, to, speed, seconds) for traceability. The DES uses
    `cycle_s`, the full pick-place-return cycle. This is how a layout solved once in 3D runs headless.
    """
    table = yaml.safe_load(table_path.read_text(encoding="utf-8"))
    for station_id, entry in table["stations"].items():
        _apply(raw, f"cell.stations.{station_id}.params.handling_time", entry["cycle_s"])


def _load_raw(path: Path) -> dict:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    base = raw.pop("extends", None)
    overrides = raw.pop("overrides", {})
    own_table = raw.get("move_table")
    if base:
        merged = copy.deepcopy(_load_raw(path.parent / base))
        merged.update(raw)  # name, description
        raw = merged
    if own_table:
        _apply_move_table(raw, path.parent / own_table)
    for dotted, value in overrides.items():
        _apply(raw, dotted, value)
    return raw


def load_scenario(path: str | Path) -> Scenario:
    """Load a scenario. A variant is `extends: base.yaml` plus dotted `overrides`."""
    return Scenario.model_validate(_load_raw(Path(path)))


def config_hash(scn: Scenario) -> str:
    """Stable hash of the fully resolved config. It is stamped on every output."""
    canonical = json.dumps(scn.model_dump(mode="json"), sort_keys=True)
    return hashlib.sha256(canonical.encode()).hexdigest()[:12]
