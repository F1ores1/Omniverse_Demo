"""Equipment catalog: the plug-and-play items, their default size, parameters and ports.

Pure Python. The Kit extension builds USD geometry from these entries and the DES reads the same
parameters, so a catalog item means the same thing in 2D, in 3D and headless.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any

Vec3 = tuple[float, float, float]
LIFT = 0.25  # m between the lowered (pick/place) and raised (approach) position of a port
LOAD_SIZE = 0.2  # m, edge of a load; conveyor end stops sit half of it from each end


@dataclass(frozen=True)
class CatalogItem:
    type: str                 # equipment registry name (simcore.equipment)
    label: str
    size: Vec3                # default X, Y, Z in metres; parametric items can be resized
    params: dict[str, Any]    # defaults; time distributions are written as text, see parse_dist
    color: Vec3 = (0.55, 0.57, 0.60)
    transfer: bool = False    # robots and gantries move loads between the ports of their neighbours
    work_height: float | None = None  # port height; None = top of the bounding box
    resizable: tuple[bool, bool, bool] = (True, True, True)
    reach: float | None = None  # m, for transfer equipment


CATALOG: dict[str, CatalogItem] = {i.type: i for i in [
    CatalogItem("load_source", "Load source", (0.6, 0.6, 0.1),
                {"shape": "box", "arrival_dist": "exponential", "arrival_mean": 30.0},
                color=(0.20, 0.45, 0.75), resizable=(True, True, False)),
    CatalogItem("conveyor", "Conveyor", (3.0, 0.6, 0.8), {"speed": 0.2, "accel": 0.5, "friction": 1.0, "drive_force": 300.0, "gap": 0.05, "slots": 4},
                color=(0.25, 0.27, 0.30), resizable=(True, True, False)),
    CatalogItem("transfer", "Transfer (lift and place)", (0.5, 0.5, 0.02),
                {"v_max": 0.5, "a_max": 1.0, "lift": 0.25, "grip_time": 0.0},
                color=(0.85, 0.75, 0.30), transfer=True, resizable=(False, False, False)),
    CatalogItem("robot", "6-axis robot (UR10)", (0.4, 0.4, 0.9), {"speed_scale": 1.0, "handling_time": "8"},
                color=(0.35, 0.55, 0.85), transfer=True, reach=1.3, resizable=(False, False, True)),
    CatalogItem("cnc", "CNC machine", (1.4, 1.6, 2.1), {"cycle_time": "20", "scrap_rate": 0.0},
                color=(0.80, 0.82, 0.85), work_height=0.9),
    CatalogItem("forge_press", "Forge press", (1.4, 1.4, 2.8), {"cycle_time": "12", "die_change_time": 0.0},
                color=(0.55, 0.30, 0.25), work_height=0.9),
    CatalogItem("inspection", "Inspection (CMM)", (1.2, 1.2, 1.9), {"cycle_time": "15", "fail_rate": 0.02},
                color=(0.30, 0.65, 0.55), work_height=0.9),
    CatalogItem("buffer", "Buffer", (1.6, 0.6, 0.8), {"slots": 4}, color=(0.45, 0.45, 0.50)),
    CatalogItem("blackbox", "DES black box", (1.2, 1.2, 0.05), {"cycle_time": "30"}, color=(0.12, 0.12, 0.14)),
]}


# Parameters picked from a list (a dropdown in the panel), not typed. Add arrival distributions here later.
CHOICES: dict[str, tuple[str, ...]] = {"shape": ("box", "cylinder"), "arrival_dist": ("exponential",)}
SOURCE_TYPES = {"load_source"}  # items that create loads; not DES stations
# Items that move loads between equipment. "transfer" is the basic one (lift, move, lower); robot and conveyor
# are upgrades. Between two items that are neither, 3D uses a default transfer and Checks flags the gap.
TRANSFER_TYPES = {"transfer", "robot", "conveyor"}
DEFAULT_TRANSFER = {"v_max": 0.5, "a_max": 1.0, "lift": 0.25, "grip_time": 0.0}


def arrival_of(params: dict[str, Any]) -> dict:
    """A Load source's arrival process as a DES distribution, e.g. exponential -> {'dist': 'exp', 'mean': 30}."""
    dist = params.get("arrival_dist", "exponential")
    if dist == "exponential":
        return {"dist": "exp", "mean": float(params["arrival_mean"])}
    raise ValueError(f"unknown arrival distribution {dist!r}")


def ports(item: CatalogItem, size: Vec3) -> dict[str, Vec3]:
    """Local port positions. Flow runs along +X: loads enter at `in` and leave at `out`.

    Machines have one work position, so both ports sit at the table centre. Conveyors and buffers
    carry the load from one end to the other. Each port has a lowered and a raised position.
    """
    x, _, z = size
    h = item.work_height if item.work_height is not None else z
    through = item.type in ("conveyor", "buffer")
    inset = min(LOAD_SIZE / 2, x / 4)  # where a load stops on the belt (Track.inlet_s / end_s), so picks land on it
    x_in, x_out = (-x / 2 + inset, x / 2 - inset) if through else (0.0, 0.0)
    return {
        "in_lowered": (x_in, 0.0, h), "in_raised": (x_in, 0.0, h + LIFT),
        "out_lowered": (x_out, 0.0, h), "out_raised": (x_out, 0.0, h + LIFT),
    }


# ---- time distributions as short text, so they are readable in a property field and in reports ----
_DIST = re.compile(r"^\s*(exp|normal|tri|uniform|avg)\s*\(([^)]*)\)\s*$")


def parse_dist(text: str | float) -> float | dict:
    """'20' -> 20.0, 'exp(60)', 'normal(240, 12)', 'tri(20, 25, 35)', 'uniform(10, 20)' -> Dist dict."""
    if isinstance(text, (int, float)):
        return float(text)
    m = _DIST.match(text)
    if not m:
        return float(text)
    kind, args = m.group(1), [float(a) for a in m.group(2).split(",")]
    if kind == "avg":  # min, average, max -> triangular whose mean is that average
        if len(args) != 3:
            raise ValueError(f"avg() takes min, average, max: {text!r}")
        lo, avg, hi = args
        mode = min(max(3 * avg - lo - hi, lo), hi)  # the peak that gives this average, kept inside [min, max]
        return {"dist": "triangular", "low": lo, "mode": mode, "high": hi}
    keys = {"exp": ("mean",), "normal": ("mean", "sd"), "tri": ("low", "mode", "high"), "uniform": ("low", "high")}[kind]
    if len(args) != len(keys):
        raise ValueError(f"{kind}() takes {len(keys)} numbers: {text!r}")
    return {"dist": "triangular" if kind == "tri" else kind, **dict(zip(keys, args))}


DIST_PARAMS = {"cycle_time", "handling_time", "transit_time", "dwell_time"}

# The time choices offered in the panel: (key, label, argument names). Stored as text that parse_dist reads.
TIME_FORMS = [("fixed", "Fixed", ("time",)), ("uniform", "Min / max", ("min", "max")),
              ("avg", "Min / average / max", ("min", "average", "max"))]


def time_form(text: str | float) -> tuple[str, list[float]]:
    """Text -> (form key, numbers) for the panel. Older forms are shown as the nearest panel form."""
    d = parse_dist(text)
    if not isinstance(d, dict):
        return "fixed", [d]
    kind = d["dist"]
    if kind == "uniform":
        return "uniform", [d["low"], d["high"]]
    if kind == "triangular":
        return "avg", [d["low"], round((d["low"] + d["mode"] + d["high"]) / 3, 3), d["high"]]
    if kind == "normal":
        return "avg", [max(0.0, d["mean"] - 2 * d["sd"]), d["mean"], d["mean"] + 2 * d["sd"]]
    return "fixed", [d["mean"]]


def time_text(form: str, args: list[float]) -> str:
    """(form key, numbers) -> stored text, e.g. ('avg', [20, 25, 35]) -> 'avg(20, 25, 35)'."""
    if form == "fixed":
        return f"{args[0]:g}"
    return f"{form}({', '.join(f'{a:g}' for a in args)})"


def autofill(form: str, centre: float) -> list[float]:
    """Starting numbers when the form is switched: spread +-20% around the current typical time."""
    return {"fixed": [centre], "uniform": [round(centre * 0.8, 2), round(centre * 1.2, 2)],
            "avg": [round(centre * 0.8, 2), round(centre, 2), round(centre * 1.2, 2)]}[form]


def station_params(item_type: str, raw: dict[str, Any], size: Vec3, travel: float | None = None) -> tuple[str, dict]:
    """Turn catalog/USD parameters into (equipment type, params) for the DES schema.

    `travel` is the pick-to-place distance of a transfer device, measured from the layout.
    """
    p = {k: (parse_dist(v) if k in DIST_PARAMS else v) for k, v in raw.items()}
    if item_type == "conveyor":  # belt length and speed -> transit time
        return "conveyor", {"transit_time": size[0] / p["speed"], "slots": int(p["slots"])}
    if item_type == "robot":
        return "robot", {"handling_time": p["handling_time"]}
    if item_type == "transfer":  # lift, trapezoid move, lower (the DES "gantry" model)
        return "gantry", {"travel": travel or size[0], "lift": p["lift"], "v_max": p["v_max"],
                          "a_max": p["a_max"], "grip_time": p["grip_time"]}
    if item_type == "buffer":
        return "buffer", {"slots": int(p["slots"])}
    if item_type == "blackbox":  # a 2D DES station: just a cycle time
        return "cnc", {"cycle_time": p["cycle_time"]}
    return item_type, p
