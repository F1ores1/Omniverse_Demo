"""USD layer: the stage holds geometry and layout, linked to config by `factory:*` attributes.

Needs only `pxr` (no Kit), so it is testable headless. Behavior never lives here: this module
places equipment, reads the layout back, and turns a stage into a scenario dict for the DES.
"""
from __future__ import annotations

import json
from math import hypot

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

from simcore.catalog import CATALOG, SOURCE_TYPES, TRANSFER_TYPES, CatalogItem, arrival_of, parse_dist, ports, station_params
from simcore.kinematics import Box
from simcore.usd import proxies

PLANT = "/Plant"
NS = "factory:"
PARAM = NS + "param:"

_TYPES = {bool: Sdf.ValueTypeNames.Bool, int: Sdf.ValueTypeNames.Int,
          float: Sdf.ValueTypeNames.Double, str: Sdf.ValueTypeNames.String}


def _set(prim: Usd.Prim, name: str, value) -> None:
    if isinstance(value, (tuple, list)):
        prim.CreateAttribute(name, Sdf.ValueTypeNames.Double3).Set(Gf.Vec3d(*value))
    else:
        prim.CreateAttribute(name, _TYPES[type(value)]).Set(value)


def _get(prim: Usd.Prim, name: str, default=None):
    attr = prim.GetAttribute(name)
    return attr.Get() if attr and attr.HasValue() else default


# ---- plant ---------------------------------------------------------------------------
PLANT_DEFAULTS = {"flow": "", "arrival": "saturated", "loadShape": "box",
                  "durationH": 8.0, "warmupH": 0.5, "seed": 1, "replications": 3}


def find_plant(stage: Usd.Stage) -> Usd.Prim | None:
    """The plant prim if the stage has one. Read-only: never changes the stage (unlike plant())."""
    prim = stage.GetPrimAtPath(PLANT)
    return prim if prim.IsValid() else None


def new_plant(stage: Usd.Stage, name: str = "plant") -> Usd.Prim:
    # Composer's default /Environment (ground, sky) is Y-up in cm. Switching the stage to Z-up metres would stand it
    # on its side at 100x size, so turn and scale it to match.
    env = stage.GetPrimAtPath("/Environment")
    old_up, old_mpu = UsdGeom.GetStageUpAxis(stage), UsdGeom.GetStageMetersPerUnit(stage)
    if env and not env.GetAttribute("xformOp:rotateX:zUp") and (old_up == UsdGeom.Tokens.y or old_mpu != 1.0):
        x = UsdGeom.Xformable(env)
        if old_up == UsdGeom.Tokens.y:
            x.AddRotateXOp(opSuffix="zUp").Set(90.0)
        if old_mpu != 1.0:
            x.AddScaleOp(opSuffix="toMeters").Set(Gf.Vec3f(old_mpu, old_mpu, old_mpu))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    plant = UsdGeom.Xform.Define(stage, PLANT).GetPrim()
    Usd.ModelAPI(plant).SetKind("assembly")
    stage.SetDefaultPrim(plant)
    if not any(p.HasAPI(UsdLux.LightAPI) for p in stage.Traverse()):  # an empty stage renders black
        dome = UsdLux.DomeLight.Define(stage, "/Environment/Dome")
        dome.CreateIntensityAttr(250.0)
        sun = UsdLux.DistantLight.Define(stage, "/Environment/Sun")
        sun.CreateIntensityAttr(900.0)
        sun.CreateAngleAttr(2.0)
        UsdGeom.XformCommonAPI(sun).SetRotate(Gf.Vec3f(-50.0, 0.0, 30.0))
        UsdGeom.XformCommonAPI(sun).SetTranslate(Gf.Vec3d(0.0, 25.0, 12.0))  # keeps its viewport icon off the line
        proxies._box(stage, Sdf.Path("/Environment/Floor"), (-40, -40, -0.02), (40, 40, 0.0), (0.22, 0.24, 0.27))
    for key, value in {"plantName": name, **PLANT_DEFAULTS}.items():
        if not plant.HasAttribute(NS + key):
            _set(plant, NS + key, value)
    return plant


def plant(stage: Usd.Stage) -> Usd.Prim:
    """The plant prim, created (and the stage switched to Z-up metres) if missing. Only call this when writing."""
    return find_plant(stage) or new_plant(stage)


# ---- equipment -----------------------------------------------------------------------
def equipment_prims(stage: Usd.Stage) -> list[Usd.Prim]:
    root = find_plant(stage)
    return [p for p in root.GetChildren() if p.HasAttribute(NS + "equipmentType")] if root else []


def item_of(prim: Usd.Prim) -> CatalogItem:
    return CATALOG[_get(prim, NS + "equipmentType")]


def size_of(prim: Usd.Prim) -> tuple[float, float, float]:
    return tuple(_get(prim, NS + "size"))


def add_equipment(stage: Usd.Stage, type_name: str, position=(0.0, 0.0, 0.0), yaw_deg: float = 0.0,
                  station_id: str | None = None, size=None, params: dict | None = None) -> Usd.Prim:
    """Place a catalog item. Its prim name is its station id in the scenario."""
    item = CATALOG[type_name]
    taken = {p.GetName() for p in plant(stage).GetChildren()}
    if station_id is None:
        station_id = next(f"{type_name}_{n}" for n in range(1, 1000) if f"{type_name}_{n}" not in taken)
    xform = UsdGeom.Xform.Define(stage, f"{PLANT}/{station_id}")
    prim = xform.GetPrim()
    api = UsdGeom.XformCommonAPI(xform)  # translate / rotateXYZ / scale: what the Kit gizmo edits
    api.SetTranslate(Gf.Vec3d(*position))
    api.SetRotate(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))
    api.SetScale(Gf.Vec3f(1.0, 1.0, 1.0))
    Usd.ModelAPI(prim).SetKind("component")  # a viewport click selects the whole item, not one box
    _set(prim, NS + "equipmentType", type_name)
    _set(prim, NS + "configRef", f"cell.stations.{station_id}")
    _set(prim, NS + "size", tuple(size or item.size))
    _set(prim, NS + "count", 1)
    _set(prim, NS + "buffer", 0)   # queue slots in front; -1 = unlimited
    _set(prim, NS + "show3d", True)
    for key, value in {**item.params, **(params or {})}.items():
        _set(prim, PARAM + key, value)
    rebuild(prim)
    return prim


UNIT_SPACING = 0.5  # m between parallel units, side by side


def unit_offsets(prim: Usd.Prim) -> list[float]:
    """Sideways (local Y) offsets of each parallel unit, centred on the item's position."""
    n = max(1, int(_get(prim, NS + "count", 1) or 1))
    pitch = size_of(prim)[1] + UNIT_SPACING
    return [(k - (n - 1) / 2) * pitch for k in range(n)]


def span_y(prim: Usd.Prim) -> float:
    """Total sideways width of all parallel units."""
    n = len(unit_offsets(prim))
    return size_of(prim)[1] * n + UNIT_SPACING * (n - 1)


def rebuild(prim: Usd.Prim) -> None:
    """Regenerate geometry and ports from the current size and unit count. Call after either changes.
    Parallel units (`factory:count` > 1) are identical copies side by side, centred on the item."""
    stage, item, size = prim.GetStage(), item_of(prim), size_of(prim)
    for child in ("Geo", "Ports"):
        stage.RemovePrim(prim.GetPath().AppendChild(child))
    flat = not _get(prim, NS + "show3d", True)
    kind, slots = "blackbox" if flat else item.type, int(_get(prim, PARAM + "slots", 0) or 0)
    geo = prim.GetPath().AppendChild("Geo")
    offsets = unit_offsets(prim)
    if len(offsets) == 1:
        proxies.build(stage, geo, kind, size, item.color, slots)
    else:
        UsdGeom.Xform.Define(stage, geo)
        for k, off in enumerate(offsets):
            unit = geo.AppendChild(f"Unit{k}")
            proxies.build(stage, unit, kind, size, item.color, slots)
            UsdGeom.Xformable(stage.GetPrimAtPath(unit)).AddTranslateOp().Set(Gf.Vec3d(0.0, off, 0.0))
    set_state(prim, "IDLE")
    if not item.transfer:
        for name, pos in ports(item, size).items():
            port = UsdGeom.Xform.Define(stage, prim.GetPath().AppendPath(f"Ports/{name}"))
            port.AddTranslateOp().Set(Gf.Vec3d(*pos))


def read_params(prim: Usd.Prim) -> dict:
    return {a.GetName()[len(PARAM):]: a.Get() for a in prim.GetAttributes() if a.GetName().startswith(PARAM)}


def world_port(prim: Usd.Prim, name: str) -> Gf.Vec3d:
    port = prim.GetPrimAtPath(f"Ports/{name}")
    return UsdGeom.Xformable(port).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()


def world_pos(prim: Usd.Prim) -> Gf.Vec3d:
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()


def _box_of(prim: Usd.Prim) -> Box:
    rng = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(prim).ComputeAlignedRange()
    return Box(tuple(rng.GetMin()), tuple(rng.GetMax()))


def _solids(prim: Usd.Prim, skip: tuple[str, ...] = ("Status",)) -> list[Usd.Prim]:
    """Every shape under Geo (all parallel units), leaving out markers such as the status rectangle."""
    return [p for p in Usd.PrimRange(prim.GetPrimAtPath("Geo")) if p.IsA(UsdGeom.Gprim) and not p.GetName().startswith(skip)]


def footprint(prim: Usd.Prim) -> Box:
    """World bounding box of the whole item (all units), without its floor status rectangle. For overlap checks."""
    boxes = [_box_of(p) for p in _solids(prim)]
    if not boxes:
        return _box_of(prim.GetPrimAtPath("Geo"))
    return Box(tuple(min(b.lo[i] for b in boxes) for i in range(3)), tuple(max(b.hi[i] for b in boxes) for i in range(3)))


def obstacles(prim: Usd.Prim) -> list[Box]:
    """World boxes of the solid parts (an open CNC enclosure is several boxes, not one)."""
    skip = ("Status", "Slot", "Arm", "Tool")  # markers and the gantry's moving carriage are not obstacles
    return [_box_of(p) for p in _solids(prim, skip)]


# ---- flow and run settings (stored on the plant prim) --------------------------------
def default_flow(stage: Usd.Stage) -> list:
    """With no explicit order, flow follows the layout from -X to +X."""
    return [p.GetName() for p in sorted(equipment_prims(stage), key=lambda p: world_pos(p)[0])]


def get_flow(stage: Usd.Stage) -> list:
    root = find_plant(stage)
    text = _get(root, NS + "flow", "") if root else ""
    known = {p.GetName() for p in equipment_prims(stage)}
    flow = json.loads(text) if text else default_flow(stage)
    keep = lambda s: s in known  # noqa: E731  (drop stations that were deleted from the stage)
    flow = [s if isinstance(s, str) else [x for x in s if keep(x)] for s in flow]
    flow = [s[0] if isinstance(s, list) and len(s) == 1 else s for s in flow if (keep(s) if isinstance(s, str) else s)]
    return flow + [s for s in default_flow(stage) if s not in _ids(flow)]


def set_flow(stage: Usd.Stage, flow: list) -> None:
    _set(plant(stage), NS + "flow", json.dumps(flow))


def _ids(flow: list) -> list[str]:
    return [s for step in flow for s in ([step] if isinstance(step, str) else step)]


def neighbours(flow: list, station_id: str) -> tuple[list[str], list[str]]:
    """Stations one step before and after `station_id` in the flow."""
    i = next(k for k, step in enumerate(flow) if station_id in ([step] if isinstance(step, str) else step))
    as_list = lambda s: [s] if isinstance(s, str) else list(s)  # noqa: E731
    return (as_list(flow[i - 1]) if i > 0 else []), (as_list(flow[i + 1]) if i + 1 < len(flow) else [])


def transfer_points(stage: Usd.Stage, flow: list, station_id: str) -> list[tuple[str, str, Gf.Vec3d, Gf.Vec3d]]:
    """(from id, to id, pick point, place point) for every move a robot or gantry makes."""
    prims = {p.GetName(): p for p in equipment_prims(stage)}
    before, after = neighbours(flow, station_id)
    return [(a, b, world_port(prims[a], "out_lowered"), world_port(prims[b], "in_lowered"))
            for a in before for b in after if not item_of(prims[a]).transfer and not item_of(prims[b]).transfer]


# ---- stage -> scenario ---------------------------------------------------------------
def stage_to_scenario(stage: Usd.Stage, name: str | None = None) -> dict:
    """Build the scenario dict the DES validates and runs. Config stays the source of truth:
    the result is written to YAML and that file is what gets simulated."""
    root, flow = plant(stage), get_flow(stage)
    prims = {p.GetName(): p for p in equipment_prims(stage)}
    sources = [s for s in _ids(flow) if item_of(prims[s]).type in SOURCE_TYPES]  # they make loads; not stations
    routing = [s for s in flow if not (isinstance(s, str) and s in sources)]
    stations = []
    for sid in _ids(routing):
        prim = prims[sid]
        item, size = item_of(prim), size_of(prim)
        travel = None
        if item.transfer:
            moves = transfer_points(stage, flow, sid)
            if moves:  # mean horizontal pick-to-place distance
                travel = sum(hypot(b[0] - a[0], b[1] - a[1]) for _, _, a, b in moves) / len(moves)
        etype, params = station_params(item.type, read_params(prim), size, travel)
        buffer = _get(prim, NS + "buffer", 0)
        stations.append({"id": sid, "type": etype, "count": _get(prim, NS + "count", 1),
                         "buffer": None if buffer < 0 else buffer, "params": params})
    arrival = (_get(root, NS + "arrival", "saturated") or "saturated").strip()
    plant_name = name or _get(root, NS + "plantName", "plant")
    table = _get(root, NS + "moveTable", "")
    return {
        **({"move_table": table} if table else {}),  # robot move times solved in 3D
        "name": plant_name,
        "description": "Built in the plug-and-play layout editor.",
        "cell": {"name": plant_name, "stations": stations},
        "part_types": [{"name": "load", "routing": routing}],
        # A Load source on the stage sets the arrivals; without one, the plant's arrival field does.
        "source": {"interarrival": arrival_of(read_params(prims[sources[0]])) if sources
                   else None if arrival == "saturated" else parse_dist(arrival)},
        "run": {"duration_h": _get(root, NS + "durationH", 8.0), "warmup_h": _get(root, NS + "warmupH", 0.5),
                "seed": _get(root, NS + "seed", 1), "replications": _get(root, NS + "replications", 3)},
    }


# The floor rectangle shows four states. The DES keeps its full set; SETUP counts as busy and DOWN as idle here.
STATE_COLORS = {
    "IDLE": (0.5, 0.5, 0.5), "PROCESSING": (0.15, 0.80, 0.25),
    "STARVED": (0.95, 0.80, 0.15), "BLOCKED": (0.90, 0.15, 0.15),
}
SHOWN_AS = {"SETUP": "PROCESSING", "DOWN": "IDLE"}


def auto_position(stage: Usd.Stage, type_name: str, gap: float = 0.7) -> tuple[float, float, float]:
    """Where a new catalog item lands: snapped onto the end of the line along +X.

    Conveyors and buffers abut. Machines leave a gap for a robot. A robot goes in that gap, on the
    operator side; a gantry is centred over it.
    """
    item = CATALOG[type_name]
    line = [p for p in equipment_prims(stage) if not item_of(p).transfer]
    if not line:
        return (0.0, 0.0, 0.0) if not item.transfer else (0.0, -0.55, 0.0)
    last = max(line, key=lambda p: footprint(p).hi[0])
    end = footprint(last).hi[0]
    if item.type == "robot":
        return (end + gap / 2, -0.55, 0.0)
    through = {"conveyor", "buffer"}
    spacing = 0.0 if item.type in through and item_of(last).type in through else gap
    return (end + spacing + item.size[0] / 2, 0.0, 0.0)


MOVER_TYPES = {"transfer", "robot"}  # sit in the gap between two items, not in the line itself
THROUGH = {"conveyor", "buffer"}               # these abut each other with no gap


def _world_xf(prim: Usd.Prim) -> Gf.Matrix4d:
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def yaw_of(prim: Usd.Prim) -> float:
    return float(UsdGeom.XformCommonAPI(prim).GetXformVectors(Usd.TimeCode.Default())[1][2])


def rotate(prim: Usd.Prim, degrees: float) -> None:
    """Turn an item about its own centre (Z up). Keeps it a multiple of 90 degrees."""
    api = UsdGeom.XformCommonAPI(prim)
    yaw = round((yaw_of(prim) + degrees) / 90.0) * 90.0 % 360.0
    api.SetRotate(Gf.Vec3f(0.0, 0.0, yaw))


def place_mover(prev: Usd.Prim, mover: Usd.Prim, gap: float) -> None:
    """Put a transfer / robot / gantry in the middle of the gap after `prev`, turned the same way as `prev`
    (a gantry spans along the flow). A robot stands on the operator side (-Y) of the line."""
    px = _world_xf(prev)
    fwd = Gf.Vec3d(px.TransformDir(Gf.Vec3d(1, 0, 0))).GetNormalized()
    side = Gf.Vec3d(px.TransformDir(Gf.Vec3d(0, -1, 0))).GetNormalized()
    mid = px.Transform(Gf.Vec3d(size_of(prev)[0] / 2, 0, 0)) + fwd * (gap / 2)
    if item_of(mover).type == "robot":
        mid = mid + side * 0.55
    api = UsdGeom.XformCommonAPI(mover)
    api.SetRotate(Gf.Vec3f(0.0, 0.0, yaw_of(prev)))
    api.SetTranslate(Gf.Vec3d(mid[0], mid[1], 0.0))


def place_robot(prev: Usd.Prim, nxt: Usd.Prim, robot: Usd.Prim) -> None:
    """A robot between two items: at the midpoint of prev's out node and nxt's in node, out on the operator side
    (-Y of prev) as far as it still reaches both (ur10_loop.robot_offset, from the robot layout study)."""
    from simcore.kinematics.ur10_loop import robot_offset

    pick, place = world_port(prev, "out_lowered"), world_port(nxt, "in_lowered")
    gap = Gf.Vec2d(place[0] - pick[0], place[1] - pick[1]).GetLength()
    side = Gf.Vec3d(_world_xf(prev).TransformDir(Gf.Vec3d(0, -1, 0))).GetNormalized()
    spot = (pick + place) / 2 + side * robot_offset(gap)
    api = UsdGeom.XformCommonAPI(robot)
    api.SetRotate(Gf.Vec3f(0.0, 0.0, 0.0))  # the arm's solver assumes an unturned base
    api.SetTranslate(Gf.Vec3d(spot[0], spot[1], 0.0))


def layout_chain(stage: Usd.Stage, flow: list, start: int = 1, gap: float = 0.7) -> list[str]:
    """Move items so each one's inlet edge meets the previous item's outlet edge, in front of it along the previous
    item's facing direction. Every item keeps its own rotation; a turned item is pushed forward just far enough not to
    overlap. Transfer / robot / gantry items are placed in the gap they serve. Parallel steps are left where they are.
    Returns the names of the items that were moved. `start` is the first line position to move (0 = first item)."""
    if any(isinstance(step, list) for step in flow):
        return []  # a layout with parallel steps is placed explicitly: leave it as is
    prims = {p.GetName(): p for p in equipment_prims(stage)}
    line, movers = [], {}  # line: items in order; movers[i]: transfer items between line[i-1] and line[i]
    pending = []
    for step in flow:
        if isinstance(step, list):
            continue
        if item_of(prims[step]).type in MOVER_TYPES:
            pending.append(prims[step])
        else:
            movers[len(line)] = pending
            pending = []
            line.append(prims[step])
    moved = []
    for i in range(max(start, 1), len(line)):
        prev, cur = line[i - 1], line[i]
        px, cx = _world_xf(prev), _world_xf(cur)
        fwd = Gf.Vec3d(px.TransformDir(Gf.Vec3d(1, 0, 0))).GetNormalized()
        out_pt = px.Transform(Gf.Vec3d(size_of(prev)[0] / 2, 0, 0))
        abut = item_of(prev).type in THROUGH and item_of(cur).type in THROUGH and not movers[i]
        g = 0.01 if abut else gap  # 1 cm: touching faces would read as an overlap
        # cur's footprint corners, relative to its inlet edge midpoint, in world directions (rotation only)
        sx, sy = size_of(cur)[0] / 2, span_y(cur) / 2  # sideways: all parallel units
        rot = lambda v: Gf.Vec3d(cx.TransformDir(v))  # noqa: E731
        inlet_rel = rot(Gf.Vec3d(-sx, 0, 0))
        corners = [rot(Gf.Vec3d(a, b, 0)) - inlet_rel for a in (-sx, sx) for b in (-sy, sy)]
        behind = min(Gf.Dot(c, fwd) for c in corners)  # how far it reaches back past its inlet (<= 0)
        inlet = out_pt + fwd * (g - behind)
        centre = inlet - inlet_rel
        UsdGeom.XformCommonAPI(cur).SetTranslate(Gf.Vec3d(centre[0], centre[1], 0.0))
        moved.append(cur.GetName())
        for m in movers[i]:
            if item_of(m).type == "robot" and len(movers[i]) == 1:
                place_robot(prev, cur, m)
            else:
                place_mover(prev, m, g)
            moved.append(m.GetName())
    if pending and line:  # transfer items after the last item: wait in front of it for the next one
        for m in pending:
            place_mover(line[-1], m, gap)
            moved.append(m.GetName())
    return moved


def reach_problems(stage: Usd.Stage, flow: list) -> list[str]:
    """Robots whose pick or place point is outside their reach."""
    out = []
    for prim in equipment_prims(stage):
        item = item_of(prim)
        if item.reach is None or prim.GetName() not in _ids(flow):
            continue
        base = world_pos(prim) + Gf.Vec3d(0, 0, size_of(prim)[2])
        for a, b, pick, place in transfer_points(stage, flow, prim.GetName()):
            for sid, point in ((a, pick), (b, place)):
                d = (point - base).GetLength()
                if d > item.reach:
                    out.append(f"{prim.GetName()}: {sid} is {d:.2f} m away, reach is {item.reach:.2f} m")
    return out


def transfer_problems(stage: Usd.Stage, flow: list) -> list[str]:
    """Neighbouring flow steps with no transfer between them (3D falls back to a default lift-and-place)."""
    prims = {p.GetName(): p for p in equipment_prims(stage)}
    kind = lambda sid: item_of(prims[sid]).type  # noqa: E731
    steps = [s if isinstance(s, list) else [s] for s in flow]
    out = []
    for before, after in zip(steps, steps[1:]):
        for a in before:
            for b in after:
                if kind(a) not in SOURCE_TYPES and kind(a) not in TRANSFER_TYPES and kind(b) not in TRANSFER_TYPES:
                    out.append(f"{a} -> {b}: no transfer between them (3D uses a default lift-and-place)")
    return out


def path_problems(stage: Usd.Stage, flow: list, lift: float = 0.25) -> list[str]:
    """Transfer moves whose up-across-down path passes through a solid part of any machine."""
    from simcore.kinematics import segment_hits_box

    prims = equipment_prims(stage)
    out = []
    for mover in prims:
        if not item_of(mover).transfer or mover.GetName() not in _ids(flow):
            continue
        # Shrink by 2 mm: a pick point lies exactly on a surface, and float rounding must not count as a hit.
        solids = [(p.GetName(), box.grown(-0.002)) for p in prims if p != mover for box in obstacles(p)]
        for a, b, pick, place in transfer_points(stage, flow, mover.GetName()):
            up, over = Gf.Vec3d(pick[0], pick[1], pick[2] + lift), Gf.Vec3d(place[0], place[1], place[2] + lift)
            path = [tuple(pick), tuple(up), tuple(over), tuple(place)]
            segments = list(zip(path, path[1:]))
            if item_of(mover).type == "gantry":  # its Z axis hangs from the rails, so it needs clear space above
                top = world_pos(mover)[2] + size_of(mover)[2]
                segments += [(tuple(p), (p[0], p[1], top)) for p in (pick, place)]
            hit = sorted({name for name, box in solids for p, q in segments if segment_hits_box(p, q, box)})
            if hit:
                out.append(f"{mover.GetName()}: move {a} -> {b} passes through {', '.join(hit)}")
    return out


def overlap_problems(stage: Usd.Stage, only: Usd.Prim | None = None, tolerance: float = 0.005) -> list[str]:
    """Pairs of items whose bounding boxes intersect."""
    from simcore.kinematics import boxes_overlap

    prims = equipment_prims(stage)  # a wall gantry has its own footprint: it may not overlap anything
    boxes = {p.GetName(): footprint(p) for p in prims}
    names = list(boxes)
    return [f"{a} overlaps {b}" for i, a in enumerate(names) for b in names[i + 1:]
            if (only is None or only.GetName() in (a, b)) and boxes_overlap(boxes[a], boxes[b], -tolerance)]


STATUS_OPACITY = 0.5


def _status_material(stage: Usd.Stage, state: str) -> UsdShade.Material:
    """One shared half-transparent material per shown state (RTX takes opacity from the material, not displayColor)."""
    path = f"{PLANT}/Looks/Status_{state}"
    mat = UsdShade.Material.Get(stage, path)
    if mat:
        return mat
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*STATE_COLORS[state]))
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(STATUS_OPACITY)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def set_unit_state(prim: Usd.Prim, unit: int, state: str) -> None:
    """Color one parallel unit's floor status rectangle (unit 0 when the item has a single unit)."""
    shown = SHOWN_AS.get(state, state)
    status = prim.GetPrimAtPath(f"Geo/Unit{unit}/Status") or prim.GetPrimAtPath("Geo/Status")
    if status:
        UsdGeom.Gprim(status).GetDisplayColorAttr().Set([Gf.Vec3f(*STATE_COLORS[shown])])
        UsdShade.MaterialBindingAPI.Apply(status).Bind(_status_material(prim.GetStage(), shown))


def set_state(prim: Usd.Prim, state: str) -> None:
    """Color the floor status rectangle. Playback calls this from the DES event log."""
    shown = SHOWN_AS.get(state, state)
    geo = prim.GetPrimAtPath("Geo")
    for p in (Usd.PrimRange(geo) if geo else []):  # one rectangle per parallel unit
        if p.GetName() == "Status":
            UsdGeom.Gprim(p).GetDisplayColorAttr().Set([Gf.Vec3f(*STATE_COLORS[shown])])
            UsdShade.MaterialBindingAPI.Apply(p).Bind(_status_material(prim.GetStage(), shown))
