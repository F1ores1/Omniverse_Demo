"""Parametric proxy geometry: boxes with the right footprint, sized by X, Y, Z in metres.

No public asset pack has CNCs or forge presses, and a downloaded mesh can only be scaled. These
proxies are built from the size, so every dimension is a real parameter. Origin = footprint centre
on the floor; flow runs along +X; the operator/robot side is -Y.
"""
from __future__ import annotations

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from simcore.catalog import LOAD_SIZE

DARK = (0.10, 0.11, 0.13)
STEEL = (0.62, 0.64, 0.67)
# Static colliders on the solid parts, so a PhysX robot arm cannot pass through machine walls, roofs and tables.
# Floor markers and flat tiles stay non-colliding. Kinematic loads are not affected (no kinematic-static contacts).
SOLID = {"cnc", "forge_press", "inspection", "conveyor", "buffer", "robot"}
NOT_SOLID = ("Status", "Slot", "Mark", "Arrow", "Head")


def _box(stage: Usd.Stage, path: Sdf.Path, lo, hi, color, collide: bool = False) -> None:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*[(a + b) / 2 for a, b in zip(lo, hi)]))
    cube.AddScaleOp().Set(Gf.Vec3f(*[max(b - a, 1e-3) for a, b in zip(lo, hi)]))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    cube.CreateExtentAttr([(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)])
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


STATUS_MARGIN = 0.125  # m the status rectangle sticks out past the footprint, so it shows from straight above


def _status(stage: Usd.Stage, geo: Sdf.Path, x: float, y: float) -> None:
    """Flat rectangle on the floor under the item, coloured by state during playback. Same for every type."""
    hx, hy = x / 2 + STATUS_MARGIN, y / 2 + STATUS_MARGIN
    _box(stage, geo.AppendChild("Status"), (-hx, -hy, 0.0), (hx, hy, 0.004), (0.5, 0.5, 0.5))


def build(stage: Usd.Stage, geo: Sdf.Path, kind: str, size, color, slots: int = 0) -> None:
    UsdGeom.Xform.Define(stage, geo)
    x, y, z = size
    _status(stage, geo, x, y)
    hx, hy, t = x / 2, y / 2, 0.06
    solid = kind in SOLID
    b = lambda name, lo, hi, c=color: _box(stage, geo.AppendChild(name), lo, hi, c,  # noqa: E731
                                           solid and not name.startswith(NOT_SOLID))

    if kind == "cnc":  # open on the front and both sides so loads can pass along X; closed back and roof
        b("Base", (-hx, -hy, 0), (hx, hy, 0.85), STEEL)
        b("Back", (-hx, hy - t, 0.85), (hx, hy, z))
        b("Column", (-0.2, hy - 0.45, 0.85), (0.2, hy - t, z - t), STEEL)
        b("Roof", (-hx, -hy, z - t), (hx, hy, z))
        b("Spindle", (-0.1, -0.1, z - 0.6), (0.1, 0.1, z - t), DARK)
    elif kind == "forge_press":  # C-frame: column at the back, open front and sides
        b("Base", (-hx, -hy, 0), (hx, hy, 0.85), STEEL)
        b("Column", (-hx, hy - 0.4, 0.85), (hx, hy, z - 0.5))
        b("Crown", (-hx, -hy / 2, z - 0.5), (hx, hy, z))
        b("Ram", (-hx / 2, -hy / 4, z - 1.0), (hx / 2, hy / 4, z - 0.5), DARK)
    elif kind == "inspection":
        b("Table", (-hx, -hy, 0), (hx, hy, 0.85), DARK)
        b("LegL", (-hx, hy - 0.2, 0.85), (-hx + 0.12, hy - 0.05, z - 0.15))
        b("LegR", (hx - 0.12, hy - 0.2, 0.85), (hx, hy - 0.05, z - 0.15))
        b("Bridge", (-hx, hy - 0.25, z - 0.15), (hx, hy, z))
        b("Probe", (-0.03, hy - 0.2, 1.25), (0.03, hy - 0.14, z - 0.15), STEEL)
    elif kind == "conveyor":
        b("Belt", (-hx, -hy + 0.04, z - 0.08), (hx, hy - 0.04, z), DARK)
        b("RailL", (-hx, -hy, z - 0.12), (hx, -hy + 0.04, z + 0.05), STEEL)
        b("RailR", (-hx, hy - 0.04, z - 0.12), (hx, hy, z + 0.05), STEEL)
        for i, lx in enumerate((-hx + 0.1, hx - 0.1)):
            for j, ly in enumerate((-hy + 0.05, hy - 0.05)):
                b(f"Leg{i}{j}", (lx - 0.03, ly - 0.03, 0), (lx + 0.03, ly + 0.03, z - 0.12), STEEL)
    elif kind == "buffer":
        b("Top", (-hx, -hy, z - 0.05), (hx, hy, z))
        b("Stand", (-hx + 0.1, -hy + 0.1, 0), (hx - 0.1, hy - 0.1, z - 0.05), STEEL)
        for i, (sx, sy) in enumerate(slot_positions(size, slots)):
            b(f"Slot{i}", (sx - 0.09, sy - 0.09, z), (sx + 0.09, sy + 0.09, z + 0.004), DARK)
    elif kind == "gantry":  # wall gantry: posts and a horizontal beam on the far side (+Y), carriage over the line
        post, wall = 0.1, hy - 0.1
        b("PostIn", (-hx, wall, 0), (-hx + post, hy, z), STEEL)
        b("PostOut", (hx - post, wall, 0), (hx, hy, z), STEEL)
        b("Beam", (-hx, wall - 0.05, z - 0.2), (hx, hy, z), color)
        # The carriage is a separate Xform so the 3D mode can slide it along the beam (local X).
        head = geo.AppendChild("Head")
        UsdGeom.Xform.Define(stage, head).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
        _box(stage, head.AppendChild("Arm"), (-0.08, -0.05, z - 0.3), (0.08, wall - 0.05, z - 0.2), DARK)
        _box(stage, head.AppendChild("Tool"), (-0.04, -0.04, z - 0.75), (0.04, 0.04, z - 0.3), STEEL)
    elif kind == "robot":  # pedestal only; the UR10 arm is referenced on top of it by the extension
        b("Pedestal", (-hx, -hy, 0), (hx, hy, z), STEEL)
    elif kind == "transfer":  # a floor marker: the move itself is drawn by the load
        b("Plate", (-hx, -hy, 0), (hx, hy, z), color)
        b("Arrow", (-hx * 0.6, -0.04, z), (hx * 0.6, 0.04, z + 0.004), DARK)
        b("Head", (hx * 0.3, -hy * 0.5, z), (hx * 0.6, hy * 0.5, z + 0.004), DARK)
    elif kind == "load_source":  # a low pad where loads appear
        b("Pad", (-hx, -hy, 0), (hx, hy, z), color)
        b("Mark", (-hx / 3, -hy / 3, z), (hx / 3, hy / 3, z + 0.004), (0.85, 0.85, 0.90))
    else:  # blackbox: a flat 2D tile
        b("Tile", (-hx, -hy, 0), (hx, hy, 0.05), color)
        b("Border", (-hx - 0.03, -hy - 0.03, 0), (hx + 0.03, hy + 0.03, 0.02), (0.85, 0.65, 0.20))


def slot_positions(size, slots: int) -> list[tuple[float, float]]:
    """Slot centres of a buffer, in one straight line along X, from the out node back to the in node. Loads
    accumulate on exactly these spots in 3D (the buffer is a track whose stop pitch is slot_pitch)."""
    if slots <= 0:
        return []
    x = size[0]
    return [(x / 2 - LOAD_SIZE / 2 - i * slot_pitch(size, slots), 0.0) for i in range(slots)]


def slot_pitch(size, slots: int) -> float:
    """Distance between buffer slot centres: the first slot is the out node, the last one the in node."""
    return (size[0] - LOAD_SIZE) / (slots - 1) if slots > 1 else 0.0
