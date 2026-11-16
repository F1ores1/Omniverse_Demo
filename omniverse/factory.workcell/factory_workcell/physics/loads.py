"""Loads in 3D mode: rigid bodies that switch between dynamic and kinematic.

Dynamic: PhysX moves them (belt friction carries them, other loads stop them, gravity drops them).
Kinematic: code moves them exactly along a transfer's motion profile (robots, gantries); no gravity, no friction.
"""
from __future__ import annotations

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

LOADS = "/Plant/Loads3D"
from simcore.catalog import LOAD_SIZE  # m, edge of a box / height and diameter of a cylinder; ports use it too

NODE_TOL = 0.02  # m: a load must sit this close to the pick node for a robot or transfer to grip it


def off_node(prim, node) -> float:
    """How far a load is from resting on a pick node (the node is the surface point under the load's centre)."""
    from pxr import Gf

    return (world_position(prim) - (Gf.Vec3d(node) + Gf.Vec3d(0, 0, LOAD_SIZE / 2 + 0.002))).GetLength()
LOAD_MASS = 2.0   # kg
LOAD_COLOR = (0.95, 0.55, 0.10)


def physics_material(stage: Usd.Stage, friction: float) -> UsdShade.Material:
    """A physics material with equal static and dynamic friction and no bounce. Shared per friction value."""
    path = f"/Plant/Looks/Friction_{str(round(friction, 3)).replace('.', '_')}"
    mat = UsdShade.Material.Get(stage, path)
    if mat:
        return mat
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(friction)
    api.CreateDynamicFrictionAttr(friction)
    api.CreateRestitutionAttr(0.0)
    return mat


def bind_physics_material(prim: Usd.Prim, mat: UsdShade.Material) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")


def spawn(stage: Usd.Stage, name: str, shape: str, position: Gf.Vec3d, material: UsdShade.Material) -> Usd.Prim:
    """A dynamic load resting with its bottom at `position`."""
    UsdGeom.Scope.Define(stage, LOADS)
    path = f"{LOADS}/{name}"
    half = LOAD_SIZE / 2
    if shape == "cylinder":
        geom = UsdGeom.Cylinder.Define(stage, path)
        geom.CreateAxisAttr(UsdGeom.Tokens.z)
        geom.CreateRadiusAttr(half)
        geom.CreateHeightAttr(LOAD_SIZE)
        geom.CreateExtentAttr([(-half, -half, -half), (half, half, half)])
    else:
        geom = UsdGeom.Cube.Define(stage, path)
        geom.CreateSizeAttr(LOAD_SIZE)
        geom.CreateExtentAttr([(-half, -half, -half), (half, half, half)])
    geom.CreateDisplayColorAttr([Gf.Vec3f(*LOAD_COLOR)])
    geom.AddTranslateOp().Set(Gf.Vec3d(position[0], position[1], position[2] + half + 0.002))
    geom.AddOrientOp().Set(Gf.Quatf(1.0))
    prim = geom.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(False)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(LOAD_MASS)
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr(0.0)  # never sleep: react when the belt starts
    bind_physics_material(prim, material)
    return prim


def set_kinematic(prim: Usd.Prim, kinematic: bool) -> None:
    """Switch a load between dynamic (PhysX moves it) and kinematic (a transfer moves it). Works while playing."""
    UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(kinematic)


def to_kinematic(prim: Usd.Prim) -> None:
    set_kinematic(prim, True)


def to_dynamic(prim: Usd.Prim) -> None:
    set_kinematic(prim, False)


def set_world_position(prim: Usd.Prim, world: Gf.Vec3d) -> None:
    """Move a kinematic load (loads live directly under LOADS, so local = world)."""
    UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Set(Gf.Vec3d(world))


def world_position(prim: Usd.Prim) -> Gf.Vec3d:
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()


def remove_all(stage: Usd.Stage) -> None:
    if stage.GetPrimAtPath(LOADS):
        stage.RemovePrim(Sdf.Path(LOADS))
