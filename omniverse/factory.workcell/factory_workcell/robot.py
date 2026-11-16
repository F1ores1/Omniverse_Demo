"""UR10 arm: cuMotion IK + trajectory timing for each pick-and-place loop, forward kinematics poses the links.

The solver is used twice: its trajectory duration becomes the robot's handling time in the DES
(via a move-time table), and the same trajectory is replayed in 3D. The arm is kinematic (no PhysX on it).
Collision avoidance: cuMotion's MotionPlanner crashed Kit natively (2026-11-14), so instead the loop is solved by
simcore.kinematics.ur10_loop: IK at taught-style waypoints (wait = above-pick pulled back, approach, pick, depart,
swing, place) and each candidate checked for clearance against the machines' proxy boxes. Robots stand where the
layout study puts them (simcore.usd.place_robot).
"""
from __future__ import annotations

from dataclasses import dataclass

import carb
import numpy as np
import omni.kit.app
from pxr import Gf, Sdf, Usd, UsdGeom

from simcore import usd as husd

ASSET = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2/"
         "Isaac/Robots/UniversalRobots/ur10/ur10.usd")
LOAD = 0.16
DOWN = (0.0, 1.0, 0.0, 0.0)  # w, x, y, z: tool axis pointing at the floor
GRIP_S = 0.5


def solver():
    """Enable cuMotion on first use and load the bundled UR10 description, once per process (cached on the `cumotion`
    module, so a hot reload of this file does not load it again)."""
    import sys

    cached = getattr(sys.modules.get("cumotion"), "_factory_solver", None)
    if cached is None:
        name = carb.settings.get_settings().get("/exts/factory.workcell/robot_solver_extension")
        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(name, True)
        import isaacsim.robot_motion.cumotion as cm  # puts the bundled cumotion wheel on sys.path
        import cumotion

        cached = cumotion._factory_solver = (cumotion, cm, cm.load_cumotion_supported_robot("ur10"))
    return cached


@dataclass
class Move:
    """One solved loop (simcore.kinematics.ur10_loop.NAMES): wait -> pick -> wait -> swing -> place -> back."""

    src: str
    dst: str
    trajectory: object     # cumotion.Trajectory
    duration: float        # s of motion
    t_pick: float          # s into the motion when the tool reaches the pick point
    t_place: float
    speed: float
    pick: Gf.Vec3d | None = None    # the source's out node the tool was solved to touch
    place: Gf.Vec3d | None = None   # the destination's in node
    t_clear: float | None = None    # s of motion when the arm is back at wait, out of the source machine
    turn: float = 0.0               # rad added to the base joint (whole turns), so all of a robot's loops line up

    @property
    def cycle_s(self) -> float:
        return self.duration + 2 * GRIP_S

    def q(self, cycle_time: float) -> np.ndarray:
        """Joint positions at a time in the cycle. The arm pauses GRIP_S at pick and at place."""
        t = cycle_time
        if t > self.t_pick:
            t = max(self.t_pick, t - GRIP_S)
        if t > self.t_place:
            t = max(self.t_place, t - GRIP_S)
        t = min(max(t, 0.0), self.duration)
        q = _eval(self.trajectory, t)
        if self.turn:
            q = q.copy()
            q[0] += self.turn
        return q

    def holding(self, cycle_time: float) -> bool:
        return self.t_pick <= cycle_time <= self.t_place + GRIP_S


class Arm:
    """The UR10 on one robot pedestal."""

    def __init__(self, prim: Usd.Prim) -> None:
        self.prim = prim
        self.cumotion, self.cm, self.robot = solver()
        self.kin = self.robot.kinematics
        self.tool = self.robot.robot_description.tool_frame_names()[0]
        self.moves: dict[tuple[str, str], Move] = {}
        self.error: str | None = None
        self._links: dict[str, UsdGeom.Xformable] = {}
        self.base = self._base_matrix()

    def _base_matrix(self) -> Gf.Matrix4d:
        """World transform of the arm's base frame: the top of the pedestal."""
        world = UsdGeom.Xformable(self.prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, husd.size_of(self.prim)[2])) * world

    # ---- visuals ---------------------------------------------------------------------
    def attach(self) -> None:
        """Reference the UR10 asset on the pedestal and find its link prims."""
        stage = self.prim.GetStage()
        path = self.prim.GetPath().AppendChild("Arm")
        arm = stage.GetPrimAtPath(path)
        if not arm.IsValid():
            xform = UsdGeom.Xform.Define(stage, path)
            xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, husd.size_of(self.prim)[2]))
            arm = xform.GetPrim()
            arm.GetReferences().AddReference(ASSET)
        self._kinematic(arm)
        frames = set(self.kin.frame_names())
        self._links = {p.GetName(): UsdGeom.Xformable(p) for p in Usd.PrimRange(arm)
                       if p.GetName() in frames and p.IsA(UsdGeom.Xformable) and not p.IsInstanceProxy()}
        if not self._links:
            self.error = f"{self.prim.GetName()}: UR10 asset did not load or has no link prims named like the URDF"
        # The asset's link origins are not the URDF's. Keep each link's offset from its URDF frame, measured once
        # in the asset's rest pose (all joints at zero), so forward kinematics can place the meshes.
        rest_q, from_world = np.zeros(self.kin.num_cspace_coords()), self.base.GetInverse()
        self._offset = {}
        for name, link in self._links.items():
            attr = link.GetPrim().GetAttribute("factory:restPose")
            if not attr:
                attr = link.GetPrim().CreateAttribute("factory:restPose", Sdf.ValueTypeNames.Matrix4d)
                attr.Set(link.ComputeLocalToWorldTransform(Usd.TimeCode.Default()) * from_world)
            self._offset[name] = attr.Get() * _to_gf(self.kin.pose(rest_q, name)).GetInverse()

    @staticmethod
    def _kinematic(arm: Usd.Prim) -> None:
        """The arm is posed by forward kinematics only (like Emulate3D: joint values straight from the motion
        profile). The asset is a PhysX articulation that would take the links over on Play and drive every joint to
        0, so its articulation, joints and bodies are switched off."""
        from pxr import UsdPhysics

        for p in Usd.PrimRange(arm):
            if p.IsInstanceProxy():
                continue
            if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr().Set(False)
            if p.IsA(UsdPhysics.Joint):
                UsdPhysics.Joint(p).CreateJointEnabledAttr().Set(False)

    def pose(self, q: np.ndarray) -> None:
        """Place every link from forward kinematics."""
        for name, link in self._links.items():
            world = self._offset[name] * _to_gf(self.kin.pose(q, name)) * self.base
            parent = link.ComputeParentToWorldTransform(Usd.TimeCode.Default())
            link.MakeMatrixXform().Set(world * parent.GetInverse())

    def home(self, face: Gf.Vec3d | None = None) -> None:
        """Home pose: upper arm straight up, forearm level and pointing out, tool pointing down
        (UR ready pose [0, -90, 90, -90, -90, 0] deg), the base turned so the forearm points at `face` (the pick
        point). Not the asset's all-zero pose, where the arm lies flat."""
        n = self.kin.num_cspace_coords()
        q = np.zeros(n)
        q[1], q[2], q[3], q[4] = -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2
        if face is not None:
            local = self.base.GetInverse().Transform(Gf.Vec3d(*face))
            reach = self.kin.position(np.zeros(n), self.tool)  # the arm's heading with the base joint at 0
            q[0] = float(np.arctan2(local[1], local[0]) - np.arctan2(reach[1], reach[0]))
        self.home_q = q
        self.pose(q)

    def transit(self, q_from, q_to):
        """Time-optimal joint move between two poses (e.g. one loop's wait pose to another's), or None when the arm
        is already there. Both ends are wait poses, pulled back out of the machines."""
        q_from, q_to = np.asarray(q_from, dtype=np.float64), np.asarray(q_to, dtype=np.float64)
        if np.allclose(q_from, q_to, atol=1e-3):
            return None
        if getattr(self, "_gen", None) is None:
            from simcore.kinematics.ur10_loop import LoopSolver

            self._gen = LoopSolver(self.cumotion, self.kin, self.tool).cgen
        return self._gen.generate_trajectory([q_from, q_to])

    def line_up(self) -> None:
        """Shift each loop by whole turns of the base joint so its wait pose is nearest the first loop's: a transit
        between two loops then never spins the arm a full turn."""
        moves = list(self.moves.values())
        if len(moves) < 2:
            return
        ref = moves[0].q(0.0)[0]
        for m in moves[1:]:
            k = round((ref - m.q(0.0)[0]) / (2 * np.pi))
            turned = m.q(0.0)[0] + 2 * np.pi * k
            if k and abs(turned) <= 2 * np.pi:  # UR10 base joint limit
                m.turn = 2 * np.pi * k

    def tool_world(self, q: np.ndarray) -> Gf.Vec3d:
        return self.base.Transform(Gf.Vec3d(*[float(v) for v in self.kin.position(q, self.tool)]))

    # ---- solve -----------------------------------------------------------------------
    def _obstacles(self) -> tuple[np.ndarray, np.ndarray]:
        """World boxes (lo, hi) of every solid proxy box except this robot's own pedestal, plus the floor."""
        from pxr import UsdPhysics

        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        lo, hi = [(-1e3, -1e3, -1.0)], [(1e3, 1e3, 0.0)]  # floor
        for item in husd.equipment_prims(self.prim.GetStage()):
            if item == self.prim:
                continue
            for p in Usd.PrimRange(item.GetPrimAtPath("Geo")):
                if p.IsA(UsdGeom.Cube) and p.HasAPI(UsdPhysics.CollisionAPI):
                    r = cache.ComputeWorldBound(p).ComputeAlignedRange()
                    lo.append(tuple(r.GetMin()))
                    hi.append(tuple(r.GetMax()))
        return np.array(lo), np.array(hi)

    def solve(self, src: str, dst: str, pick: Gf.Vec3d, place: Gf.Vec3d, lift: float = 0.25,
              speed: float = 1.0) -> Move | None:
        """Plan the loop for one pick node and one place node with simcore.kinematics.ur10_loop (the same solver
        the headless layout study uses). Returns None if it cannot be reached. `lift` is kept for callers; the
        solver picks the approach height itself."""
        from simcore.kinematics.ur10_loop import CLEAR_LEG, PICK_LEG, PLACE_LEG, LoopSolver

        lo, hi = self._obstacles()
        base = self.base.ExtractTranslation()
        loop = LoopSolver(self.cumotion, self.kin, self.tool, speed).solve(
            [base[0], base[1], base[2]], list(pick), list(place), lo, hi)
        if loop is None:
            self.error = f"{self.prim.GetName()}: cannot reach the move {src} -> {dst}"
            return None
        if not loop.ok:
            self.error = (f"{self.prim.GetName()}: no collision-free move {src} -> {dst} "
                          f"(closest gap {loop.clearance * 100:.0f} cm); using the best one")
        chain = Chain(loop.legs)
        move = Move(src, dst, chain, loop.cycle_s, chain.starts[PICK_LEG + 1], chain.starts[PLACE_LEG + 1], speed,
                    Gf.Vec3d(pick), Gf.Vec3d(place), chain.starts[CLEAR_LEG + 1])
        self.moves[(src, dst)] = move
        return move


class _Span:
    def __init__(self, lower: float, upper: float) -> None:
        self.lower, self.upper = lower, upper

    def span(self) -> float:
        return self.upper - self.lower


class Chain:
    """Timed legs played one after another, with the same domain() / eval_all() calls as a cumotion.Trajectory."""

    def __init__(self, legs: list) -> None:
        self.legs, self.starts, t = legs, [], 0.0
        for leg in legs:
            self.starts.append(t)
            t += leg.domain().span()
        self._domain = _Span(0.0, t)

    def domain(self) -> _Span:
        return self._domain

    def eval_all(self, t: float):
        i = max(k for k, s in enumerate(self.starts) if s <= t or k == 0)
        leg = self.legs[i]
        d = leg.domain()
        return leg.eval_all(min(d.lower + t - self.starts[i], d.upper))


def _eval(trajectory, t: float) -> np.ndarray:
    """Joint positions t seconds into a trajectory, clamped to its domain."""
    domain = trajectory.domain()
    return np.asarray(trajectory.eval_all(min(max(domain.lower + t, domain.lower), domain.upper))[0], dtype=np.float64)


def _to_gf(pose) -> Gf.Matrix4d:
    """cuMotion Pose3 (column-vector convention) -> Gf.Matrix4d (row-vector convention)."""
    r, t = np.asarray(pose.rotation.matrix()), np.asarray(pose.translation).flatten()
    m = Gf.Matrix4d(1.0)
    m.SetRotateOnly(Gf.Matrix3d(*[float(v) for v in r.T.flatten()]))
    m.SetTranslateOnly(Gf.Vec3d(float(t[0]), float(t[1]), float(t[2])))
    return m


def write_table(path, arms: dict[str, "Arm"]) -> dict:
    """Save solved moves as a move-time table the DES can use without any 3D."""
    import yaml

    table = {"solver": "cuMotion time-optimal trajectory, UR10", "grip_s": GRIP_S, "stations": {}}
    for sid, arm in arms.items():
        if not arm.moves:
            continue
        moves = list(arm.moves.values())
        table["stations"][sid] = {
            "cycle_s": round(sum(m.cycle_s for m in moves) / len(moves), 3),  # mean over this robot's moves
            "moves": [{"from": m.src, "to": m.dst, "speed": m.speed, "seconds": round(m.cycle_s, 3)} for m in moves],
        }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("# Robot move times solved in 3D. The DES uses cycle_s as the robot handling time.\n"
                    + yaml.safe_dump(table, sort_keys=False), encoding="utf-8")
    return table
