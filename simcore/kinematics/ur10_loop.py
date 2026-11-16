"""One robot's pick-and-place loop, solved with cuMotion and checked for clearance against the machine boxes.

Shared by the Kit extension (robot.py, which plays the loop in 3D) and the headless layout study
(scripts/robot_layout_study.py), so the pose chosen offline is the pose shown in Composer. The cuMotion module and
its UR10 kinematics are passed in; nothing here imports Kit.

Loop (taught-point style: approach and depart points outside the machines):
    wait -> above pick -> pick -> above pick -> wait -> short of place -> above place -> place -> above place
    -> short of place -> wait
"wait" is the above-pick pose pulled straight back toward the robot until the whole arm keeps WAIT_CLEAR from every
machine box: the same posture, stopped short. "short of place" is the same for the place. The swing with the load
runs only between the two stopped-short poses, closer to the base, so its arc stays out of the machines (a full-reach
swing between two points on the line centre bulges past the centre line into the machines' backs). The robot waits at
"wait", so the next robot can use the machine and the first pick has no jump.
Candidates (approach height x base turn x elbow posture) are ranked by the smallest clearance along the whole loop
(capped at CLEAR_ENOUGH), then by cycle time.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .arm_clearance import LOAD_R, ArmBody, clearance

LOAD = 0.16          # m, tool flange above the node when gripping
DOWN = (0.0, 1.0, 0.0, 0.0)  # w, x, y, z: tool pointing at the floor
NEED = 0.03          # m of clearance a loop must keep everywhere to count as collision-free
WAIT_CLEAR = 0.10    # m the wait pose keeps from every machine box
CLEAR_ENOUGH = 0.3   # m: more clearance than this does not make a loop better
DT = 0.05            # s between clearance samples
UPS = (0.25, 0.15, 0.08)  # m, approach heights above the node (low ones fit under a press ram / CNC spindle)
NAMES = ["wait", "above_pick", "pick", "above_pick", "wait", "short_of_place", "above_place", "place", "above_place",
         "short_of_place", "wait"]
# Pedestal placement, from scripts/robot_layout_study.py (output/reports/robot_study.json): the robot stands at the
# midpoint of its pick and place nodes, out to the operator side as far as it still reaches both nodes (REACH,
# horizontal, nodes at table height), at most MAX_OUT. Closer in, the arm folds its elbow up into press crowns and CNC
# roofs; further out, it cannot reach machine centres.
REACH = 1.25    # m, horizontal distance from the arm's base to a node it can still serve with margin
MAX_OUT = 1.0   # m, furthest a pedestal stands out from the line


def robot_offset(node_gap: float) -> float:
    """How far out from the line (sideways) the pedestal stands, for nodes `node_gap` apart."""
    return float(min(MAX_OUT, np.sqrt(max(REACH ** 2 - (node_gap / 2) ** 2, 0.0))))


EXTRA_BACK = (0.0, 0.2, 0.4)  # m pulled back beyond "just clear", so the swing arc stays out of the machines
CARRY_LEGS = (3, 4, 5)  # legs where the load hangs in the air (not the ones lifting it off or setting it down)
PICK_LEG, PLACE_LEG, CLEAR_LEG = 1, 6, 3  # leg ending at the pick / at the place; leg ending back at "wait"


@dataclass
class Loop:
    legs: list                     # one cumotion trajectory per waypoint pair
    waypoints: list[np.ndarray]    # joint vectors, NAMES order
    cycle_s: float                 # s of motion (no grip pauses)
    clearance: float               # m, smallest gap to a machine box along the loop (negative = collision)
    closest_box: int               # index into the boxes given
    worst_leg: int
    approach_up: float
    wait_back: float               # m the wait pose stops short of the pick, horizontally
    starts: list[float] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.clearance >= NEED


class LoopSolver:
    def __init__(self, cumotion, kin, tool: str, speed: float = 1.0) -> None:
        self.cm, self.kin, self.tool = cumotion, kin, tool
        self.body = ArmBody(kin)
        n = kin.num_cspace_coords()
        self.cgen = cumotion.create_cspace_trajectory_generator(kin)
        self.cgen.set_velocity_limits(np.array([kin.cspace_coord_velocity_limit(i) for i in range(n)]) * speed)
        self.cgen.set_acceleration_limits(np.array([kin.cspace_coord_acceleration_limit(i) for i in range(n)]) * speed)

    def _ik(self, target: np.ndarray, seed: np.ndarray):
        cfg = self.cm.IkConfig()
        cfg.cspace_seeds = [seed]
        pose = self.cm.Pose3(translation=np.asarray(target, dtype=np.float64), rotation=self.cm.Rotation3(*DOWN))
        r = self.cm.solve_ik(self.kin, pose, self.tool, cfg)
        return np.array(r.cspace_position) if r.success else None

    def _clear(self, q, base, lo, hi):
        return clearance(self.body.points(q) + base, lo, hi)

    def solve(self, base, pick, place, boxes_lo, boxes_hi) -> Loop | None:
        """base: arm base (pedestal top centre) in world; pick / place: nodes in world; boxes: (B, 3) lo / hi corners
        of every machine box (not this robot's own pedestal). Arm base frame = world translated to `base` (no yaw)."""
        base, pick, place = (np.asarray(v, dtype=np.float64) for v in (base, pick, place))
        lo, hi = np.asarray(boxes_lo, dtype=np.float64), np.asarray(boxes_hi, dtype=np.float64)
        local = lambda p, up: p - base + np.array([0.0, 0.0, LOAD + up])  # noqa: E731
        ang = lambda p: float(np.arctan2(p[1] - base[1], p[0] - base[0]))  # noqa: E731
        swing = (ang(place) - ang(pick) + np.pi) % (2 * np.pi) - np.pi
        best, half = None, np.pi / 2
        # The swing can go the short way round (over the line, where the machines are) or the long way (through the
        # operator side, open floor). Both are tried; the clearance check decides.
        ways = (swing, swing - 2 * np.pi * np.sign(swing))
        for up in UPS:
            seen = []
            for offset in (0.0, half, -half, np.pi):
                for sh, el in ((-1.2, 1.6), (-1.9, -1.6), (-0.8, 2.2)):
                    seed = np.array([ang(pick) + offset, sh, el, -(sh + el) - half, -half, 0.0])
                    above = self._ik(local(pick, up), seed)
                    if above is None or any(np.allclose(above, s, atol=0.05) for s in seen):
                        continue  # many seeds land on the same posture
                    seen.append(above)
                    for extra in EXTRA_BACK:
                        pick_side = self._side(pick, above, up, base, lo, hi, local, extra)
                        if pick_side is None:
                            continue
                        for way in ways:
                            turned = above.copy()
                            turned[0] += way
                            place_above = self._ik(local(place, up), turned)
                            place_side = None if place_above is None else self._side(
                                place, place_above, up, base, lo, hi, local, extra)
                            if place_side is None:
                                continue
                            loop = self._loop(pick_side, place_side, above[0] + way, base, lo, hi)
                            if loop is not None:
                                loop.approach_up, loop.wait_back = up, pick_side[3]
                                if best is None or self._score(loop) > self._score(best):
                                    best = loop
        return best

    def _side(self, node, above, up, base, lo, hi, local, extra=0.0):
        """(above, at, short_of, metres pulled back) for one node, or None when any of them can't be reached.
        `extra`: how much further than just clear of the machine the arm pulls back before swinging."""
        at = self._ik(local(node, 0.0), above)
        short, back = self._short_of(node, above, up, base, lo, hi, local, extra)
        return None if at is None or short is None else (above, at, short, back)

    def _loop(self, pick_side, place_side, place_base_angle, base, lo, hi) -> Loop | None:
        """Chain the waypoints, with the place side's base joint shifted by whole turns so the swing goes the way
        asked (short way over the line, or long way through the operator side)."""
        ap, at_p, wait, _ = pick_side
        turns = 2 * np.pi * np.round((place_base_angle - place_side[2][0]) / (2 * np.pi))
        apl, at_pl, short = (q + np.r_[turns, np.zeros(len(q) - 1)] for q in place_side[:3])
        if any(abs(q[0]) > 2 * np.pi for q in (apl, at_pl, short)):  # UR10 base joint limit
            return None
        return self._evaluate([wait, ap, at_p, ap, wait, short, apl, at_pl, apl, short, wait], base, lo, hi)

    def _short_of(self, node, above, up, base, lo, hi, local, extra=0.0):
        """`above` (the pose over a node) pulled straight back toward the robot, same height and posture, until the
        whole arm keeps WAIT_CLEAR from every machine box, then `extra` further. Returns (joints, metres pulled
        back) or (None, None)."""
        back = np.array([base[0] - node[0], base[1] - node[1], 0.0])
        back /= max(np.linalg.norm(back), 1e-9)
        for s in np.arange(0.1, 1.3, 0.05):
            q = self._ik(local(node + back * s, up), above)
            if q is not None and self._clear(q, base, lo, hi)[0] >= WAIT_CLEAR:
                if extra:
                    q = self._ik(local(node + back * (s + extra), up), q)
                    if q is None or self._clear(q, base, lo, hi)[0] < WAIT_CLEAR:
                        return None, None
                return q, float(s + extra)
        return None, None

    @staticmethod
    def _score(loop: Loop) -> tuple:
        return (min(loop.clearance, CLEAR_ENOUGH), -loop.cycle_s)

    def _evaluate(self, way, base, lo, hi) -> Loop | None:
        legs, starts, total = [], [], 0.0
        worst, worst_box, worst_leg = np.inf, -1, -1
        for i, (a, b) in enumerate(zip(way, way[1:])):
            traj = self.cgen.generate_trajectory([a, b])
            if traj is None:
                return None
            dom = traj.domain()
            for t in np.arange(dom.lower, dom.upper + 1e-9, DT):
                q = np.asarray(traj.eval_all(t)[0], dtype=np.float64)
                c, k = self._clear(q, base, lo, hi)
                if i in CARRY_LEGS:  # the load in the air must clear the machines too
                    cl, kl = clearance(self.body.load_point(q)[None] + base, lo, hi, LOAD_R)
                    if cl < c:
                        c, k = cl, kl
                if c < worst:
                    worst, worst_box, worst_leg = c, k, i
            legs.append(traj)
            starts.append(total)
            total += dom.upper - dom.lower
        return Loop(legs, list(way), total, float(worst), int(worst_box), worst_leg, 0.0, 0.0, starts)
