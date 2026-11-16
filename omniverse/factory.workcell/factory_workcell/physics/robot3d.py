"""3D robot transfer: the UR10 runs its solved cuMotion loops and carries the load on its tool.

Same hand-over as every transfer (src.release -> dst.accept). The arm is kinematic: each step the joint values are
read from the solved trajectory and the links are posed from them (forward kinematics), as Emulate3D does. The load is
attached kinematically to the tool: it takes the tool position from the same joint values, so it cannot drift, and
fast forward just reads the trajectory faster.

A robot can serve a parallel step: several sources and/or several destinations, one solved loop per pair. Each cycle
it takes a ready source (round robin) and the least-loaded destination that has room. When the chosen loop starts from
another wait pose than where the arm is, the arm first makes a timed transit there (no jump).

Cycle (Move.q time): wait -> down to pick, grip -> back to wait -> swing -> place, release -> back to wait.
"""
from __future__ import annotations

import carb
import numpy as np
from pxr import Gf, Usd

from ..robot import GRIP_S
from . import loads


def _load_of(item) -> int:
    """How many loads an item holds or is working on (to send work to the idlest parallel machine)."""
    if hasattr(item, "on_unit"):
        return sum(u is not None for u in item.on_unit) + len(getattr(item, "queue", [])) + len(item.departing)
    if hasattr(item, "track"):
        return len(item.track.loads)
    return 0


class RobotTransfer3D:
    def __init__(self, srcs, dsts, prim: Usd.Prim, arm) -> None:
        self.srcs, self.dsts = (list(srcs) if isinstance(srcs, (list, tuple)) else [srcs],
                                list(dsts) if isinstance(dsts, (list, tuple)) else [dsts])
        self.prim, self.arm, self.id = prim, arm, prim.GetName()
        self.src = self.dst = self.move = None
        self.load: Usd.Prim | None = None
        self.t = 0.0
        self.pre = 0.0         # s of transit before the loop starts (0 when the arm is already at its wait pose)
        self._transit = None
        self.busy = False      # in a cycle
        self.lifted = False
        self.done = False      # at the place point, waiting for room
        self._grip: Gf.Vec3d | None = None     # load minus tool position, fixed at grip
        self.fault: str | None = None          # set when a load is not on the pick node; the robot stops (DOWN)
        self._turn = 0                         # round robin over sources
        self.q_now = getattr(arm, "home_q", None)

    # ---- choosing the next move ----------------------------------------------------------
    def _choose(self):
        open_dsts = [d for d in self.dsts if d.can_accept()]
        if not open_dsts:
            return None
        n = len(self.srcs)
        for k in range(n):
            src = self.srcs[(self._turn + k) % n]
            if src.ready_at_out():
                dst = min(open_dsts, key=_load_of)  # ties: first listed
                self._turn = (self._turn + k + 1) % n
                return src, dst
        return None

    def _start(self, src, dst) -> bool:
        move = self.arm.moves.get((src.id, dst.id))
        if move is None:
            return False
        miss = loads.off_node(src.front(), move.pick)
        if miss > loads.NODE_TOL:  # the tool would not touch the load: never grip from a distance
            self.fault = f"{self.id}: load on {src.id} is {miss * 100:.0f} cm off the pick node, robot stopped"
            carb.log_error(self.fault)
            return False
        self.src, self.dst, self.move = src, dst, move
        start = move.q(0.0)
        self._transit = None if self.q_now is None else self.arm.transit(self.q_now, start)
        self.pre = 0.0 if self._transit is None else float(self._transit.domain().span())
        self.t_grip = self.pre + move.t_pick + GRIP_S                 # load is in the gripper
        self.t_release = self.pre + move.t_place + 2 * GRIP_S         # load is put down
        # the source is free for the next robot only once this arm is back out of it, at its wait pose
        self.t_lifted = self.pre + ((move.t_clear + GRIP_S) if move.t_clear is not None else move.t_pick + GRIP_S + 0.3)
        self.t_end = self.pre + move.cycle_s
        self.load = src.release()
        loads.to_kinematic(self.load)
        self.t, self.busy, self.lifted, self.done, self._grip = 0.0, True, False, False, None
        return True

    def _q(self, t: float) -> np.ndarray:
        if t < self.pre and self._transit is not None:
            d = self._transit.domain()
            return np.asarray(self._transit.eval_all(d.lower + t)[0], dtype=np.float64)
        return self.move.q(min(t - self.pre, self.move.cycle_s))

    # ---- stepping ----------------------------------------------------------------------
    def step(self, dt: float) -> list[tuple[str, str, str]]:
        events = []
        if self.fault:
            return events
        if not self.busy:
            pick = self._choose()
            if pick and self._start(*pick):
                events.append((self.load.GetName(), self.src.id, "leave"))
            return events
        if self.load is not None and self.t >= self.t_release - 1e-9:  # at the place point
            self.done = True
            if not self.dst.can_accept():
                return events  # hold the load until there is room
            name = self.load.GetName()
            self.dst.accept(self.load)
            self.load, self.done = None, False
            events.append((name, self.dst.id, "enter"))
        self.t += dt
        q = self._q(min(self.t, self.t_end))
        self.arm.pose(q)
        self.q_now = q
        if self.load is not None and self.t >= self.t_grip:  # riding on the tool
            tool = self.arm.tool_world(q)
            if self._grip is None:  # keep where the load sat relative to the tool when gripped: no jump onto it
                self._grip = loads.world_position(self.load) - tool
            loads.set_world_position(self.load, tool + self._grip)
        if not self.lifted and self.t >= self.t_lifted:
            self.lifted = True
            self.src.cleared()
        if self.t >= self.t_end:  # back at the wait pose: ready for the next load
            self.busy = False
            self.q_now = self.move.q(self.move.cycle_s)
            self.arm.pose(self.q_now)
        return events

    def state(self) -> str:
        if self.fault:
            return "DOWN"
        if not self.busy:
            return "STARVED"
        return "BLOCKED" if self.done else "PROCESSING"

    def reset(self) -> None:
        self.fault = None
        self.q_now = getattr(self.arm, "home_q", None)
        if self.q_now is not None:
            self.arm.pose(self.q_now)
