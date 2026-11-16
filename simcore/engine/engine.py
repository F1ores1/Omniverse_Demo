"""Discrete-event engine: a clock and a time-ordered queue of callbacks."""
from __future__ import annotations

import heapq
from itertools import count
from typing import Callable


class Engine:
    def __init__(self) -> None:
        self.now = 0.0
        self._queue: list[tuple[float, int, Callable, tuple]] = []
        self._seq = count()  # tie-break so same-time events run in schedule order

    def schedule(self, delay: float, fn: Callable, *args) -> None:
        heapq.heappush(self._queue, (self.now + delay, next(self._seq), fn, args))

    def run(self, until: float) -> None:
        while self._queue and self._queue[0][0] <= until:
            self.now, _, fn, args = heapq.heappop(self._queue)
            fn(*args)
        self.now = until
