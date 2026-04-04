from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import hypot


@dataclass(frozen=True)
class TargetObservation:
    label: str
    score: float
    pixel_u: int
    pixel_v: int
    x_mm: float
    y_mm: float


class StableTargetTracker:
    def __init__(self, required_frames: int, max_jitter_mm: float):
        self.required_frames = required_frames
        self.max_jitter_mm = max_jitter_mm
        self._history: deque[TargetObservation] = deque(maxlen=required_frames)

    def reset(self) -> None:
        self._history.clear()

    def update(self, target: TargetObservation | None) -> tuple[bool, int, TargetObservation | None]:
        if target is None:
            self.reset()
            return False, 0, None

        if self._history and target.label != self._history[-1].label:
            self.reset()

        if self._history:
            prev = self._history[-1]
            if hypot(target.x_mm - prev.x_mm, target.y_mm - prev.y_mm) > self.max_jitter_mm * 3:
                self.reset()

        self._history.append(target)
        if len(self._history) < self.required_frames:
            return False, len(self._history), target

        mean_x = sum(item.x_mm for item in self._history) / len(self._history)
        mean_y = sum(item.y_mm for item in self._history) / len(self._history)
        stable = all(
            hypot(item.x_mm - mean_x, item.y_mm - mean_y) <= self.max_jitter_mm
            for item in self._history
        )
        stable_target = TargetObservation(
            label=target.label,
            score=target.score,
            pixel_u=target.pixel_u,
            pixel_v=target.pixel_v,
            x_mm=mean_x,
            y_mm=mean_y,
        )
        return stable, len(self._history), stable_target
