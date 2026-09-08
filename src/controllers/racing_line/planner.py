"""Public-sensor outside-inside-outside target planner."""

from __future__ import annotations

from dataclasses import dataclass
from math import exp
from typing import Protocol

from controllers.centerline.policy import WALL_DANGER_THRESHOLD_M
from controllers.centerline.state import CenterlineState
from controllers.racing_line.parameters import RacingLineParameters


@dataclass(frozen=True, slots=True)
class LineTarget:
    desired_lateral_offset_m: float
    turn_direction: int
    corner_strength: float
    phase_signal: float
    entry_weight: float
    apex_weight: float
    exit_weight: float
    raw_offset_m: float
    filtered_offset_m: float
    clearance_limited: bool


class LinePlanner(Protocol):
    def reset(self) -> None: ...

    def plan(self, state: CenterlineState) -> LineTarget: ...


class RacingLinePlanner:
    """Generate a bounded local racing line without track-map access."""

    def __init__(self, parameters: RacingLineParameters) -> None:
        self.parameters = parameters
        self._last_tick: int | None = None
        self._filtered_offset_m = 0.0

    def reset(self) -> None:
        self._last_tick = None
        self._filtered_offset_m = 0.0

    def plan(self, state: CenterlineState) -> LineTarget:
        if self._last_tick is not None and state.tick <= self._last_tick:
            self.reset()
        self._last_tick = state.tick
        if not state.camera_visible or min(state.left_wall_m, state.right_wall_m, state.front_wall_m) < (
            WALL_DANGER_THRESHOLD_M
        ):
            self._filtered_offset_m = 0.0
            return LineTarget(0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False)

        p = self.parameters
        near_slope, _mid_slope, far_slope = state.lookahead_slopes
        bend = state.path_bend
        conflicting = near_slope * far_slope < 0.0 and min(abs(near_slope), abs(far_slope)) >= p.bend_activation
        turn_direction = 0 if abs(bend) < p.bend_activation or conflicting else (1 if bend > 0.0 else -1)
        corner_strength = _clamp((abs(bend) - p.bend_activation) / p.bend_saturation_width, 0.0, 1.0)
        if turn_direction == 0:
            corner_strength = 0.0
        phase_signal = _clamp(p.phase_gain * (abs(far_slope) - abs(near_slope)), -1.0, 1.0)
        entry_weight = max(phase_signal, 0.0)
        exit_weight = max(-phase_signal, 0.0)
        apex_weight = 1.0 - max(entry_weight, exit_weight)
        phase_offset = (
            p.apex_inside_fraction * apex_weight
            - p.entry_outside_fraction * entry_weight
            - p.exit_outside_fraction * exit_weight
        )
        raw_offset = turn_direction * p.max_offset_m * corner_strength * phase_offset
        alpha = 1.0 - exp(-state.dt_s / p.offset_filter_time_constant_s)
        blended = self._filtered_offset_m + alpha * (raw_offset - self._filtered_offset_m)
        max_delta = p.max_offset_rate_mps * state.dt_s
        rate_limited = self._filtered_offset_m + _clamp(blended - self._filtered_offset_m, -max_delta, max_delta)
        filtered = _clamp(rate_limited, -p.max_offset_m, p.max_offset_m)
        line_error = state.center_offset_m + filtered
        clearance = state.right_wall_m if line_error > 0.0 else state.left_wall_m
        limited_error = _clamp(
            line_error, -max(0.0, clearance - p.wall_margin_m), max(0.0, clearance - p.wall_margin_m)
        )
        limited_offset = _clamp(limited_error - state.center_offset_m, -p.max_offset_m, p.max_offset_m)
        clearance_limited = abs(limited_offset - filtered) > 1e-9
        self._filtered_offset_m = limited_offset
        return LineTarget(
            desired_lateral_offset_m=limited_offset,
            turn_direction=turn_direction,
            corner_strength=corner_strength,
            phase_signal=phase_signal,
            entry_weight=entry_weight,
            apex_weight=apex_weight,
            exit_weight=exit_weight,
            raw_offset_m=raw_offset,
            filtered_offset_m=filtered,
            clearance_limited=clearance_limited,
        )


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
