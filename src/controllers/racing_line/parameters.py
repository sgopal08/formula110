"""Immutable authored and optimizable racing-line planner parameters."""

from __future__ import annotations

from dataclasses import astuple, dataclass
from math import isfinite
from typing import Final


@dataclass(frozen=True, slots=True)
class RacingLineParameters:
    max_offset_m: float
    bend_activation: float
    bend_saturation_width: float
    phase_gain: float
    entry_outside_fraction: float
    apex_inside_fraction: float
    exit_outside_fraction: float
    offset_filter_time_constant_s: float
    max_offset_rate_mps: float
    wall_margin_m: float

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in astuple(self)):
            raise ValueError("racing-line parameters must be finite")
        if self.max_offset_m < 0.0 or self.bend_activation < 0.0 or self.bend_saturation_width <= 0.0:
            raise ValueError("offset and bend parameters must be nonnegative with positive saturation width")
        if self.phase_gain < 0.0:
            raise ValueError("phase gain cannot be negative")
        if min(self.entry_outside_fraction, self.apex_inside_fraction, self.exit_outside_fraction) < 0.0:
            raise ValueError("phase fractions cannot be negative")
        if self.offset_filter_time_constant_s <= 0.0 or self.max_offset_rate_mps <= 0.0:
            raise ValueError("offset filter and rate limit must be positive")
        if self.wall_margin_m <= 0.0:
            raise ValueError("wall margin must be positive")


AUTHORED_RACING_LINE_PARAMETERS: Final = RacingLineParameters(
    max_offset_m=1.20,
    bend_activation=0.08,
    bend_saturation_width=0.55,
    phase_gain=3.0,
    entry_outside_fraction=0.80,
    apex_inside_fraction=1.00,
    exit_outside_fraction=0.65,
    offset_filter_time_constant_s=0.15,
    max_offset_rate_mps=3.0,
    wall_margin_m=1.60,
)

RACING_LINE_PARAMETER_NAMES: Final = tuple(RacingLineParameters.__dataclass_fields__)
RACING_LINE_PARAMETER_BOUNDS: Final = (
    (0.20, 1.60),
    (0.02, 0.25),
    (0.15, 1.20),
    (0.50, 8.00),
    (0.00, 1.25),
    (0.00, 1.25),
    (0.00, 1.25),
    (0.03, 0.50),
    (0.50, 8.00),
    (1.35, 2.50),
)
RACING_LINE_PARAMETER_COUNT: Final = len(RACING_LINE_PARAMETER_NAMES)


def racing_line_to_normalized(parameters: RacingLineParameters) -> tuple[float, ...]:
    return tuple(
        2.0 * (value - lower) / (upper - lower) - 1.0
        for value, (lower, upper) in zip(astuple(parameters), RACING_LINE_PARAMETER_BOUNDS, strict=True)
    )


def racing_line_from_normalized(values: tuple[float, ...] | list[float]) -> RacingLineParameters:
    if len(values) != RACING_LINE_PARAMETER_COUNT:
        raise ValueError(f"expected {RACING_LINE_PARAMETER_COUNT} normalized planner parameters, got {len(values)}")
    physical = tuple(
        lower + (max(-1.0, min(1.0, float(value))) + 1.0) * 0.5 * (upper - lower)
        for value, (lower, upper) in zip(values, RACING_LINE_PARAMETER_BOUNDS, strict=True)
    )
    return RacingLineParameters(*physical)
