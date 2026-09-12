"""Immutable authored and optimizable centerline parameters."""

from __future__ import annotations

from dataclasses import astuple, dataclass
from math import isfinite
from typing import Final


@dataclass(frozen=True, slots=True)
class CenterlineParameters:
    """The complete tunable parameter vector for the fixed centerline expert."""

    center_gain: float
    heading_gain: float
    bend_gain: float
    bend_change_gain: float
    yaw_damping: float
    straight_target_speed_mps: float
    corner_speed_reduction_mps: float
    center_speed_penalty: float
    heading_speed_penalty: float
    acceleration_gain: float
    braking_gain: float
    steering_smoothing_s: float
    throttle_smoothing_s: float

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in astuple(self)):
            raise ValueError("centerline parameters must be finite")
        if self.straight_target_speed_mps <= 0.0:
            raise ValueError("straight target speed must be positive")
        if self.corner_speed_reduction_mps < 0.0:
            raise ValueError("corner speed reduction cannot be negative")
        if self.acceleration_gain < 0.0 or self.braking_gain < 0.0:
            raise ValueError("speed-control gains cannot be negative")
        if self.steering_smoothing_s <= 0.0 or self.throttle_smoothing_s <= 0.0:
            raise ValueError("smoothing time constants must be positive")


AUTHORED_PARAMETERS: Final = CenterlineParameters(
    center_gain=0.22,
    heading_gain=0.012,
    bend_gain=0.55,
    bend_change_gain=1.0,
    yaw_damping=0.0015,
    straight_target_speed_mps=16.0,
    corner_speed_reduction_mps=10.5,
    center_speed_penalty=2.0,
    heading_speed_penalty=0.04,
    acceleration_gain=0.14,
    braking_gain=0.30,
    steering_smoothing_s=0.05,
    throttle_smoothing_s=0.08,
)

# The written plan called this a twelve-parameter vector, but enumerated thirteen
# values. The executable mapping preserves all thirteen explicitly named values.
PARAMETER_NAMES: Final = tuple(CenterlineParameters.__dataclass_fields__)
PARAMETER_BOUNDS: Final = (
    (0.08, 0.45),
    (0.004, 0.025),
    (0.10, 1.25),
    (0.0, 3.0),
    (0.0, 0.006),
    (10.0, 20.0),
    (3.0, 14.0),
    (0.0, 4.0),
    (0.0, 0.10),
    (0.05, 0.35),
    (0.10, 0.70),
    (0.02, 0.15),
    (0.03, 0.20),
)
PARAMETER_COUNT: Final = len(PARAMETER_NAMES)


def to_normalized(parameters: CenterlineParameters) -> tuple[float, ...]:
    """Map physical parameters into the CMA-ES search cube ``[-1, 1]``."""
    return tuple(
        2.0 * (value - lower) / (upper - lower) - 1.0
        for value, (lower, upper) in zip(astuple(parameters), PARAMETER_BOUNDS, strict=True)
    )


def from_normalized(values: tuple[float, ...] | list[float]) -> CenterlineParameters:
    """Map a bounded CMA-ES vector into physical controller parameters."""
    if len(values) != PARAMETER_COUNT:
        raise ValueError(f"expected {PARAMETER_COUNT} normalized parameters, got {len(values)}")
    physical = tuple(
        lower + (max(-1.0, min(1.0, float(value))) + 1.0) * 0.5 * (upper - lower)
        for value, (lower, upper) in zip(values, PARAMETER_BOUNDS, strict=True)
    )
    return CenterlineParameters(*physical)
