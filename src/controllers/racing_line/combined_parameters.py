"""Normalized parameter mapping for joint racing-line optimization."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final

from controllers.centerline.parameters import (
    PARAMETER_BOUNDS,
    PARAMETER_NAMES,
    CenterlineParameters,
    from_normalized,
    to_normalized,
)
from controllers.racing_line.parameters import (
    RACING_LINE_PARAMETER_BOUNDS,
    RACING_LINE_PARAMETER_COUNT,
    RACING_LINE_PARAMETER_NAMES,
    RacingLineParameters,
    racing_line_from_normalized,
    racing_line_to_normalized,
)

JOINT_PARAMETER_NAMES: Final = tuple(f"planner.{name}" for name in RACING_LINE_PARAMETER_NAMES) + tuple(
    f"expert.{name}" for name in PARAMETER_NAMES
)
JOINT_PARAMETER_BOUNDS: Final = RACING_LINE_PARAMETER_BOUNDS + PARAMETER_BOUNDS
JOINT_PARAMETER_COUNT: Final = len(JOINT_PARAMETER_NAMES)


@dataclass(frozen=True, slots=True)
class JointParameters:
    planner: RacingLineParameters
    expert: CenterlineParameters


def joint_to_normalized(parameters: JointParameters) -> tuple[float, ...]:
    return (*racing_line_to_normalized(parameters.planner), *to_normalized(parameters.expert))


def joint_from_normalized(values: tuple[float, ...] | list[float]) -> JointParameters:
    if len(values) != JOINT_PARAMETER_COUNT:
        raise ValueError(f"expected {JOINT_PARAMETER_COUNT} normalized joint parameters, got {len(values)}")
    return JointParameters(
        racing_line_from_normalized(list(values[:RACING_LINE_PARAMETER_COUNT])),
        from_normalized(list(values[RACING_LINE_PARAMETER_COUNT:])),
    )
