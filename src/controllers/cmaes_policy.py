"""Small, dependency-free neural policy used by CMA-ES training and inference."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from math import exp, isfinite, tanh
from typing import Final

from racing import RobotCommand, RobotSensors

INPUT_SIZE: Final = 12
HIDDEN_SIZE: Final = 8
OUTPUT_SIZE: Final = 2
PARAMETER_COUNT: Final = INPUT_SIZE * HIDDEN_SIZE + HIDDEN_SIZE + HIDDEN_SIZE * OUTPUT_SIZE + OUTPUT_SIZE

SPEED_CAP_MPS: Final = 12.0
YAW_RATE_CAP_DEGREES_PER_S: Final = 180.0
CENTER_OFFSET_CAP_M: Final = 6.0
LOOKAHEAD_OFFSET_CAP_M: Final = 12.0
LIDAR_CAP_M: Final = 20.0
MIN_THROTTLE: Final = 0.05
WALL_LIDAR_ANGLES: Final = (-90.0, -20.0, 0.0, 20.0, 90.0)


def initial_parameters() -> tuple[float, ...]:
    """Return a safe CMA-ES starting mean: straight steering and gentle throttle."""
    parameters = [0.0] * PARAMETER_COUNT
    parameters[-1] = -0.85  # Approximately 0.33 forward throttle after the sigmoid mapping.
    return tuple(parameters)


def observation_vector(sensors: RobotSensors) -> tuple[float, ...]:
    """Build the documented 12-value normalized policy observation."""
    camera = sensors.camera
    lookahead = tuple(camera.lookahead_offsets_m)
    padded_lookahead = (*lookahead[:3], 0.0, 0.0, 0.0)[:3]
    values = (
        _signed_scale(sensors.odometry.speed_mps, SPEED_CAP_MPS),
        _signed_scale(sensors.imu.yaw_rate_degrees_per_s, YAW_RATE_CAP_DEGREES_PER_S),
        _signed_scale(camera.center_offset_m, CENTER_OFFSET_CAP_M),
        _signed_scale(camera.heading_error_degrees, 180.0),
        *(_signed_scale(value, LOOKAHEAD_OFFSET_CAP_M) for value in padded_lookahead),
        *(_lidar_scale(sensors.wall_lidar.distance_at_angle_degrees(angle)) for angle in WALL_LIDAR_ANGLES),
    )
    if len(values) != INPUT_SIZE:
        raise AssertionError(f"expected {INPUT_SIZE} observations, got {len(values)}")
    return values


@dataclass(frozen=True, slots=True)
class FixedMLPPolicy:
    """A fixed 12-8-2 MLP whose complete parameter vector is evolved by CMA-ES."""

    parameters: tuple[float, ...]

    def __init__(self, parameters: Sequence[float]) -> None:
        converted = tuple(float(value) for value in parameters)
        if len(converted) != PARAMETER_COUNT:
            raise ValueError(f"expected {PARAMETER_COUNT} parameters, got {len(converted)}")
        if not all(isfinite(value) for value in converted):
            raise ValueError("policy parameters must all be finite")
        object.__setattr__(self, "parameters", converted)

    def __call__(self, sensors: RobotSensors) -> RobotCommand:
        return self.command(observation_vector(sensors))

    def command(self, observation: Sequence[float]) -> RobotCommand:
        """Run dependency-free inference for one already normalized observation."""
        if len(observation) != INPUT_SIZE:
            raise ValueError(f"expected {INPUT_SIZE} observations, got {len(observation)}")
        offset = 0
        hidden: list[float] = []
        for hidden_index in range(HIDDEN_SIZE):
            weighted_sum = 0.0
            for input_index in range(INPUT_SIZE):
                weight_index = hidden_index * INPUT_SIZE + input_index
                weighted_sum += self.parameters[weight_index] * float(observation[input_index])
            hidden.append(tanh(weighted_sum + self.parameters[INPUT_SIZE * HIDDEN_SIZE + hidden_index]))
        offset += INPUT_SIZE * HIDDEN_SIZE + HIDDEN_SIZE

        outputs: list[float] = []
        for output_index in range(OUTPUT_SIZE):
            weighted_sum = 0.0
            for hidden_index, hidden_value in enumerate(hidden):
                weight_index = offset + output_index * HIDDEN_SIZE + hidden_index
                weighted_sum += self.parameters[weight_index] * hidden_value
            outputs.append(weighted_sum)
        offset += HIDDEN_SIZE * OUTPUT_SIZE
        outputs = [outputs[index] + self.parameters[offset + index] for index in range(OUTPUT_SIZE)]

        steer = tanh(outputs[0])
        throttle = MIN_THROTTLE + (1.0 - MIN_THROTTLE) * _sigmoid(outputs[1])
        return RobotCommand(throttle=throttle, steer=steer)


def _signed_scale(value: float, cap: float) -> float:
    if not isfinite(value):
        return 0.0
    return max(-1.0, min(1.0, value / cap))


def _lidar_scale(value: float) -> float:
    finite_value = value if isfinite(value) else LIDAR_CAP_M
    return max(0.0, min(1.0, finite_value / LIDAR_CAP_M))


def _sigmoid(value: float) -> float:
    if value >= 0.0:
        return 1.0 / (1.0 + exp(-min(value, 700.0)))
    exponential = exp(max(value, -700.0))
    return exponential / (1.0 + exponential)
