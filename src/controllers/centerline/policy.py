"""Auditable centerline expert and fixed emergency safety filter."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final

from controllers.centerline.parameters import CenterlineParameters
from controllers.centerline.state import CAMERA_HOLD_SECONDS, CenterlineState
from racing import RobotCommand

WALL_DANGER_THRESHOLD_M: Final = 1.35
WALL_EMERGENCY_THRESHOLD_M: Final = 0.72
EMERGENCY_CORRECTION_GAIN: Final = 0.75


@dataclass(frozen=True, slots=True)
class ExpertDecision:
    """Nominal command plus decomposed terms for tests and experiment traces."""

    target_center_offset_m: float
    target_speed_mps: float
    center_steer: float
    heading_steer: float
    bend_steer: float
    bend_change_steer: float
    yaw_damping_steer: float
    command: RobotCommand


class CenterlineExpert:
    """Convert translated local track state into a nominal centerline command."""

    def __init__(self, parameters: CenterlineParameters) -> None:
        self.parameters = parameters

    def decide(self, state: CenterlineState) -> ExpertDecision:
        p = self.parameters
        if not state.camera_visible and state.camera_missing_seconds > CAMERA_HOLD_SECONDS:
            return ExpertDecision(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, RobotCommand(throttle=-0.2, steer=0.0))

        center_steer = p.center_gain * state.center_offset_m
        heading_steer = p.heading_gain * state.heading_error_degrees
        bend_steer = p.bend_gain * state.path_bend
        bend_change_steer = p.bend_change_gain * state.bend_change_per_m
        yaw_damping_steer = -p.yaw_damping * state.yaw_rate_degrees_per_s
        steer = center_steer + heading_steer + bend_steer + bend_change_steer + yaw_damping_steer

        corner_demand = _clamp(abs(state.path_bend) + 4.0 * abs(state.bend_change_per_m), 0.0, 1.0)
        minimum_corner_speed = max(3.0, p.straight_target_speed_mps - p.corner_speed_reduction_mps)
        target_speed = (
            p.straight_target_speed_mps
            - p.corner_speed_reduction_mps * corner_demand
            - p.center_speed_penalty * abs(state.center_offset_m)
            - p.heading_speed_penalty * abs(state.heading_error_degrees)
        )
        target_speed = _clamp(target_speed, minimum_corner_speed, p.straight_target_speed_mps)
        if min(state.left_wall_m, state.right_wall_m) < WALL_DANGER_THRESHOLD_M:
            target_speed = min(target_speed, 3.0)
        if not state.camera_visible:
            target_speed = min(target_speed, 3.0)

        speed_error = target_speed - state.speed_mps
        gain = p.acceleration_gain if speed_error >= 0.0 else p.braking_gain
        throttle = _clamp(gain * speed_error, -0.8, 0.8)
        command = RobotCommand(throttle=throttle, steer=_clamp(steer, -1.0, 1.0))
        return ExpertDecision(
            target_center_offset_m=0.0,
            target_speed_mps=target_speed,
            center_steer=center_steer,
            heading_steer=heading_steer,
            bend_steer=bend_steer,
            bend_change_steer=bend_change_steer,
            yaw_damping_steer=yaw_damping_steer,
            command=command,
        )


class SafetyFilter:
    """Apply fixed wall avoidance after nominal command smoothing."""

    def __init__(self, *, emergency_brake_intent: float = -0.2) -> None:
        self.emergency_brake_intent = emergency_brake_intent

    def apply(self, state: CenterlineState, command: RobotCommand) -> RobotCommand:
        steer = command.steer
        nearest_side = min(state.left_wall_m, state.right_wall_m)
        if nearest_side < WALL_DANGER_THRESHOLD_M:
            open_side = 1.0 if state.right_wall_m > state.left_wall_m else -1.0
            steer += open_side * EMERGENCY_CORRECTION_GAIN * (1.0 - nearest_side / WALL_DANGER_THRESHOLD_M)

        throttle = command.throttle
        if state.front_wall_m < WALL_DANGER_THRESHOLD_M:
            throttle = min(
                throttle,
                self.emergency_brake_intent if state.front_wall_m < WALL_EMERGENCY_THRESHOLD_M else 0.05,
            )
        return RobotCommand(throttle=_clamp(throttle, -1.0, 1.0), steer=_clamp(steer, -1.0, 1.0))


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
