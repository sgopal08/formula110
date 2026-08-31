"""Baseline Reactive Controller v1"""

from __future__ import annotations

from math import isfinite

from racing import RobotCommand, RobotSensors

RACING_NAME: str = "Minimum Viable Reactive"
RACING_COLOR: str = "#36C275"

# Steering parameters. Positive sensor offsets and steering commands point right.
STEERING_GAIN: float = 0.22
HEADING_GAIN: float = 0.012
EMERGENCY_CORRECTION_GAIN: float = 0.75

# Throttle schedule by absolute steering demand.
STRAIGHT_THROTTLE: float = 0.62
MODERATE_TURN_THROTTLE: float = 0.38
SHARP_TURN_THROTTLE: float = 0.18
MODERATE_TURN_THRESHOLD: float = 0.28
SHARP_TURN_THRESHOLD: float = 0.58
TARGET_SPEED_MPS: float = 5.5

# Below this wall distance, prioritize recovery over speed.
WALL_DANGER_THRESHOLD_M: float = 1.35
WALL_EMERGENCY_THRESHOLD_M: float = 0.72


def _finite_or(value: float, fallback: float) -> float:
    return value if isfinite(value) else fallback


def control(sensors: RobotSensors) -> RobotCommand:
    """Follow the processed centerline while slowing for turns and walls."""
    camera = sensors.camera
    wall_lidar = sensors.wall_lidar

    center_offset = _finite_or(camera.center_offset_m, 0.0)
    heading_error = _finite_or(camera.heading_error_degrees, 0.0)
    steer = STEERING_GAIN * center_offset + HEADING_GAIN * heading_error

    # Lookahead curvature makes the controller begin turning before a corner.
    if camera.lookahead_offsets_m:
        lookahead_offset = _finite_or(camera.lookahead_offsets_m[-1], 0.0)
        steer += 0.035 * lookahead_offset

    left_wall = _finite_or(wall_lidar.left_m, 1000.0)
    right_wall = _finite_or(wall_lidar.right_m, 1000.0)
    front_wall = _finite_or(wall_lidar.front_m, 1000.0)

    nearest_side = min(left_wall, right_wall)
    if nearest_side < WALL_DANGER_THRESHOLD_M:
        # Steer toward the side with more open space.
        open_side = 1.0 if right_wall > left_wall else -1.0
        steer += open_side * EMERGENCY_CORRECTION_GAIN * (
            1.0 - nearest_side / WALL_DANGER_THRESHOLD_M
        )

    steer = max(-1.0, min(1.0, steer))
    turn_amount = abs(steer)
    if turn_amount >= SHARP_TURN_THRESHOLD:
        throttle = SHARP_TURN_THROTTLE
    elif turn_amount >= MODERATE_TURN_THRESHOLD:
        throttle = MODERATE_TURN_THROTTLE
    else:
        throttle = STRAIGHT_THROTTLE

    speed = _finite_or(sensors.odometry.speed_mps, 0.0)
    if speed > TARGET_SPEED_MPS:
        throttle = min(throttle, 0.12)
    if front_wall < WALL_DANGER_THRESHOLD_M:
        throttle = min(throttle, -0.2 if front_wall < WALL_EMERGENCY_THRESHOLD_M else 0.05)

    return RobotCommand(throttle=throttle, steer=steer)
