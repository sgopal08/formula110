"""Translate public simulator sensors into a bounded centerline-control state."""

from __future__ import annotations

from dataclasses import dataclass
from math import exp, isfinite
from typing import Final

from racing import RobotSensors

LIDAR_CAP_M: Final = 20.0
FILTER_TIME_CONSTANT_S: Final = 0.10
CAMERA_HOLD_SECONDS: Final = 0.25
DEFAULT_DT_S: Final = 1.0 / 60.0
LOOKAHEAD_WEIGHTS: Final = (0.2, 0.3, 0.5)
DEFAULT_LOOKAHEAD_DISTANCES_M: Final = (4.0, 9.0, 16.0)


@dataclass(frozen=True, slots=True)
class CenterlineState:
    """Finite, filtered public observations and auditable derived features."""

    tick: int
    dt_s: float
    speed_mps: float
    yaw_rate_degrees_per_s: float
    center_offset_m: float
    heading_error_degrees: float
    center_rate_mps: float
    heading_rate_degrees_per_s: float
    lookahead_offsets_m: tuple[float, float, float]
    lookahead_distances_m: tuple[float, float, float]
    lookahead_slopes: tuple[float, float, float]
    path_bend: float
    bend_change_per_m: float
    wall_clearances_m: tuple[float, ...]
    left_wall_m: float
    right_wall_m: float
    front_wall_m: float
    camera_visible: bool
    camera_missing_seconds: float


class SensorTranslator:
    """Maintain a small, resettable filter over public ``RobotSensors``."""

    def __init__(self) -> None:
        self._last_tick: int | None = None
        self._center = 0.0
        self._heading = 0.0
        self._speed = 0.0
        self._yaw_rate = 0.0
        self._lookahead = (0.0, 0.0, 0.0)
        self._camera_missing_seconds = 0.0

    def reset(self) -> None:
        """Discard all history so the next snapshot initializes the filter."""
        self.__init__()

    def update(self, sensors: RobotSensors) -> CenterlineState:
        """Sanitize, filter, and derive centerline-control features."""
        if self._last_tick is not None and sensors.tick <= self._last_tick:
            self.reset()

        dt_s = _bounded_finite(sensors.dt_s, fallback=DEFAULT_DT_S, lower=1e-6, upper=0.25)
        first_sample = self._last_tick is None
        alpha = 1.0 if first_sample else 1.0 - exp(-dt_s / FILTER_TIME_CONSTANT_S)

        raw_speed = _bounded_finite(sensors.odometry.speed_mps, fallback=0.0, lower=-40.0, upper=40.0)
        raw_yaw = _bounded_finite(
            sensors.imu.yaw_rate_degrees_per_s,
            fallback=0.0,
            lower=-720.0,
            upper=720.0,
        )
        previous_center = self._center
        previous_heading = self._heading

        camera_visible = bool(sensors.camera.visible)
        if camera_visible:
            raw_center = _bounded_finite(sensors.camera.center_offset_m, fallback=0.0, lower=-12.0, upper=12.0)
            raw_heading = _bounded_finite(
                sensors.camera.heading_error_degrees,
                fallback=0.0,
                lower=-180.0,
                upper=180.0,
            )
            raw_lookahead, lookahead_distances = _lookahead_values(sensors, raw_center)
            self._camera_missing_seconds = 0.0
            self._center = _blend(self._center, raw_center, alpha)
            self._heading = _blend_angle_degrees(self._heading, raw_heading, alpha)
            blended_lookahead = tuple(
                _blend(previous, current, alpha)
                for previous, current in zip(self._lookahead, raw_lookahead, strict=True)
            )
            self._lookahead = (blended_lookahead[0], blended_lookahead[1], blended_lookahead[2])
        else:
            self._camera_missing_seconds += dt_s
            lookahead_distances = DEFAULT_LOOKAHEAD_DISTANCES_M
            if self._camera_missing_seconds > CAMERA_HOLD_SECONDS:
                self._center = 0.0
                self._heading = 0.0
                self._lookahead = (0.0, 0.0, 0.0)

        self._speed = _blend(self._speed, raw_speed, alpha)
        self._yaw_rate = _blend(self._yaw_rate, raw_yaw, alpha)
        center_rate = 0.0 if first_sample else (self._center - previous_center) / dt_s
        heading_delta = _wrapped_degrees(self._heading - previous_heading)
        heading_rate = 0.0 if first_sample else heading_delta / dt_s

        slope_values = tuple(
            (offset - self._center) / max(distance, 1.0)
            for offset, distance in zip(self._lookahead, lookahead_distances, strict=True)
        )
        slopes = (slope_values[0], slope_values[1], slope_values[2])
        path_bend = sum(weight * slope for weight, slope in zip(LOOKAHEAD_WEIGHTS, slopes, strict=True))
        separation = max(lookahead_distances[2] - lookahead_distances[0], 1.0)
        bend_change = (slopes[2] - slopes[0]) / separation

        wall_clearances = tuple(_lidar_distance(value) for value in sensors.wall_lidar.distances_m)
        left_wall = _lidar_distance(sensors.wall_lidar.left_m)
        right_wall = _lidar_distance(sensors.wall_lidar.right_m)
        front_wall = _lidar_distance(sensors.wall_lidar.front_m)
        self._last_tick = sensors.tick
        return CenterlineState(
            tick=sensors.tick,
            dt_s=dt_s,
            speed_mps=self._speed,
            yaw_rate_degrees_per_s=self._yaw_rate,
            center_offset_m=self._center,
            heading_error_degrees=self._heading,
            center_rate_mps=center_rate,
            heading_rate_degrees_per_s=heading_rate,
            lookahead_offsets_m=self._lookahead,
            lookahead_distances_m=lookahead_distances,
            lookahead_slopes=slopes,
            path_bend=path_bend,
            bend_change_per_m=bend_change,
            wall_clearances_m=wall_clearances,
            left_wall_m=left_wall,
            right_wall_m=right_wall,
            front_wall_m=front_wall,
            camera_visible=camera_visible,
            camera_missing_seconds=self._camera_missing_seconds,
        )


def _lookahead_values(
    sensors: RobotSensors,
    center_offset_m: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    offsets: list[float] = []
    distances: list[float] = []
    for index in range(3):
        raw_offset = (
            sensors.camera.lookahead_offsets_m[index]
            if index < len(sensors.camera.lookahead_offsets_m)
            else center_offset_m
        )
        raw_distance = (
            sensors.camera.lookahead_distances_m[index]
            if index < len(sensors.camera.lookahead_distances_m)
            else DEFAULT_LOOKAHEAD_DISTANCES_M[index]
        )
        offsets.append(_bounded_finite(raw_offset, fallback=center_offset_m, lower=-24.0, upper=24.0))
        distances.append(
            _bounded_finite(
                raw_distance,
                fallback=DEFAULT_LOOKAHEAD_DISTANCES_M[index],
                lower=1.0,
                upper=100.0,
            )
        )
    return (offsets[0], offsets[1], offsets[2]), (distances[0], distances[1], distances[2])


def _bounded_finite(value: float, *, fallback: float, lower: float, upper: float) -> float:
    finite = float(value) if isfinite(value) else fallback
    return max(lower, min(upper, finite))


def _lidar_distance(value: float) -> float:
    return max(0.0, min(LIDAR_CAP_M, float(value))) if isfinite(value) else LIDAR_CAP_M


def _blend(previous: float, current: float, alpha: float) -> float:
    return previous + alpha * (current - previous)


def _blend_angle_degrees(previous: float, current: float, alpha: float) -> float:
    return _wrapped_degrees(previous + alpha * _wrapped_degrees(current - previous))


def _wrapped_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0
