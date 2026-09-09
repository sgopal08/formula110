"""CMA-ES-tuned reactive speed and racing-line refinement of learned policies."""

from __future__ import annotations

import json
from collections.abc import Sequence
from math import isfinite
from pathlib import Path
from typing import cast

from controllers.cmaes_policy import FixedMLPPolicy
from controllers.cmaes_racing_line import SAFETY_LIDAR_ANGLES
from racing import RobotCommand, RobotSensors

RACING_NAME = "CMA-ES Speed Hybrid"
RACING_COLOR = "#F05A28"
RESIDUAL_COUNT = 8


def _load_policy(name: str) -> FixedMLPPolicy:
    payload = cast(dict[str, object], json.loads(Path(__file__).with_name(name).read_text(encoding="utf-8")))
    return FixedMLPPolicy(cast(list[float], payload["parameters"]))


class SpeedHybridController:
    """Learn acceleration, turn anticipation and centering without a corner speed cap.

    Zero residuals reproduce the previous racing-line hybrid. Parameters are
    dimensionless CMA-ES coordinates; inference uses only public local sensors.
    """

    def __init__(self, parameters: Sequence[float]) -> None:
        self.parameters = tuple(float(value) for value in parameters)
        if len(self.parameters) != RESIDUAL_COUNT or not all(isfinite(v) for v in self.parameters):
            raise ValueError("expected eight finite reactive parameters")
        self._fast = _load_policy("cmaes_racing_line_weights.json")
        self._safe = _load_policy("cmaes_weights_pre_racing_line.json")

    def __call__(self, sensors: RobotSensors) -> RobotCommand:
        launch, cruise, corner, turn, center, yaw, shield, launch_limit = self.parameters
        clearances = (sensors.wall_lidar.distance_at_angle_degrees(a) for a in SAFETY_LIDAR_ANGLES)
        nearest = min((d for d in clearances if isfinite(d)), default=float("inf"))
        safety_distance = max(0.8, min(2.8, 1.8 + 0.3 * shield))
        command = (self._safe if nearest < safety_distance else self._fast)(sensors)
        camera = sensors.camera
        speed = sensors.odometry.speed_mps
        lookahead = camera.lookahead_offsets_m
        future_turn = max(-1.0, min(1.0, lookahead[-1] / 12.0)) if lookahead else 0.0
        launch_target = max(3.0, min(16.0, 10.0 + 2.0 * launch_limit))
        launch_fraction = max(0.0, min(1.0, 1.0 - max(0.0, speed) / launch_target))
        throttle = command.throttle + 0.5 * launch * launch_fraction + 0.08 * cruise
        throttle += 0.08 * corner * abs(future_turn)
        steer = command.steer + 0.08 * turn * future_turn
        steer += 0.025 * center * camera.center_offset_m
        steer -= 0.0005 * yaw * sensors.imu.yaw_rate_degrees_per_s
        return RobotCommand(throttle=max(0.0, min(1.0, throttle)), steer=max(-1.0, min(1.0, steer)))


def create_controller() -> SpeedHybridController:
    payload = cast(
        dict[str, object],
        json.loads(Path(__file__).with_name("cmaes_speed_hybrid_weights.json").read_text(encoding="utf-8")),
    )
    return SpeedHybridController(cast(list[float], payload["parameters"]))
