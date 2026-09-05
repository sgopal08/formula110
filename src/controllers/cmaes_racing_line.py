"""Fast CMA-ES racing-line policy protected by a learned safety fallback."""

from __future__ import annotations

import json
from math import isfinite
from pathlib import Path
from typing import Final, cast

from controllers.cmaes_policy import FixedMLPPolicy
from racing import RobotCommand, RobotSensors

RACING_NAME: str = "CMA-ES Racing Line"
RACING_COLOR: str = "#FF8A34"

SAFETY_DISTANCE_M: Final = 1.8
SAFETY_LIDAR_ANGLES: Final = (-90.0, -45.0, -20.0, 0.0, 20.0, 45.0, 90.0)


def _load_policy(artifact_name: str) -> FixedMLPPolicy:
    path = Path(__file__).with_name(artifact_name)
    payload = cast(dict[str, object], json.loads(path.read_text(encoding="utf-8")))
    return FixedMLPPolicy(cast(list[float], payload["parameters"]))


class RacingLineController:
    """Use the fast evolved policy except near walls, where the safe champion takes over."""

    def __init__(self) -> None:
        self._racing_line = _load_policy("cmaes_racing_line_weights.json")
        self._safe_fallback = _load_policy("cmaes_weights_pre_racing_line.json")

    def __call__(self, sensors: RobotSensors) -> RobotCommand:
        clearances = tuple(sensors.wall_lidar.distance_at_angle_degrees(angle) for angle in SAFETY_LIDAR_ANGLES)
        nearest_wall = min((distance for distance in clearances if isfinite(distance)), default=float("inf"))
        policy = self._safe_fallback if nearest_wall < SAFETY_DISTANCE_M else self._racing_line
        return policy(sensors)


def create_controller() -> RacingLineController:
    return RacingLineController()
