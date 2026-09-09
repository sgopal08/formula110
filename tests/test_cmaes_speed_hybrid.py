from __future__ import annotations

from math import inf

import pytest

from controllers.cmaes_racing_line import RacingLineController
from controllers.cmaes_speed_hybrid import SpeedHybridController
from racing import CameraSensors, ImuSensors, LidarSensors, OdometrySensors, RobotSensors


@pytest.mark.parametrize("clearance", [0.5, 1.79, 1.8, 3.0, inf])
def test_zero_residual_reproduces_previous_hybrid(clearance: float) -> None:
    sensors = RobotSensors(
        camera=CameraSensors(center_offset_m=0.7, lookahead_offsets_m=(0.1, 1.0, 3.0)),
        imu=ImuSensors(yaw_rate_degrees_per_s=35),
        odometry=OdometrySensors(speed_mps=15),
        wall_lidar=LidarSensors(distances_m=(clearance,) * 7),
    )
    assert SpeedHybridController([0.0] * 8)(sensors) == RacingLineController()(sensors)


def test_launch_boost_increases_initial_throttle_and_fades_at_target_speed() -> None:
    baseline = SpeedHybridController([0.0] * 8)
    boosted = SpeedHybridController([1.0, 0, 0, 0, 0, 0, 0, 0])
    stopped = RobotSensors()
    fast = RobotSensors(odometry=OdometrySensors(speed_mps=12))
    assert boosted(stopped).throttle > baseline(stopped).throttle
    assert boosted(stopped).steer == baseline(stopped).steer
    assert boosted(fast) == baseline(fast)


def test_corner_boost_preserves_steering_and_does_not_cap_throttle() -> None:
    sensors = RobotSensors(camera=CameraSensors(lookahead_offsets_m=(1, 3, 8)))
    baseline = SpeedHybridController([0.0] * 8)(sensors)
    boosted = SpeedHybridController([0, 0, 1, 0, 0, 0, 0, 0])(sensors)
    assert boosted.throttle > baseline.throttle
    assert boosted.steer == baseline.steer


@pytest.mark.parametrize("parameters", [[0.0], [float("nan")] * 8, [inf] * 8])
def test_invalid_residuals_rejected(parameters: list[float]) -> None:
    with pytest.raises(ValueError, match="eight finite"):
        SpeedHybridController(parameters)
