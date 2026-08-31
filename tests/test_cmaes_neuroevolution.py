from __future__ import annotations

from math import inf

import pytest

from controllers.cmaes_policy import (
    INPUT_SIZE,
    MIN_THROTTLE,
    PARAMETER_COUNT,
    FixedMLPPolicy,
    initial_parameters,
    observation_vector,
)
from racing import CameraSensors, ImuSensors, LidarSensors, OdometrySensors, RobotSensors
from racing.experiments.neuroevolution import FitnessResult, TrialMetrics, aggregate_fitness, trial_score


def test_policy_shape_and_output_bounds() -> None:
    policy = FixedMLPPolicy(initial_parameters())
    command = policy.command([0.0] * INPUT_SIZE)

    assert len(policy.parameters) == PARAMETER_COUNT == 122
    assert MIN_THROTTLE <= command.throttle <= 1.0
    assert -1.0 <= command.steer <= 1.0
    assert command.steer == 0.0


def test_policy_rejects_wrong_parameter_and_observation_shapes() -> None:
    with pytest.raises(ValueError, match="parameters"):
        FixedMLPPolicy([0.0])
    with pytest.raises(ValueError, match="observations"):
        FixedMLPPolicy(initial_parameters()).command([0.0])


def test_observation_vector_clips_and_replaces_infinite_lidar() -> None:
    sensors = RobotSensors(
        imu=ImuSensors(yaw_rate_degrees_per_s=360.0),
        odometry=OdometrySensors(speed_mps=-24.0),
        camera=CameraSensors(
            center_offset_m=12.0,
            heading_error_degrees=-90.0,
            lookahead_offsets_m=(24.0, -24.0, 0.0),
        ),
        wall_lidar=LidarSensors(distances_m=(inf, inf, inf, 0.0, inf, inf, inf)),
    )

    observation = observation_vector(sensors)

    assert len(observation) == 12
    assert observation[:7] == (-1.0, 1.0, 1.0, -0.5, 1.0, -1.0, 0.0)
    assert all(0.0 <= value <= 1.0 for value in observation[7:])
    assert observation[9] == 0.0


def _metrics(*, progress: float, damage: float = 0.0, eliminated: bool = False) -> TrialMetrics:
    return TrialMetrics(
        seed=1,
        elapsed_seconds=20.0,
        track_length_m=100.0,
        raw_distance_m=progress * 100.0,
        partial_laps=progress,
        lap_count=int(progress),
        damage=damage,
        survived=not eliminated and damage < 1.0,
        eliminated=eliminated,
        wall_contact_seconds=0.0,
        off_track_seconds=0.0,
        low_progress_seconds=0.0,
        max_speed_mps=5.0,
        first_lap_time_seconds=15.0 if progress >= 1.0 else None,
        best_lap_time_seconds=15.0 if progress >= 1.0 else None,
    )


def test_failure_penalty_outweighs_high_progress() -> None:
    safe = _metrics(progress=0.6)
    fast_crash = _metrics(progress=1.2, damage=1.0, eliminated=True)

    assert trial_score(safe) > trial_score(fast_crash)


def test_aggregate_fitness_penalizes_one_catastrophic_seed() -> None:
    consistent = aggregate_fitness([_metrics(progress=0.6), _metrics(progress=0.6), _metrics(progress=0.6)])
    brittle: FitnessResult = aggregate_fitness(
        [_metrics(progress=1.0), _metrics(progress=1.0), _metrics(progress=0.0, damage=1.0, eliminated=True)]
    )

    assert consistent.fitness > brittle.fitness
    assert brittle.minimum_score < consistent.minimum_score
