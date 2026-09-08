from __future__ import annotations

import subprocess
import sys
from dataclasses import astuple, replace
from math import inf

import pytest

from controllers.centerline import CenterlineController
from controllers.centerline.braking_tuned_parameters import BRAKING_TUNED_PARAMETERS
from controllers.centerline.policy import CenterlineExpert
from controllers.centerline.state import SensorTranslator
from controllers.centerline_v4 import create_controller as create_centerline_v4
from controllers.racing_line import AUTHORED_RACING_LINE_PARAMETERS, LineTarget, RacingLinePlanner
from controllers.racing_line.combined_parameters import (
    JOINT_PARAMETER_COUNT,
    JointParameters,
    joint_from_normalized,
    joint_to_normalized,
)
from controllers.racing_line.parameters import (
    RACING_LINE_PARAMETER_COUNT,
    racing_line_from_normalized,
    racing_line_to_normalized,
)
from controllers.racing_line_v1 import create_controller as create_racing_line_v1
from controllers.racing_line_v2 import create_controller as create_racing_line_v2
from controllers.racing_line_v3 import create_controller as create_racing_line_v3
from racing import CameraSensors, ImuSensors, LidarSensors, OdometrySensors, RobotSensors
from racing.experiments.centerline_braking import BrakingDiagnostics, DiagnosedTrial
from racing.experiments.neuroevolution import TrialMetrics
from racing.experiments.racing_line import racing_line_fitness, racing_line_promotion


def test_racing_line_package_imports_without_centerline_import_order_dependency() -> None:
    result = subprocess.run(
        [sys.executable, "-c", "import controllers.racing_line; import controllers.centerline"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr


def sensors(*, tick: int = 0, visible: bool = True) -> RobotSensors:
    return RobotSensors(
        dt_s=1.0 / 60.0,
        tick=tick,
        imu=ImuSensors(),
        odometry=OdometrySensors(speed_mps=12.0),
        camera=CameraSensors(visible=visible, lookahead_offsets_m=(0.0, 0.0, 0.0)),
        wall_lidar=LidarSensors(distances_m=(inf,) * 7),
    )


def state_for(*, bend: float, near: float, far: float, tick: int = 0):
    base = SensorTranslator().update(sensors(tick=tick))
    return replace(base, path_bend=bend, lookahead_slopes=(near, bend, far))


@pytest.mark.parametrize("direction", (-1, 1))
def test_planner_selects_outside_entry_inside_apex_and_outside_exit(direction: int) -> None:
    entry = RacingLinePlanner(AUTHORED_RACING_LINE_PARAMETERS).plan(
        state_for(bend=0.5 * direction, near=0.1 * direction, far=0.5 * direction)
    )
    apex = RacingLinePlanner(AUTHORED_RACING_LINE_PARAMETERS).plan(
        state_for(bend=0.5 * direction, near=0.5 * direction, far=0.5 * direction)
    )
    exit_target = RacingLinePlanner(AUTHORED_RACING_LINE_PARAMETERS).plan(
        state_for(bend=0.5 * direction, near=0.5 * direction, far=0.1 * direction)
    )

    assert entry.turn_direction == direction
    assert entry.raw_offset_m * direction < 0.0
    assert apex.raw_offset_m * direction > 0.0
    assert exit_target.raw_offset_m * direction < 0.0
    assert entry.entry_weight > 0.0 and apex.apex_weight == 1.0 and exit_target.exit_weight > 0.0


def test_planner_centers_weak_and_conflicting_bends() -> None:
    weak = RacingLinePlanner(AUTHORED_RACING_LINE_PARAMETERS).plan(state_for(bend=0.01, near=0.01, far=0.01))
    conflict = RacingLinePlanner(AUTHORED_RACING_LINE_PARAMETERS).plan(state_for(bend=0.2, near=-0.2, far=0.4))

    assert weak.desired_lateral_offset_m == 0.0
    assert conflict.turn_direction == 0
    assert conflict.desired_lateral_offset_m == 0.0


def test_planner_filters_rate_limits_and_resets_on_tick_regression() -> None:
    planner = RacingLinePlanner(AUTHORED_RACING_LINE_PARAMETERS)
    first = planner.plan(state_for(bend=1.0, near=1.0, far=1.0, tick=0))
    second = planner.plan(state_for(bend=1.0, near=1.0, far=1.0, tick=1))
    reset = planner.plan(state_for(bend=-1.0, near=-1.0, far=-1.0, tick=0))
    maximum_delta = AUTHORED_RACING_LINE_PARAMETERS.max_offset_rate_mps / 60.0

    assert 0.0 < first.desired_lateral_offset_m <= maximum_delta
    assert first.desired_lateral_offset_m < second.desired_lateral_offset_m
    assert -maximum_delta <= reset.desired_lateral_offset_m < 0.0


def test_planner_centers_on_camera_loss_or_wall_danger_and_limits_clearance() -> None:
    planner = RacingLinePlanner(AUTHORED_RACING_LINE_PARAMETERS)
    corner = state_for(bend=1.0, near=1.0, far=1.0)
    limited = planner.plan(replace(corner, center_offset_m=0.0, right_wall_m=1.62))
    lost = planner.plan(replace(corner, tick=1, camera_visible=False))
    danger = planner.plan(replace(corner, tick=2, left_wall_m=1.0))

    assert limited.clearance_limited
    assert limited.desired_lateral_offset_m <= 0.02 + 1e-9
    assert lost.desired_lateral_offset_m == 0.0
    assert danger.desired_lateral_offset_m == 0.0


def test_expert_uses_racing_line_error_without_changing_default_centerline() -> None:
    state = state_for(bend=0.0, near=0.0, far=0.0)
    target = LineTarget(1.0, 1, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, False)
    expert = CenterlineExpert(BRAKING_TUNED_PARAMETERS)

    centerline = expert.decide(state)
    racing_line = expert.decide(state, target)

    assert centerline.target_center_offset_m == 0.0
    assert centerline.center_steer == 0.0
    assert racing_line.target_center_offset_m == 1.0
    assert racing_line.center_steer == pytest.approx(BRAKING_TUNED_PARAMETERS.center_gain)


def test_racing_line_parameter_mapping_round_trips() -> None:
    normalized = racing_line_to_normalized(AUTHORED_RACING_LINE_PARAMETERS)
    reconstructed = racing_line_from_normalized(list(normalized))

    assert len(normalized) == RACING_LINE_PARAMETER_COUNT == 10
    assert astuple(reconstructed) == pytest.approx(astuple(AUTHORED_RACING_LINE_PARAMETERS))
    assert all(-1.0 <= value <= 1.0 for value in racing_line_to_normalized(racing_line_from_normalized([2.0] * 10)))


def test_racing_line_factories_are_independent_and_centerline_has_no_planner() -> None:
    centerline = create_centerline_v4()
    first = create_racing_line_v1()
    second = create_racing_line_v1()

    assert isinstance(first, CenterlineController)
    assert centerline.line_planner is None and centerline.last_line_target is None
    assert first.line_planner is not None and first.line_planner is not second.line_planner
    assert create_racing_line_v2().line_planner is not None
    assert create_racing_line_v3().line_planner is not None


def test_joint_parameter_mapping_round_trips_all_23_values() -> None:
    original = JointParameters(AUTHORED_RACING_LINE_PARAMETERS, BRAKING_TUNED_PARAMETERS)
    normalized = joint_to_normalized(original)
    reconstructed = joint_from_normalized(list(normalized))

    assert len(normalized) == JOINT_PARAMETER_COUNT == 23
    assert astuple(reconstructed.planner) == pytest.approx(astuple(original.planner))
    assert astuple(reconstructed.expert) == pytest.approx(astuple(original.expert))


def diagnosed_trial(
    *,
    seed: int,
    distance: float,
    laps: int = 3,
    damage: float = 0.0,
    wall: float = 0.0,
    off_track: float = 0.0,
    reverse: float = 0.0,
    stops: int = 0,
) -> DiagnosedTrial:
    metrics = TrialMetrics(
        seed=seed,
        elapsed_seconds=30.0,
        track_length_m=183.0,
        raw_distance_m=distance,
        partial_laps=distance / 183.0,
        lap_count=laps,
        damage=damage,
        survived=damage < 1.0,
        eliminated=False,
        wall_contact_seconds=wall,
        off_track_seconds=off_track,
        low_progress_seconds=0.0,
        max_speed_mps=20.0,
        first_lap_time_seconds=10.0,
        best_lap_time_seconds=9.5,
    )
    diagnostics = BrakingDiagnostics(0, 0, 0, reverse, stops, stops, 0.2 * stops)
    return DiagnosedTrial(metrics, diagnostics)


def test_racing_line_fitness_preserves_every_dominant_tier() -> None:
    clean = racing_line_fitness([diagnosed_trial(seed=1, distance=550.0)])
    off_track = racing_line_fitness([diagnosed_trial(seed=1, distance=1000.0, off_track=0.1)])
    stopped = racing_line_fitness([diagnosed_trial(seed=1, distance=2000.0, stops=1)])
    reverse = racing_line_fitness([diagnosed_trial(seed=1, distance=3000.0, reverse=0.1)])
    incomplete = racing_line_fitness([diagnosed_trial(seed=1, distance=4000.0, laps=2)])
    contact = racing_line_fitness([diagnosed_trial(seed=1, distance=5000.0, wall=0.01)])
    damaged = racing_line_fitness([diagnosed_trial(seed=1, distance=6000.0, damage=0.01)])

    assert clean.fitness > off_track.fitness > stopped.fitness > reverse.fitness > incomplete.fitness
    assert incomplete.fitness > contact.fitness > damaged.fitness


@pytest.mark.parametrize(
    ("candidate", "reason"),
    (
        (diagnosed_trial(seed=1, distance=520.0, laps=2), "fewer than three laps"),
        (diagnosed_trial(seed=1, distance=520.0, damage=0.01), "recorded damage"),
        (diagnosed_trial(seed=1, distance=520.0, wall=0.01), "wall contact"),
        (diagnosed_trial(seed=1, distance=520.0, off_track=0.01), "off track"),
        (diagnosed_trial(seed=1, distance=520.0, reverse=0.01), "reversed"),
        (diagnosed_trial(seed=1, distance=520.0, stops=1), "non-emergency stop"),
    ),
)
def test_racing_line_promotion_rejects_each_safety_failure(candidate: DiagnosedTrial, reason: str) -> None:
    baseline = [diagnosed_trial(seed=1, distance=500.0)]
    result = racing_line_promotion([candidate], baseline, baseline)

    assert not result.passed
    assert any(reason in item for item in result.reasons)


def test_racing_line_promotion_requires_both_performance_comparisons() -> None:
    centerline = [diagnosed_trial(seed=1, distance=500.0), diagnosed_trial(seed=2, distance=510.0)]
    incumbent = [diagnosed_trial(seed=1, distance=510.0), diagnosed_trial(seed=2, distance=520.0)]
    passing = [diagnosed_trial(seed=1, distance=511.0), diagnosed_trial(seed=2, distance=521.0)]
    slower = [diagnosed_trial(seed=1, distance=509.0), diagnosed_trial(seed=2, distance=519.0)]

    assert racing_line_promotion(passing, centerline, incumbent).passed
    assert not racing_line_promotion(slower, centerline, incumbent).passed
