from __future__ import annotations

from dataclasses import astuple, replace
from math import inf

import pytest

from controllers.centerline import AUTHORED_PARAMETERS, CenterlineController, ForwardOnlyDriveAdapter
from controllers.centerline.drive import BRAKE_CUTOFF_MPS
from controllers.centerline.parameters import PARAMETER_COUNT, from_normalized, to_normalized
from controllers.centerline.policy import CenterlineExpert, SafetyFilter
from controllers.centerline.state import CAMERA_HOLD_SECONDS, LIDAR_CAP_M, SensorTranslator
from controllers.centerline_v1 import create_controller as create_centerline_v1
from controllers.centerline_v2 import create_controller as create_centerline_v2
from controllers.centerline_v3 import create_controller as create_centerline_v3
from controllers.centerline_v4 import create_controller as create_centerline_v4
from racing import CameraSensors, ImuSensors, LidarSensors, OdometrySensors, RobotCommand, RobotSensors
from racing.experiments.centerline import centerline_fitness, promotion_result
from racing.experiments.centerline_braking import (
    BrakingDiagnostics,
    DiagnosedTrial,
    DiagnosticCenterlineController,
    braking_fitness,
    braking_promotion_result,
)
from racing.experiments.neuroevolution import TrialMetrics
from racing.physics import FORMULA_VEHICLE_PHYSICS_CONFIG, resolve_vehicle_actuator_command


def sensors(
    *,
    tick: int = 0,
    center: float = 0.0,
    heading: float = 0.0,
    lookahead: tuple[float, ...] = (0.0, 0.0, 0.0),
    speed: float = 0.0,
    yaw_rate: float = 0.0,
    walls: tuple[float, ...] = (inf, inf, inf, inf, inf, inf, inf),
    visible: bool = True,
    dt_s: float = 1.0 / 60.0,
) -> RobotSensors:
    return RobotSensors(
        dt_s=dt_s,
        tick=tick,
        imu=ImuSensors(yaw_rate_degrees_per_s=yaw_rate),
        odometry=OdometrySensors(speed_mps=speed),
        camera=CameraSensors(
            visible=visible,
            center_offset_m=center,
            heading_error_degrees=heading,
            lookahead_offsets_m=lookahead,
        ),
        wall_lidar=LidarSensors(distances_m=walls),
    )


def test_sensor_translation_sanitizes_lidar_and_derives_lookahead_features() -> None:
    state = SensorTranslator().update(
        sensors(center=1.0, lookahead=(2.0, 4.0, 7.0), walls=(inf, 4.0, inf, 3.0, inf, 2.0, inf))
    )

    assert state.wall_clearances_m[0] == LIDAR_CAP_M
    assert state.front_wall_m == 3.0
    assert state.lookahead_slopes == pytest.approx((0.25, 1.0 / 3.0, 0.375))
    assert state.path_bend == pytest.approx(0.2 * 0.25 + 0.3 / 3.0 + 0.5 * 0.375)
    assert state.center_rate_mps == 0.0
    assert state.heading_rate_degrees_per_s == 0.0


def test_sensor_translation_filters_changes_and_resets_on_nonincreasing_tick() -> None:
    translator = SensorTranslator()
    translator.update(sensors(tick=0, center=0.0, heading=0.0))
    changed = translator.update(sensors(tick=1, center=2.0, heading=30.0))
    reset = translator.update(sensors(tick=0, center=-1.0, heading=-20.0))

    assert 0.0 < changed.center_offset_m < 2.0
    assert changed.center_rate_mps > 0.0
    assert changed.heading_rate_degrees_per_s > 0.0
    assert reset.center_offset_m == -1.0
    assert reset.center_rate_mps == 0.0
    assert reset.heading_rate_degrees_per_s == 0.0


def test_sensor_translation_holds_then_discards_missing_camera_geometry() -> None:
    translator = SensorTranslator()
    translator.update(sensors(tick=0, center=1.0, heading=10.0))
    held = translator.update(sensors(tick=1, visible=False, dt_s=0.1))
    expired = translator.update(sensors(tick=2, visible=False, dt_s=CAMERA_HOLD_SECONDS))

    assert held.center_offset_m == 1.0
    assert held.camera_missing_seconds == pytest.approx(0.1)
    assert expired.camera_missing_seconds > CAMERA_HOLD_SECONDS
    assert expired.center_offset_m == 0.0
    assert expired.heading_error_degrees == 0.0


def test_expert_steers_with_errors_and_opposes_yaw_rate() -> None:
    base = SensorTranslator().update(sensors())
    right_error = replace(base, center_offset_m=1.0, heading_error_degrees=10.0, path_bend=0.2)
    right_yaw = replace(right_error, yaw_rate_degrees_per_s=100.0)
    expert = CenterlineExpert(AUTHORED_PARAMETERS)

    decision = expert.decide(right_error)
    damped = expert.decide(right_yaw)

    assert decision.target_center_offset_m == 0.0
    assert decision.center_steer > 0.0
    assert decision.heading_steer > 0.0
    assert decision.bend_steer > 0.0
    assert damped.command.steer < decision.command.steer


def test_expert_accelerates_below_target_and_brakes_above_target() -> None:
    base = SensorTranslator().update(sensors())
    expert = CenterlineExpert(AUTHORED_PARAMETERS)

    accelerating = expert.decide(replace(base, speed_mps=0.0))
    braking = expert.decide(replace(base, speed_mps=20.0))

    assert accelerating.command.throttle > 0.0
    assert braking.command.throttle < 0.0
    assert braking.target_speed_mps == AUTHORED_PARAMETERS.straight_target_speed_mps


def test_safety_filter_steers_toward_open_side_and_emergency_brakes() -> None:
    state = SensorTranslator().update(sensors(walls=(0.5, inf, inf, 0.5, inf, inf, 5.0)))
    command = SafetyFilter().apply(state, RobotCommand(throttle=0.8, steer=0.0))

    assert command.steer > 0.0
    assert command.throttle == -0.2


def test_controller_smooths_commands_and_factories_have_independent_state() -> None:
    first = create_centerline_v1()
    second = create_centerline_v1()
    first_command = first(sensors(tick=0, center=1.0, heading=20.0))
    first(sensors(tick=1, center=1.0, heading=20.0))

    assert isinstance(first, CenterlineController)
    assert first is not second
    assert first.translator is not second.translator
    assert 0.0 < first_command.steer < 1.0
    assert second.last_state is None


def test_forward_drive_adapter_pulses_then_neutralizes_before_acceleration() -> None:
    adapter = ForwardOnlyDriveAdapter()
    braking = RobotCommand(throttle=-0.6, steer=0.25)

    pulse = adapter.apply(speed_mps=12.0, command=braking)
    neutral = adapter.apply(speed_mps=11.0, command=RobotCommand(throttle=0.8, steer=0.1))
    acceleration = adapter.apply(speed_mps=10.0, command=RobotCommand(throttle=0.8, steer=0.1))

    assert pulse.event == "brake_pulse" and pulse.command.throttle == -0.6
    assert neutral.event == "neutral_release" and neutral.command.throttle == 0.0
    assert acceleration.event == "pass_through" and acceleration.command.throttle == 0.8


def test_forward_drive_adapter_clears_physics_pending_direction() -> None:
    adapter = ForwardOnlyDriveAdapter()
    pulse = adapter.apply(speed_mps=10.0, command=RobotCommand(throttle=-0.5))
    first = resolve_vehicle_actuator_command(
        command=pulse.command, current_speed_kmh=36.0, config=FORMULA_VEHICLE_PHYSICS_CONFIG
    )
    neutral = adapter.apply(speed_mps=9.0, command=RobotCommand(throttle=0.8))
    second = resolve_vehicle_actuator_command(
        command=neutral.command,
        current_speed_kmh=32.0,
        config=FORMULA_VEHICLE_PHYSICS_CONFIG,
        pending_drive_direction=first.next_pending_drive_direction,
    )
    acceleration = adapter.apply(speed_mps=9.0, command=RobotCommand(throttle=0.8))
    third = resolve_vehicle_actuator_command(
        command=acceleration.command,
        current_speed_kmh=32.0,
        config=FORMULA_VEHICLE_PHYSICS_CONFIG,
        pending_drive_direction=second.next_pending_drive_direction,
    )

    assert first.brake_force > 0.0 and first.next_pending_drive_direction == -1
    assert second.brake_force == 0.0 and second.next_pending_drive_direction == 0
    assert third.engine_force > 0.0 and third.brake_force == 0.0


def test_forward_drive_adapter_suppresses_low_speed_braking_and_resets() -> None:
    adapter = ForwardOnlyDriveAdapter()
    held = adapter.apply(speed_mps=BRAKE_CUTOFF_MPS, command=RobotCommand(throttle=-1.0))
    adapter.apply(speed_mps=10.0, command=RobotCommand(throttle=-1.0))
    adapter.reset()
    passed = adapter.apply(speed_mps=10.0, command=RobotCommand(throttle=0.7))

    assert held.event == "low_speed_hold" and held.command.throttle == 0.0
    assert passed.event == "pass_through" and passed.command.throttle == 0.7


def test_corrected_factories_are_independent_and_legacy_factories_stay_legacy() -> None:
    legacy_v1 = create_centerline_v1()
    legacy_v2 = create_centerline_v2()
    corrected_v3 = create_centerline_v3()
    corrected_v4 = create_centerline_v4()

    assert legacy_v1.drive_adapter is None and legacy_v2.drive_adapter is None
    assert corrected_v3.drive_adapter is not None and corrected_v4.drive_adapter is not None
    assert corrected_v3 is not create_centerline_v3()
    assert corrected_v3.parameters == legacy_v2.parameters


def test_corrected_controller_interlocks_emergency_and_camera_loss_braking() -> None:
    controller = create_centerline_v3()
    emergency = controller(sensors(tick=0, speed=10.0, walls=(5.0, 5.0, 5.0, 0.5, 5.0, 5.0, 5.0)))
    release = controller(sensors(tick=1, speed=9.0, walls=(5.0, 5.0, 5.0, 0.5, 5.0, 5.0, 5.0)))

    assert emergency.throttle == -1.0
    assert release.throttle == 0.0

    command = release
    for tick in range(2, 20):
        command = controller(sensors(tick=tick, speed=8.0, visible=False, dt_s=0.05))
    assert command.throttle <= 0.0


def test_parameter_mapping_round_trips_and_clamps_search_values() -> None:
    normalized = to_normalized(AUTHORED_PARAMETERS)
    reconstructed = from_normalized(normalized)
    extremes = from_normalized([-2.0] * PARAMETER_COUNT)

    assert len(normalized) == PARAMETER_COUNT == 13
    assert astuple(reconstructed) == pytest.approx(astuple(AUTHORED_PARAMETERS))
    assert all(value >= -1.0 for value in to_normalized(extremes))


def metrics(
    *,
    seed: int,
    distance: float,
    laps: int = 2,
    damage: float = 0.0,
    wall: float = 0.0,
    eliminated: bool = False,
) -> TrialMetrics:
    return TrialMetrics(
        seed=seed,
        elapsed_seconds=30.0,
        track_length_m=180.0,
        raw_distance_m=distance,
        partial_laps=distance / 180.0,
        lap_count=laps,
        damage=damage,
        survived=not eliminated and damage < 1.0,
        eliminated=eliminated,
        wall_contact_seconds=wall,
        off_track_seconds=0.0,
        low_progress_seconds=0.0,
        max_speed_mps=18.0,
        first_lap_time_seconds=10.0 if laps else None,
        best_lap_time_seconds=10.0 if laps else None,
    )


def test_centerline_fitness_never_trades_damage_for_distance() -> None:
    safe = centerline_fitness([metrics(seed=1, distance=400.0)])
    unsafe = centerline_fitness([metrics(seed=1, distance=1000.0, damage=0.01)])

    assert safe.fitness > unsafe.fitness


def test_promotion_requires_safety_two_laps_and_baseline_improvement() -> None:
    baseline = [metrics(seed=1, distance=400.0), metrics(seed=2, distance=410.0)]
    passing = [metrics(seed=1, distance=401.0), metrics(seed=2, distance=411.0)]
    unsafe = [metrics(seed=1, distance=500.0, wall=0.01), metrics(seed=2, distance=500.0)]

    assert promotion_result(passing, baseline).passed
    assert not promotion_result(unsafe, baseline).passed


def diagnosed(
    *, seed: int, distance: float, stops: int = 0, reverse: float = 0.0, damage: float = 0.0
) -> DiagnosedTrial:
    return DiagnosedTrial(
        metrics(seed=seed, distance=distance, damage=damage),
        BrakingDiagnostics(0, 0, 0, reverse, stops, stops, 0.2 * stops),
    )


def test_braking_fitness_prioritizes_safety_then_stops_then_pace() -> None:
    safe = braking_fitness([diagnosed(seed=1, distance=400.0)])
    stopped = braking_fitness([diagnosed(seed=1, distance=500.0, stops=1)])
    reversed_trial = braking_fitness([diagnosed(seed=1, distance=1000.0, reverse=0.1)])
    damaged = braking_fitness([diagnosed(seed=1, distance=2000.0, damage=0.01)])

    assert safe.fitness > stopped.fitness > reversed_trial.fitness > damaged.fitness


def test_braking_promotion_requires_no_stops_reverse_and_v2_improvement() -> None:
    baseline = [diagnosed(seed=1, distance=400.0), diagnosed(seed=2, distance=410.0)]
    passing = [diagnosed(seed=1, distance=401.0), diagnosed(seed=2, distance=411.0)]
    stopped = [diagnosed(seed=1, distance=500.0, stops=1), diagnosed(seed=2, distance=500.0)]

    assert braking_promotion_result(passing, baseline)[0]
    assert not braking_promotion_result(stopped, baseline)[0]


def test_diagnostics_count_non_emergency_stop_and_exempt_recent_emergency() -> None:
    ordinary = DiagnosticCenterlineController(create_centerline_v3())
    emergency = DiagnosticCenterlineController(create_centerline_v3())
    for tick, speed in enumerate((6.0,) * 12 + (0.0,) * 18 + (2.0,) * 12):
        ordinary(sensors(tick=tick, speed=speed))
        walls = (5.0, 5.0, 5.0, 0.5, 5.0, 5.0, 5.0) if tick == 12 else (5.0,) * 7
        emergency(sensors(tick=tick, speed=speed, walls=walls))

    assert ordinary.diagnostics().non_emergency_stop_episodes == 1
    assert emergency.diagnostics().near_stop_episodes == 1
    assert emergency.diagnostics().non_emergency_stop_episodes == 0
