"""Deterministic evaluation and robust fitness for CMA-ES neuroevolution."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
from importlib import import_module
from math import sqrt
from typing import Any, cast

from controllers.cmaes_policy import FixedMLPPolicy
from racing.graphics.panda_config import configure_headless_panda
from racing.graphics.track_rendering import add_racing_scene_collisions
from racing.physics import (
    FORMULA_VEHICLE_PHYSICS_CONFIG,
    PhysicsScene,
    apply_robot_vehicle_command,
    apply_wall_impact_damage,
    create_physics_world,
    create_robot_vehicle,
)
from racing.race.progress import default_track_progress_model, project_track_position
from racing.race.runtime import (
    RaceCarRuntime,
    lap_progress_tracker_for_spawn_pose,
    race_contact_states,
    race_spawn_poses,
    robot_is_eliminated,
    robot_score_damage,
    robot_track_point,
    update_race_runtime_after_step,
)
from racing.race.sensors import build_robot_sensors
from racing.student.api import RobotController

FIXED_DELTA_SECONDS = 1.0 / 60.0


@dataclass(frozen=True, slots=True)
class TrialMetrics:
    seed: int
    elapsed_seconds: float
    track_length_m: float
    raw_distance_m: float
    partial_laps: float
    lap_count: int
    damage: float
    survived: bool
    eliminated: bool
    wall_contact_seconds: float
    off_track_seconds: float
    low_progress_seconds: float
    max_speed_mps: float
    first_lap_time_seconds: float | None
    best_lap_time_seconds: float | None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class FitnessResult:
    fitness: float
    seed_scores: tuple[float, ...]
    minimum_score: float
    mean_score: float
    score_stddev: float


class HeadlessPolicyEvaluator:
    """Reuse Panda initialization while giving every trial fresh race state."""

    def __init__(self) -> None:
        configure_headless_panda()
        showbase = cast(Any, import_module("direct.showbase.ShowBase"))
        self._base = showbase.ShowBase(windowType="none")
        self._trial_index = 0

    def close(self) -> None:
        self._base.destroy()

    def __enter__(self) -> HeadlessPolicyEvaluator:
        return self

    def __exit__(self, _exc_type: object, _exc_value: object, _traceback: object) -> None:
        self.close()

    def evaluate(self, parameters: Sequence[float], *, seed: int, seconds: float) -> TrialMetrics:
        return self.evaluate_controller(FixedMLPPolicy(parameters), seed=seed, seconds=seconds)

    def evaluate_controller(self, policy: RobotController, *, seed: int, seconds: float) -> TrialMetrics:
        """Evaluate any public controller with the same isolated trial machinery."""
        if seconds <= 0.0:
            raise ValueError("trial duration must be positive")
        self._trial_index += 1
        model = default_track_progress_model()
        physics_world = create_physics_world()
        physics_scene = PhysicsScene(world=physics_world, vehicles=[])
        root = self._base.render.attachNewNode(f"cmaes-trial-{self._trial_index}-{seed}")
        try:
            add_racing_scene_collisions(physics_world=physics_world, render=root)
            spawn_pose = race_spawn_poses(
                1,
                model=model,
                config=FORMULA_VEHICLE_PHYSICS_CONFIG,
                random_seed=seed,
                race_index=1,
            )[0]
            robot = create_robot_vehicle(
                world=physics_world,
                render=root,
                name=f"cmaes-car-{self._trial_index}",
                position=spawn_pose.position,
                heading_degrees=spawn_pose.heading_degrees,
                config=FORMULA_VEHICLE_PHYSICS_CONFIG,
            )
            physics_scene.vehicles.append(robot)
            runtime = RaceCarRuntime(
                robot=robot,
                tracker=lap_progress_tracker_for_spawn_pose(model=model, spawn_pose=spawn_pose),
            )
            elapsed_seconds = 0.0
            while elapsed_seconds < seconds:
                if not robot_is_eliminated(robot):
                    sensors, runtime.sensor_state = build_robot_sensors(
                        physics_world=physics_world,
                        robot=robot,
                        track_model=model,
                        time_s=elapsed_seconds,
                        dt_s=FIXED_DELTA_SECONDS,
                        previous_state=runtime.sensor_state,
                    )
                    apply_robot_vehicle_command(robot=robot, command=policy(sensors))
                physics_scene.step(FIXED_DELTA_SECONDS)
                next_elapsed = min(seconds, elapsed_seconds + FIXED_DELTA_SECONDS)
                contact_state = race_contact_states(physics_world=physics_world, runtimes=(runtime,))[0]
                apply_wall_impact_damage(
                    physics_world=physics_world,
                    robots=(robot,),
                    fixed_time_step=physics_scene.fixed_time_step,
                )
                projection = project_track_position(model, robot_track_point(robot))
                update_race_runtime_after_step(
                    runtime=runtime,
                    projection=projection,
                    contact_state=contact_state,
                    elapsed_seconds=next_elapsed,
                    delta_seconds=FIXED_DELTA_SECONDS,
                )
                elapsed_seconds = next_elapsed

            lap_crossing_times = tuple(runtime.tracker.lap_times_seconds)
            lap_durations = tuple(
                crossing - (lap_crossing_times[index - 1] if index else 0.0)
                for index, crossing in enumerate(lap_crossing_times)
            )
            damage = robot_score_damage(robot)
            return TrialMetrics(
                seed=seed,
                elapsed_seconds=elapsed_seconds,
                track_length_m=model.total_length_m,
                raw_distance_m=runtime.tracker.best_distance_m,
                partial_laps=runtime.tracker.best_distance_m / model.total_length_m,
                lap_count=runtime.tracker.lap_count,
                damage=damage,
                survived=not robot_is_eliminated(robot) and damage < 1.0,
                eliminated=robot_is_eliminated(robot),
                wall_contact_seconds=runtime.tracker.wall_contact_seconds,
                off_track_seconds=runtime.off_track_seconds,
                low_progress_seconds=runtime.low_progress_seconds,
                max_speed_mps=runtime.max_speed_mps,
                first_lap_time_seconds=lap_crossing_times[0] if lap_crossing_times else None,
                best_lap_time_seconds=min(lap_durations) if lap_durations else None,
            )
        finally:
            root.removeNode()


def trial_score(metrics: TrialMetrics) -> float:
    """Score useful pace while making damage and catastrophic failure expensive."""
    duration = max(metrics.elapsed_seconds, FIXED_DELTA_SECONDS)
    lap_bonus = 0.0
    if metrics.first_lap_time_seconds is not None:
        lap_bonus = 0.5 * max(0.0, 1.0 - metrics.first_lap_time_seconds / duration)
    elimination = 1.0 if metrics.eliminated else 0.0
    survival_bonus = 0.1 if metrics.survived else 0.0
    return (
        metrics.partial_laps
        + lap_bonus
        + survival_bonus
        - metrics.off_track_seconds / duration
        - metrics.wall_contact_seconds / duration
        - 1.5 * metrics.damage
        - 1.5 * elimination
    )


def racing_line_trial_score(metrics: TrialMetrics) -> float:
    """Prioritize fast progress, but make contact and damage unacceptable tradeoffs."""
    duration = max(metrics.elapsed_seconds, FIXED_DELTA_SECONDS)
    lap_bonus = 0.0
    if metrics.first_lap_time_seconds is not None:
        lap_bonus = 0.75 * max(0.0, 1.0 - metrics.first_lap_time_seconds / duration)
    elimination = 1.0 if metrics.eliminated else 0.0
    return (
        metrics.partial_laps
        + lap_bonus
        + (0.1 if metrics.survived else 0.0)
        - 0.5 * metrics.off_track_seconds / duration
        - 3.0 * metrics.wall_contact_seconds / duration
        - 5.0 * metrics.damage
        - 5.0 * elimination
    )


def aggregate_fitness(
    trials: Sequence[TrialMetrics],
    *,
    score_function: Callable[[TrialMetrics], float] = trial_score,
) -> FitnessResult:
    """Emphasize the worst seed and penalize inconsistent controllers."""
    if not trials:
        raise ValueError("fitness requires at least one trial")
    scores = tuple(score_function(trial) for trial in trials)
    mean_score = sum(scores) / len(scores)
    variance = sum((score - mean_score) ** 2 for score in scores) / len(scores)
    score_stddev = sqrt(variance)
    minimum_score = min(scores)
    fitness = 0.6 * minimum_score + 0.4 * mean_score - 0.25 * score_stddev
    return FitnessResult(
        fitness=fitness,
        seed_scores=scores,
        minimum_score=minimum_score,
        mean_score=mean_score,
        score_stddev=score_stddev,
    )
