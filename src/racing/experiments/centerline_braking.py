"""Diagnostics, fitness, and promotion gates for corrected centerline braking."""

from __future__ import annotations

from dataclasses import dataclass
from statistics import mean

from controllers.centerline import CenterlineController
from controllers.centerline.policy import WALL_EMERGENCY_THRESHOLD_M
from racing import RobotCommand, RobotSensors
from racing.experiments.neuroevolution import HeadlessPolicyEvaluator, TrialMetrics

REVERSE_THRESHOLD_MPS = -0.1
LAUNCH_THRESHOLD_MPS = 5.0
NEAR_STOP_THRESHOLD_MPS = 1.2
NEAR_STOP_EXIT_MPS = 1.5
NEAR_STOP_MIN_TICKS = 3
EMERGENCY_LOOKBACK_S = 1.0


@dataclass(frozen=True, slots=True)
class BrakingDiagnostics:
    brake_pulse_ticks: int
    neutral_release_ticks: int
    low_speed_hold_ticks: int
    reverse_seconds: float
    near_stop_episodes: int
    non_emergency_stop_episodes: int
    non_emergency_stop_seconds: float

    def to_dict(self) -> dict[str, int | float]:
        return {
            "brake_pulse_ticks": self.brake_pulse_ticks,
            "neutral_release_ticks": self.neutral_release_ticks,
            "low_speed_hold_ticks": self.low_speed_hold_ticks,
            "reverse_seconds": self.reverse_seconds,
            "near_stop_episodes": self.near_stop_episodes,
            "non_emergency_stop_episodes": self.non_emergency_stop_episodes,
            "non_emergency_stop_seconds": self.non_emergency_stop_seconds,
        }


@dataclass(frozen=True, slots=True)
class DiagnosedTrial:
    metrics: TrialMetrics
    diagnostics: BrakingDiagnostics

    def to_dict(self) -> dict[str, object]:
        return {**self.metrics.to_dict(), "braking_diagnostics": self.diagnostics.to_dict()}


class DiagnosticCenterlineController:
    """Collect bounded episode counters without changing controller commands."""

    def __init__(self, controller: CenterlineController) -> None:
        self.controller = controller
        self._last_tick: int | None = None
        self._launched = False
        self._emergency_age_s = float("inf")
        self._low_ticks = 0
        self._low_seconds = 0.0
        self._low_emergency = False
        self._brake_pulses = 0
        self._neutral_releases = 0
        self._low_speed_holds = 0
        self._reverse_seconds = 0.0
        self._near_stops = 0
        self._non_emergency_stops = 0
        self._non_emergency_stop_seconds = 0.0

    def __call__(self, sensors: RobotSensors) -> RobotCommand:
        if self._last_tick is not None and sensors.tick <= self._last_tick:
            self.__init__(self.controller)
        command = self.controller(sensors)
        state = self.controller.last_state
        if state is None:
            return command
        event = self.controller.last_drive_decision
        if event is not None:
            self._brake_pulses += event.event == "brake_pulse"
            self._neutral_releases += event.event == "neutral_release"
            self._low_speed_holds += event.event == "low_speed_hold"

        emergency = state.front_wall_m < WALL_EMERGENCY_THRESHOLD_M
        self._emergency_age_s = 0.0 if emergency else self._emergency_age_s + state.dt_s
        if state.speed_mps < REVERSE_THRESHOLD_MPS:
            self._reverse_seconds += state.dt_s
        if state.speed_mps >= LAUNCH_THRESHOLD_MPS:
            self._launched = True
        if self._launched and abs(state.speed_mps) < NEAR_STOP_THRESHOLD_MPS:
            self._low_ticks += 1
            self._low_seconds += state.dt_s
            self._low_emergency = self._low_emergency or self._emergency_age_s <= EMERGENCY_LOOKBACK_S
        elif abs(state.speed_mps) > NEAR_STOP_EXIT_MPS:
            self._finish_episode()
        self._last_tick = sensors.tick
        return command

    def diagnostics(self) -> BrakingDiagnostics:
        self._finish_episode()
        return BrakingDiagnostics(
            brake_pulse_ticks=self._brake_pulses,
            neutral_release_ticks=self._neutral_releases,
            low_speed_hold_ticks=self._low_speed_holds,
            reverse_seconds=self._reverse_seconds,
            near_stop_episodes=self._near_stops,
            non_emergency_stop_episodes=self._non_emergency_stops,
            non_emergency_stop_seconds=self._non_emergency_stop_seconds,
        )

    def _finish_episode(self) -> None:
        if self._low_ticks >= NEAR_STOP_MIN_TICKS:
            self._near_stops += 1
            if not self._low_emergency:
                self._non_emergency_stops += 1
                self._non_emergency_stop_seconds += self._low_seconds
        self._low_ticks = 0
        self._low_seconds = 0.0
        self._low_emergency = False


def evaluate_diagnosed(
    evaluator: HeadlessPolicyEvaluator,
    controller: CenterlineController,
    *,
    seed: int,
    seconds: float,
) -> DiagnosedTrial:
    diagnostic = DiagnosticCenterlineController(controller)
    metrics = evaluator.evaluate_controller(diagnostic, seed=seed, seconds=seconds)
    return DiagnosedTrial(metrics, diagnostic.diagnostics())


@dataclass(frozen=True, slots=True)
class BrakingFitness:
    fitness: float
    eliminations: int
    damaged_runs: int
    total_damage: float
    wall_contact_runs: int
    wall_contact_seconds: float
    missing_two_lap_runs: int
    reverse_runs: int
    reverse_seconds: float
    non_emergency_stop_episodes: int
    non_emergency_stop_seconds: float
    off_track_seconds: float
    minimum_distance_m: float
    mean_distance_m: float
    mean_best_lap_seconds: float


def braking_fitness(trials: tuple[DiagnosedTrial, ...] | list[DiagnosedTrial]) -> BrakingFitness:
    if not trials:
        raise ValueError("braking fitness requires at least one trial")
    metrics = tuple(trial.metrics for trial in trials)
    eliminations = sum(trial.eliminated or not trial.survived for trial in metrics)
    damaged_runs = sum(trial.damage > 0.0 for trial in metrics)
    total_damage = sum(trial.damage for trial in metrics)
    wall_runs = sum(trial.wall_contact_seconds > 0.0 for trial in metrics)
    wall_seconds = sum(trial.wall_contact_seconds for trial in metrics)
    missing_laps = sum(trial.lap_count < 2 for trial in metrics)
    reverse_runs = sum(trial.diagnostics.reverse_seconds > 0.0 for trial in trials)
    reverse_seconds = sum(trial.diagnostics.reverse_seconds for trial in trials)
    stop_episodes = sum(trial.diagnostics.non_emergency_stop_episodes for trial in trials)
    stop_seconds = sum(trial.diagnostics.non_emergency_stop_seconds for trial in trials)
    off_track = sum(trial.off_track_seconds for trial in metrics)
    distances = tuple(trial.raw_distance_m for trial in metrics)
    lap_times = tuple(trial.best_lap_time_seconds for trial in metrics if trial.best_lap_time_seconds is not None)
    mean_lap = mean(lap_times) if lap_times else 30.0
    fitness = (
        -1_000_000_000_000.0 * eliminations
        - 100_000_000_000.0 * damaged_runs
        - 10_000_000_000.0 * total_damage
        - 1_000_000_000.0 * wall_runs
        - 100_000_000.0 * wall_seconds
        - 10_000_000.0 * missing_laps
        - 1_000_000.0 * reverse_runs
        - 100_000.0 * reverse_seconds
        - 10_000.0 * stop_episodes
        - 1_000.0 * stop_seconds
        - 1_000.0 * off_track
        + min(distances)
        + 0.25 * mean(distances)
        - mean_lap
    )
    return BrakingFitness(
        fitness,
        eliminations,
        damaged_runs,
        total_damage,
        wall_runs,
        wall_seconds,
        missing_laps,
        reverse_runs,
        reverse_seconds,
        stop_episodes,
        stop_seconds,
        off_track,
        min(distances),
        mean(distances),
        mean_lap,
    )


def braking_promotion_result(
    candidate: tuple[DiagnosedTrial, ...] | list[DiagnosedTrial],
    baseline: tuple[DiagnosedTrial, ...] | list[DiagnosedTrial],
) -> tuple[bool, tuple[str, ...]]:
    if not candidate or len(candidate) != len(baseline):
        raise ValueError("candidate and baseline require equal non-empty seed sets")
    reasons: list[str] = []
    for trial in candidate:
        metrics = trial.metrics
        if metrics.eliminated or not metrics.survived:
            reasons.append(f"seed {metrics.seed} did not survive")
        if metrics.lap_count < 2:
            reasons.append(f"seed {metrics.seed} completed fewer than two laps")
        if metrics.damage != 0.0:
            reasons.append(f"seed {metrics.seed} recorded damage")
        if metrics.wall_contact_seconds != 0.0:
            reasons.append(f"seed {metrics.seed} recorded wall contact")
        if trial.diagnostics.reverse_seconds != 0.0:
            reasons.append(f"seed {metrics.seed} reversed")
        if trial.diagnostics.non_emergency_stop_episodes != 0:
            reasons.append(f"seed {metrics.seed} made a non-emergency stop")
    baseline_by_seed = {trial.metrics.seed: trial.metrics for trial in baseline}
    for trial in candidate:
        reference = baseline_by_seed.get(trial.metrics.seed)
        if reference is None:
            reasons.append(f"seed {trial.metrics.seed} is missing from baseline")
        elif trial.metrics.raw_distance_m < reference.raw_distance_m - 1.0:
            reasons.append(f"seed {trial.metrics.seed} regressed by more than one metre")
    if mean(trial.metrics.raw_distance_m for trial in candidate) <= mean(
        trial.metrics.raw_distance_m for trial in baseline
    ):
        reasons.append("mean distance did not exceed centerline_v2")
    return not reasons, tuple(reasons)
