"""Fitness and promotion gates for structured racing-line optimization."""

from __future__ import annotations

from dataclasses import dataclass
from statistics import mean

from racing.experiments.centerline_braking import DiagnosedTrial


@dataclass(frozen=True, slots=True)
class RacingLineFitness:
    fitness: float
    eliminations: int
    damaged_runs: int
    total_damage: float
    wall_contact_runs: int
    wall_contact_seconds: float
    missing_three_lap_runs: int
    reverse_runs: int
    reverse_seconds: float
    non_emergency_stop_episodes: int
    non_emergency_stop_seconds: float
    off_track_runs: int
    off_track_seconds: float
    minimum_distance_m: float
    mean_distance_m: float
    mean_best_lap_seconds: float


@dataclass(frozen=True, slots=True)
class RacingLinePromotion:
    passed: bool
    reasons: tuple[str, ...]


def racing_line_fitness(trials: tuple[DiagnosedTrial, ...] | list[DiagnosedTrial]) -> RacingLineFitness:
    if not trials:
        raise ValueError("racing-line fitness requires at least one trial")
    metrics = tuple(trial.metrics for trial in trials)
    eliminations = sum(trial.eliminated or not trial.survived for trial in metrics)
    damaged_runs = sum(trial.damage > 0.0 for trial in metrics)
    damage = sum(trial.damage for trial in metrics)
    wall_runs = sum(trial.wall_contact_seconds > 0.0 for trial in metrics)
    wall_seconds = sum(trial.wall_contact_seconds for trial in metrics)
    missing_laps = sum(trial.lap_count < 3 for trial in metrics)
    reverse_runs = sum(trial.diagnostics.reverse_seconds > 0.0 for trial in trials)
    reverse_seconds = sum(trial.diagnostics.reverse_seconds for trial in trials)
    stop_episodes = sum(trial.diagnostics.non_emergency_stop_episodes for trial in trials)
    stop_seconds = sum(trial.diagnostics.non_emergency_stop_seconds for trial in trials)
    off_track_runs = sum(trial.off_track_seconds > 0.0 for trial in metrics)
    off_track_seconds = sum(trial.off_track_seconds for trial in metrics)
    distances = tuple(trial.raw_distance_m for trial in metrics)
    lap_times = tuple(trial.best_lap_time_seconds for trial in metrics if trial.best_lap_time_seconds is not None)
    mean_lap = mean(lap_times) if lap_times else 30.0
    fitness = (
        -1_000_000_000_000_000.0 * eliminations
        - 100_000_000_000_000.0 * damaged_runs
        - 10_000_000_000_000.0 * damage
        - 1_000_000_000_000.0 * wall_runs
        - 100_000_000_000.0 * wall_seconds
        - 10_000_000_000.0 * missing_laps
        - 1_000_000_000.0 * reverse_runs
        - 100_000_000.0 * reverse_seconds
        - 10_000_000.0 * stop_episodes
        - 1_000_000.0 * stop_seconds
        - 100_000.0 * off_track_runs
        - 10_000.0 * off_track_seconds
        + min(distances)
        + 0.25 * mean(distances)
        - mean_lap
    )
    return RacingLineFitness(
        fitness,
        eliminations,
        damaged_runs,
        damage,
        wall_runs,
        wall_seconds,
        missing_laps,
        reverse_runs,
        reverse_seconds,
        stop_episodes,
        stop_seconds,
        off_track_runs,
        off_track_seconds,
        min(distances),
        mean(distances),
        mean_lap,
    )


def racing_line_promotion(
    candidate: tuple[DiagnosedTrial, ...] | list[DiagnosedTrial],
    centerline_baseline: tuple[DiagnosedTrial, ...] | list[DiagnosedTrial],
    incumbent: tuple[DiagnosedTrial, ...] | list[DiagnosedTrial],
) -> RacingLinePromotion:
    if not candidate or len(candidate) != len(centerline_baseline) or len(candidate) != len(incumbent):
        raise ValueError("candidate and both baselines require equal non-empty seed sets")
    reasons: list[str] = []
    for trial in candidate:
        m = trial.metrics
        if m.eliminated or not m.survived:
            reasons.append(f"seed {m.seed} did not survive")
        if m.lap_count < 3:
            reasons.append(f"seed {m.seed} completed fewer than three laps")
        if m.damage != 0.0:
            reasons.append(f"seed {m.seed} recorded damage")
        if m.wall_contact_seconds != 0.0:
            reasons.append(f"seed {m.seed} recorded wall contact")
        if m.off_track_seconds != 0.0:
            reasons.append(f"seed {m.seed} went off track")
        if trial.diagnostics.reverse_seconds != 0.0:
            reasons.append(f"seed {m.seed} reversed")
        if trial.diagnostics.non_emergency_stop_episodes != 0:
            reasons.append(f"seed {m.seed} made a non-emergency stop")
    for label, reference_trials in (("centerline_v4", centerline_baseline), ("incumbent", incumbent)):
        reference = {trial.metrics.seed: trial.metrics for trial in reference_trials}
        for trial in candidate:
            baseline = reference.get(trial.metrics.seed)
            if baseline is None:
                reasons.append(f"seed {trial.metrics.seed} is missing from {label}")
            elif trial.metrics.raw_distance_m < baseline.raw_distance_m - 1.0:
                reasons.append(f"seed {trial.metrics.seed} regressed by more than one metre versus {label}")
        if mean(trial.metrics.raw_distance_m for trial in candidate) <= mean(
            trial.metrics.raw_distance_m for trial in reference_trials
        ):
            reasons.append(f"mean distance did not exceed {label}")
    return RacingLinePromotion(not reasons, tuple(reasons))
