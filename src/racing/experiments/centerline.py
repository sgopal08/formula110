"""Fitness and promotion gates for the structured centerline experiment."""

from __future__ import annotations

from dataclasses import dataclass
from statistics import mean

from racing.experiments.neuroevolution import TrialMetrics


@dataclass(frozen=True, slots=True)
class CenterlineFitness:
    """Auditable components of the safety-tier centerline objective."""

    fitness: float
    eliminations: int
    total_damage: float
    wall_contact_seconds: float
    missing_two_lap_runs: int
    off_track_seconds: float
    minimum_distance_m: float
    mean_distance_m: float
    mean_best_lap_seconds: float


@dataclass(frozen=True, slots=True)
class PromotionResult:
    """Whether a candidate satisfies the frozen centerline promotion contract."""

    passed: bool
    reasons: tuple[str, ...]


def centerline_fitness(trials: tuple[TrialMetrics, ...] | list[TrialMetrics]) -> CenterlineFitness:
    """Rank safety failures before incomplete laps, containment, and pace."""
    if not trials:
        raise ValueError("centerline fitness requires at least one trial")
    eliminations = sum(1 for trial in trials if trial.eliminated or not trial.survived)
    total_damage = sum(trial.damage for trial in trials)
    wall_contact = sum(trial.wall_contact_seconds for trial in trials)
    missing_laps = sum(1 for trial in trials if trial.lap_count < 2)
    off_track = sum(trial.off_track_seconds for trial in trials)
    distances = tuple(trial.raw_distance_m for trial in trials)
    completed_laps = tuple(trial.best_lap_time_seconds for trial in trials if trial.best_lap_time_seconds is not None)
    mean_best_lap = mean(completed_laps) if completed_laps else 30.0
    fitness = (
        -1_000_000_000.0 * eliminations
        - 100_000_000.0 * total_damage
        - 1_000_000.0 * wall_contact
        - 100_000.0 * missing_laps
        - 100.0 * off_track
        + min(distances)
        + 0.25 * mean(distances)
        - mean_best_lap
    )
    return CenterlineFitness(
        fitness=fitness,
        eliminations=eliminations,
        total_damage=total_damage,
        wall_contact_seconds=wall_contact,
        missing_two_lap_runs=missing_laps,
        off_track_seconds=off_track,
        minimum_distance_m=min(distances),
        mean_distance_m=mean(distances),
        mean_best_lap_seconds=mean_best_lap,
    )


def promotion_result(
    candidate: tuple[TrialMetrics, ...] | list[TrialMetrics],
    baseline: tuple[TrialMetrics, ...] | list[TrialMetrics],
) -> PromotionResult:
    """Apply the validation safety and reactive-v1 performance gates."""
    if not candidate or len(candidate) != len(baseline):
        raise ValueError("candidate and baseline require equal non-empty seed sets")
    reasons: list[str] = []
    for trial in candidate:
        if trial.eliminated or not trial.survived:
            reasons.append(f"seed {trial.seed} did not survive")
        if trial.lap_count < 2:
            reasons.append(f"seed {trial.seed} completed fewer than two laps")
        if trial.damage != 0.0:
            reasons.append(f"seed {trial.seed} recorded damage")
        if trial.wall_contact_seconds != 0.0:
            reasons.append(f"seed {trial.seed} recorded wall contact")

    candidate_by_seed = {trial.seed: trial for trial in candidate}
    for baseline_trial in baseline:
        candidate_trial = candidate_by_seed.get(baseline_trial.seed)
        if candidate_trial is None:
            reasons.append(f"seed {baseline_trial.seed} is missing")
        elif candidate_trial.raw_distance_m < baseline_trial.raw_distance_m - 1.0:
            reasons.append(f"seed {baseline_trial.seed} regressed by more than one metre")
    if mean(trial.raw_distance_m for trial in candidate) <= mean(trial.raw_distance_m for trial in baseline):
        reasons.append("mean distance did not exceed reactive_v1")
    return PromotionResult(passed=not reasons, reasons=tuple(reasons))
