#!/usr/bin/env python3
"""Audit centerline artifacts and preserved baselines on a fixed seed suite."""

from __future__ import annotations

import argparse
import json
from collections.abc import Callable
from pathlib import Path
from typing import cast

from controllers.centerline_v1 import create_controller as create_centerline_v1
from controllers.centerline_v2 import create_controller as create_centerline_v2
from controllers.cmaes_v2 import create_controller as create_cmaes_v2
from controllers.reactive_v1 import control as reactive_v1
from racing.experiments.neuroevolution import HeadlessPolicyEvaluator, TrialMetrics
from racing.student.api import RobotController

BROAD_VALIDATION_SEEDS = (3, 29, 61, 107, 173, 257, 349, 433, 547, 661, 773, 997, 1109, 1223, 1301)
OFFICIAL_SEEDS = (110, 2026)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=_seed_list, default=(*BROAD_VALIDATION_SEEDS, *OFFICIAL_SEEDS))
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--output", type=Path, default=Path("artifacts/centerline-cmaes/final_evaluation.json"))
    return parser.parse_args()


def _seed_list(value: str) -> tuple[int, ...]:
    seeds = tuple(int(item.strip()) for item in value.split(",") if item.strip())
    if not seeds:
        raise argparse.ArgumentTypeError("at least one seed is required")
    return seeds


def _summarize(trials: tuple[TrialMetrics, ...]) -> dict[str, object]:
    return {
        "run_count": len(trials),
        "completed_two_laps": sum(trial.lap_count >= 2 for trial in trials),
        "eliminations": sum(trial.eliminated for trial in trials),
        "total_damage": sum(trial.damage for trial in trials),
        "wall_contact_seconds": sum(trial.wall_contact_seconds for trial in trials),
        "off_track_seconds": sum(trial.off_track_seconds for trial in trials),
        "mean_distance_m": sum(trial.raw_distance_m for trial in trials) / len(trials),
        "mean_best_lap_seconds": sum(
            trial.best_lap_time_seconds if trial.best_lap_time_seconds is not None else 30.0 for trial in trials
        )
        / len(trials),
    }


def main() -> None:
    args = parse_args()
    if args.seconds <= 0.0:
        raise ValueError("seconds must be positive")
    factories: tuple[tuple[str, Callable[[], RobotController]], ...] = (
        ("reactive_v1", lambda: cast(RobotController, reactive_v1)),
        ("centerline_v1", create_centerline_v1),
        ("centerline_v2", create_centerline_v2),
        ("cmaes_v2", create_cmaes_v2),
    )
    results: dict[str, object] = {
        "schema_version": 1,
        "seconds": float(args.seconds),
        "seeds": list(args.seeds),
        "broad_validation_seeds": list(BROAD_VALIDATION_SEEDS),
        "official_seeds": list(OFFICIAL_SEEDS),
        "controllers": {},
    }
    controller_results = cast(dict[str, object], results["controllers"])
    with HeadlessPolicyEvaluator() as evaluator:
        for name, factory in factories:
            trials = tuple(
                evaluator.evaluate_controller(factory(), seed=seed, seconds=float(args.seconds)) for seed in args.seeds
            )
            summary = _summarize(trials)
            controller_results[name] = {
                "summary": summary,
                "trials": [trial.to_dict() for trial in trials],
            }
            print(f"{name}: {summary}", flush=True)

    output = cast(Path, args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(results, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
