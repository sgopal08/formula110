#!/usr/bin/env python3
"""Audit legacy and corrected centerline artifacts with braking diagnostics."""

from __future__ import annotations

import argparse
import json
from collections.abc import Callable
from pathlib import Path
from typing import cast

from controllers.centerline import CenterlineController
from controllers.centerline_v2 import create_controller as create_v2
from controllers.centerline_v3 import create_controller as create_v3
from controllers.centerline_v4 import create_controller as create_v4
from racing.experiments.centerline_braking import evaluate_diagnosed
from racing.experiments.neuroevolution import HeadlessPolicyEvaluator

BROAD_SEEDS = (3, 29, 61, 107, 173, 257, 349, 433, 547, 661, 773, 997, 1109, 1223, 1301)
OFFICIAL_SEEDS = (110, 2026)


def _seeds(value: str) -> tuple[int, ...]:
    result = tuple(int(item.strip()) for item in value.split(",") if item.strip())
    if not result:
        raise argparse.ArgumentTypeError("at least one seed is required")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=_seeds, default=(*BROAD_SEEDS, *OFFICIAL_SEEDS))
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--output", type=Path, default=Path("artifacts/centerline-braking-cmaes/final_evaluation.json"))
    args = parser.parse_args()
    factories: tuple[tuple[str, Callable[[], CenterlineController]], ...] = (
        ("centerline_v2", create_v2),
        ("centerline_v3", create_v3),
        ("centerline_v4", create_v4),
    )
    output: dict[str, object] = {
        "schema_version": 2,
        "seconds": args.seconds,
        "seeds": list(args.seeds),
        "controllers": {},
    }
    controllers = cast(dict[str, object], output["controllers"])
    with HeadlessPolicyEvaluator() as evaluator:
        for name, factory in factories:
            trials = tuple(
                evaluate_diagnosed(evaluator, factory(), seed=seed, seconds=args.seconds) for seed in args.seeds
            )
            summary = {
                "runs": len(trials),
                "two_laps": sum(t.metrics.lap_count >= 2 for t in trials),
                "damage": sum(t.metrics.damage for t in trials),
                "wall_contact_seconds": sum(t.metrics.wall_contact_seconds for t in trials),
                "off_track_seconds": sum(t.metrics.off_track_seconds for t in trials),
                "reverse_seconds": sum(t.diagnostics.reverse_seconds for t in trials),
                "non_emergency_stops": sum(t.diagnostics.non_emergency_stop_episodes for t in trials),
                "mean_distance_m": sum(t.metrics.raw_distance_m for t in trials) / len(trials),
            }
            controllers[name] = {"summary": summary, "trials": [trial.to_dict() for trial in trials]}
            print(f"{name}: {summary}", flush=True)
    destination = cast(Path, args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
