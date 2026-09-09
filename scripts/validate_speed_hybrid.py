#!/usr/bin/env python3
"""Compare a frozen speed candidate with the prior hybrid on unused starting seeds."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean

from train_speed_hybrid import DEVELOPMENT_SEEDS, evaluate, write

from racing.experiments.neuroevolution import HeadlessPolicyEvaluator

# Fixed before training finishes; excluded from training and candidate selection.
FINAL_SEEDS = (
    149,
    281,
    397,
    563,
    677,
    809,
    941,
    1063,
    1187,
    1327,
    1451,
    1579,
    1693,
    1823,
    1951,
    2081,
    2203,
    2333,
    2467,
    2591,
)


def summary(result: dict) -> dict:
    trials = result["trials"]
    return dict(
        runs=len(trials),
        safe=result["safe"],
        mean_distance=result["mean_distance"],
        mean_best_lap=mean(t["best_lap_time_seconds"] for t in trials if t["best_lap_time_seconds"] is not None),
        mean_start_speed=mean(t["start_mean_speed"] for t in trials),
        mean_corner_speed=mean(t["corner_mean_speed"] for t in trials),
        mean_straight_speed=mean(t["straight_mean_speed"] for t in trials),
        mean_time_to_10=mean(t["time_to_10"] for t in trials if t["time_to_10"] is not None),
        mean_absolute_center_offset=mean(t["mean_absolute_center_offset"] for t in trials),
        mean_corner_throttle=mean(t["corner_mean_throttle"] for t in trials),
        total_damage=sum(t["damage"] for t in trials),
        total_wall_contact=sum(t["wall_contact_seconds"] for t in trials),
        total_off_track=sum(t["off_track_seconds"] for t in trials),
        minimum_laps=min(t["lap_count"] for t in trials),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", type=Path, default=Path("artifacts/cmaes-speed-hybrid"))
    parser.add_argument("--seed-offset", type=int, default=0, help="Offset the final seeds for a new untouched suite.")
    parser.add_argument(
        "--selection-source", type=Path, help="Read ranked development candidates from this experiment."
    )
    parser.add_argument(
        "--candidate-rank", type=int, default=1, help="One-based rank among feasible development results."
    )
    args = parser.parse_args()
    if args.selection_source is not None:
        candidates = json.loads((args.selection_source / "development.json").read_text())
        ranked = sorted((r for r in candidates if r["safe"]), key=lambda r: r["score"], reverse=True)
        if not 1 <= args.candidate_rank <= len(ranked):
            parser.error("candidate-rank is outside the feasible development candidates")
        args.experiment.mkdir(parents=True, exist_ok=False)
        write(args.experiment / "selected.json", ranked[args.candidate_rank - 1])
        write(
            args.experiment / "selection-provenance.json",
            dict(
                source=str(args.selection_source),
                rank=args.candidate_rank,
            ),
        )
    final_seeds = tuple(seed + args.seed_offset for seed in FINAL_SEEDS)
    write(
        args.experiment / "validation-configuration.json", dict(final_seeds=final_seeds, seed_offset=args.seed_offset)
    )
    selected = json.loads((args.experiment / "selected.json").read_text())["parameters"]
    results = {}
    with HeadlessPolicyEvaluator() as evaluator:
        for name, parameters in [("previous", [0.0] * 8), ("candidate", selected)]:
            result = evaluate(evaluator, parameters, final_seeds)
            results[name] = result
            print(name, json.dumps(summary(result)), flush=True)
            write(args.experiment / "final-validation.json", results)
        for name, parameters in [("previous", [0.0] * 8), ("candidate", selected)]:
            result = evaluate(evaluator, parameters, (110, 2026))
            results[name + "_official"] = result
            print(name + "_official", json.dumps(summary(result)), flush=True)
            write(args.experiment / "final-validation.json", results)
        # Four times the training horizon to detect cumulative drift or delayed contact.
        for name, parameters in [("previous", [0.0] * 8), ("candidate", selected)]:
            result = evaluate(evaluator, parameters, (149, 809, 1451, 110, 2026), seconds=120)
            results[name + "_long"] = result
            print(name + "_long", json.dumps(summary(result)), flush=True)
            write(args.experiment / "final-validation.json", results)
    write(args.experiment / "final-summary.json", {k: summary(v) for k, v in results.items()})
    # Explain the frozen controller on development seeds, without selecting a new one.
    ablations = {}
    with HeadlessPolicyEvaluator() as evaluator:
        for name, indices in [("full", ()), ("without_launch", (0,)), ("without_line_refinement", (3, 4, 5))]:
            parameters = list(selected)
            for index in indices:
                parameters[index] = 0.0
            result = evaluate(evaluator, parameters, DEVELOPMENT_SEEDS)
            ablations[name] = result
            print(name, json.dumps(summary(result)), flush=True)
            write(args.experiment / "ablations.json", ablations)
    accepted = all(results[key]["safe"] for key in ("candidate", "candidate_official", "candidate_long"))
    accepted = accepted and results["candidate"]["mean_distance"] > results["previous"]["mean_distance"]
    write(args.experiment / "acceptance.json", dict(accepted=accepted))
    if not accepted:
        raise SystemExit("Candidate rejected: safety or progress acceptance failed; do not export it.")


if __name__ == "__main__":
    main()
