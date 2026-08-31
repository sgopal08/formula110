#!/usr/bin/env python3
"""Train the small Formula 110 neural controller with CMA-ES."""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any, cast

import cma  # pyright: ignore[reportMissingImports]

from controllers.cmaes_policy import PARAMETER_COUNT, initial_parameters
from racing.experiments.neuroevolution import HeadlessPolicyEvaluator, TrialMetrics, aggregate_fitness

DEFAULT_TRAINING_SEEDS = (17, 83, 241)
DEFAULT_VALIDATION_SEEDS = (41, 137, 311, 509, 887)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generations", type=int, default=25)
    parser.add_argument("--population-size", type=int, default=20)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--training-seeds", type=_seed_list, default=DEFAULT_TRAINING_SEEDS)
    parser.add_argument("--validation-seeds", type=_seed_list, default=DEFAULT_VALIDATION_SEEDS)
    parser.add_argument("--validate-every", type=int, default=5)
    parser.add_argument("--optimizer-seed", type=int, default=110)
    parser.add_argument("--sigma", type=float, default=0.35)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/cmaes"))
    parser.add_argument(
        "--export-controller-weights",
        type=Path,
        default=None,
        help="Also write the final champion in controller artifact format.",
    )
    return parser.parse_args()


def _seed_list(value: str) -> tuple[int, ...]:
    try:
        seeds = tuple(int(item.strip()) for item in value.split(",") if item.strip())
    except ValueError as error:
        raise argparse.ArgumentTypeError("seeds must be comma-separated integers") from error
    if not seeds:
        raise argparse.ArgumentTypeError("at least one seed is required")
    return seeds


def _validate_args(args: argparse.Namespace) -> None:
    if args.generations < 1 or args.population_size < 2:
        raise ValueError("generations must be positive and population size must be at least two")
    if args.seconds <= 0.0 or args.sigma <= 0.0:
        raise ValueError("seconds and sigma must be positive")
    if args.validate_every < 1:
        raise ValueError("validate-every must be positive")
    if set(args.training_seeds) & set(args.validation_seeds):
        raise ValueError("training and validation seeds must be disjoint")


def _evaluate_parameters(
    evaluator: HeadlessPolicyEvaluator,
    parameters: Sequence[float],
    *,
    seeds: Sequence[int],
    seconds: float,
) -> tuple[tuple[TrialMetrics, ...], float]:
    trials = tuple(evaluator.evaluate(parameters, seed=seed, seconds=seconds) for seed in seeds)
    return trials, aggregate_fitness(trials).fitness


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _append_jsonl(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, sort_keys=True) + "\n")


def _parameter_checksum(parameters: Sequence[float]) -> str:
    packed = ",".join(f"{float(value):.17g}" for value in parameters).encode()
    return hashlib.sha256(packed).hexdigest()[:16]


def _artifact_payload(parameters: Sequence[float], metadata: dict[str, object]) -> dict[str, object]:
    return {
        "schema_version": 1,
        "architecture": "12-8-2",
        "parameter_count": PARAMETER_COUNT,
        "parameters": [float(value) for value in parameters],
        "metadata": metadata,
    }


def main() -> None:
    args = parse_args()
    _validate_args(args)
    output_dir = cast(Path, args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = output_dir / "metrics.jsonl"
    if metrics_path.exists():
        metrics_path.unlink()

    options: dict[str, Any] = {
        "seed": int(args.optimizer_seed),
        "popsize": int(args.population_size),
        "bounds": [-5.0, 5.0],
        "verbose": -9,
    }
    cma_module = cast(Any, cma)
    strategy: Any = cma_module.CMAEvolutionStrategy(list(initial_parameters()), float(args.sigma), options)
    best_parameters: list[float] | None = None
    best_fitness = float("-inf")
    best_generation = 0
    started = time.monotonic()

    configuration = {
        "generations": args.generations,
        "population_size": args.population_size,
        "seconds": args.seconds,
        "training_seeds": list(args.training_seeds),
        "validation_seeds": list(args.validation_seeds),
        "validate_every": args.validate_every,
        "optimizer_seed": args.optimizer_seed,
        "sigma": args.sigma,
        "parameter_count": PARAMETER_COUNT,
    }
    _write_json(output_dir / "configuration.json", configuration)

    with HeadlessPolicyEvaluator() as evaluator:
        for generation in range(1, int(args.generations) + 1):
            population = cast(list[Sequence[float]], strategy.ask())
            objective_values: list[float] = []
            generation_records: list[dict[str, object]] = []
            for individual_index, candidate in enumerate(population):
                parameters = [float(value) for value in candidate]
                trials, fitness = _evaluate_parameters(
                    evaluator,
                    parameters,
                    seeds=args.training_seeds,
                    seconds=float(args.seconds),
                )
                objective_values.append(-fitness)  # pycma minimizes.
                record: dict[str, object] = {
                    "record_type": "individual",
                    "generation": generation,
                    "individual": individual_index,
                    "checksum": _parameter_checksum(parameters),
                    "fitness": fitness,
                    "trials": [trial.to_dict() for trial in trials],
                }
                generation_records.append(record)
                _append_jsonl(metrics_path, record)
                if fitness > best_fitness:
                    best_fitness = fitness
                    best_parameters = parameters
                    best_generation = generation

            strategy.tell(population, objective_values)
            champion = max(generation_records, key=lambda record: cast(float, record["fitness"]))
            champion_fitness = cast(float, champion["fitness"])
            summary: dict[str, object] = {
                "record_type": "generation",
                "generation": generation,
                "best_fitness": champion["fitness"],
                "mean_fitness": sum(-value for value in objective_values) / len(objective_values),
                "worst_fitness": min(-value for value in objective_values),
                "sigma": float(strategy.sigma),
                "elapsed_wall_seconds": time.monotonic() - started,
            }
            if generation % int(args.validate_every) == 0 and best_parameters is not None:
                validation_trials, validation_fitness = _evaluate_parameters(
                    evaluator,
                    best_parameters,
                    seeds=args.validation_seeds,
                    seconds=float(args.seconds),
                )
                summary["validation_fitness"] = validation_fitness
                summary["validation_trials"] = [trial.to_dict() for trial in validation_trials]
            _append_jsonl(metrics_path, summary)
            print(
                f"generation {generation:03d} best={champion_fitness:.4f} "
                f"mean={cast(float, summary['mean_fitness']):.4f} sigma={float(strategy.sigma):.4f}",
                flush=True,
            )
            if best_parameters is not None:
                _write_json(
                    output_dir / "best_weights.json",
                    _artifact_payload(
                        best_parameters,
                        {
                            **configuration,
                            "champion_generation": best_generation,
                            "training_fitness": best_fitness,
                        },
                    ),
                )
            if strategy.stop():
                break

    if best_parameters is None:
        raise RuntimeError("CMA-ES produced no candidates")
    if args.export_controller_weights is not None:
        _write_json(
            cast(Path, args.export_controller_weights),
            _artifact_payload(
                best_parameters,
                {
                    **configuration,
                    "champion_generation": best_generation,
                    "training_fitness": best_fitness,
                },
            ),
        )


if __name__ == "__main__":
    main()
