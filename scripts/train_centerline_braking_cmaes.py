#!/usr/bin/env python3
"""Retune the centerline expert with forward-only braking semantics."""

from __future__ import annotations

import argparse
import hashlib
import json
import pickle
import time
from collections.abc import Callable, Sequence
from dataclasses import asdict
from pathlib import Path
from statistics import mean
from typing import Any, cast

import cma  # pyright: ignore[reportMissingImports]

from controllers.centerline import CenterlineController, CenterlineParameters
from controllers.centerline.parameters import (
    PARAMETER_BOUNDS,
    PARAMETER_COUNT,
    PARAMETER_NAMES,
    from_normalized,
    to_normalized,
)
from controllers.centerline.tuned_parameters import TUNED_PARAMETERS
from controllers.centerline_v2 import create_controller as create_centerline_v2
from racing.experiments.centerline_braking import (
    DiagnosedTrial,
    braking_fitness,
    braking_promotion_result,
    evaluate_diagnosed,
)
from racing.experiments.neuroevolution import HeadlessPolicyEvaluator

TRAINING_SEEDS = (17, 41, 83, 137, 241, 311, 509)
VALIDATION_SEEDS = (613, 719, 823, 929, 1031)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generations", type=int, default=20)
    parser.add_argument("--population-size", type=int, default=16)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--training-seeds", type=_seed_list, default=TRAINING_SEEDS)
    parser.add_argument("--validation-seeds", type=_seed_list, default=VALIDATION_SEEDS)
    parser.add_argument("--validate-every", type=int, default=5)
    parser.add_argument("--optimizer-seed", type=int, default=20260906)
    parser.add_argument("--sigma", type=float, default=0.25)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/centerline-braking-cmaes"))
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
    if args.seconds <= 0.0 or args.sigma <= 0.0 or args.validate_every < 1:
        raise ValueError("seconds, sigma, and validate-every must be positive")
    if set(args.training_seeds) & set(args.validation_seeds):
        raise ValueError("training and validation seeds must be disjoint")


def corrected_controller(parameters: CenterlineParameters) -> CenterlineController:
    return CenterlineController(parameters, forward_only_braking=True)


def evaluate_factory(
    evaluator: HeadlessPolicyEvaluator,
    factory: Callable[[], CenterlineController],
    *,
    seeds: Sequence[int],
    seconds: float,
) -> tuple[DiagnosedTrial, ...]:
    return tuple(evaluate_diagnosed(evaluator, factory(), seed=seed, seconds=seconds) for seed in seeds)


def _values(parameters: CenterlineParameters) -> tuple[float, ...]:
    return tuple(float(getattr(parameters, name)) for name in PARAMETER_NAMES)


def _payload(parameters: CenterlineParameters, metadata: dict[str, object]) -> dict[str, object]:
    return {
        "schema_version": 2,
        "controller": "centerline_v4",
        "parameter_count": PARAMETER_COUNT,
        "parameters": dict(zip(PARAMETER_NAMES, _values(parameters), strict=True)),
        "normalized": list(to_normalized(parameters)),
        "bounds": {name: list(bounds) for name, bounds in zip(PARAMETER_NAMES, PARAMETER_BOUNDS, strict=True)},
        "metadata": metadata,
    }


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _append_jsonl(path: Path, payload: object) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, sort_keys=True) + "\n")


def _checksum(values: Sequence[float]) -> str:
    packed = ",".join(f"{float(value):.17g}" for value in values).encode()
    return hashlib.sha256(packed).hexdigest()[:16]


def main() -> None:
    args = parse_args()
    _validate_args(args)
    output_dir = cast(Path, args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = output_dir / "metrics.jsonl"
    if metrics_path.exists():
        metrics_path.unlink()
    configuration = {
        "generations": args.generations,
        "population_size": args.population_size,
        "seconds": args.seconds,
        "training_seeds": list(args.training_seeds),
        "validation_seeds": list(args.validation_seeds),
        "validate_every": args.validate_every,
        "optimizer_seed": args.optimizer_seed,
        "sigma": args.sigma,
        "parameter_names": list(PARAMETER_NAMES),
        "parameter_bounds": [list(bounds) for bounds in PARAMETER_BOUNDS],
        "brake_cutoff_mps": 0.5,
    }
    _write_json(output_dir / "configuration.json", configuration)
    strategy = cast(Any, cma).CMAEvolutionStrategy(
        list(to_normalized(TUNED_PARAMETERS)),
        float(args.sigma),
        {"seed": int(args.optimizer_seed), "popsize": int(args.population_size), "bounds": [-1.0, 1.0], "verbose": -9},
    )
    best_parameters = TUNED_PARAMETERS
    best_fitness = float("-inf")
    best_generation = 0
    best_promoted: CenterlineParameters | None = None
    started = time.monotonic()
    with HeadlessPolicyEvaluator() as evaluator:
        baseline_validation = evaluate_factory(
            evaluator, create_centerline_v2, seeds=args.validation_seeds, seconds=float(args.seconds)
        )
        v3_training = evaluate_factory(
            evaluator,
            lambda: corrected_controller(TUNED_PARAMETERS),
            seeds=args.training_seeds,
            seconds=float(args.seconds),
        )
        v3_validation = evaluate_factory(
            evaluator,
            lambda: corrected_controller(TUNED_PARAMETERS),
            seeds=args.validation_seeds,
            seconds=float(args.seconds),
        )
        initial = braking_fitness(v3_training)
        best_fitness = initial.fitness
        _write_json(output_dir / "centerline_v2_validation.json", [trial.to_dict() for trial in baseline_validation])
        _write_json(output_dir / "centerline_v3_training.json", [trial.to_dict() for trial in v3_training])
        _write_json(output_dir / "centerline_v3_validation.json", [trial.to_dict() for trial in v3_validation])
        _append_jsonl(
            metrics_path,
            {
                "record_type": "initial_mean",
                "fitness": initial.fitness,
                "fitness_components": asdict(initial),
                "parameters": dict(zip(PARAMETER_NAMES, _values(TUNED_PARAMETERS), strict=True)),
                "trials": [trial.to_dict() for trial in v3_training],
            },
        )
        for generation in range(1, int(args.generations) + 1):
            population = cast(list[Sequence[float]], strategy.ask())
            objectives: list[float] = []
            fitnesses: list[float] = []
            for individual, candidate in enumerate(population):
                normalized = [float(value) for value in candidate]
                parameters = from_normalized(normalized)
                trials = evaluate_factory(
                    evaluator,
                    lambda parameters=parameters: corrected_controller(parameters),
                    seeds=args.training_seeds,
                    seconds=float(args.seconds),
                )
                result = braking_fitness(trials)
                objectives.append(-result.fitness)
                fitnesses.append(result.fitness)
                _append_jsonl(
                    metrics_path,
                    {
                        "record_type": "individual",
                        "generation": generation,
                        "individual": individual,
                        "checksum": _checksum(normalized),
                        "fitness": result.fitness,
                        "normalized": normalized,
                        "parameters": dict(zip(PARAMETER_NAMES, _values(parameters), strict=True)),
                        "fitness_components": asdict(result),
                        "trials": [trial.to_dict() for trial in trials],
                    },
                )
                if result.fitness > best_fitness:
                    best_parameters, best_fitness, best_generation = parameters, result.fitness, generation
            strategy.tell(population, objectives)
            summary: dict[str, object] = {
                "record_type": "generation",
                "generation": generation,
                "best_fitness": max(fitnesses),
                "mean_fitness": mean(fitnesses),
                "worst_fitness": min(fitnesses),
                "sigma": float(strategy.sigma),
                "elapsed_wall_seconds": time.monotonic() - started,
            }
            if generation % int(args.validate_every) == 0:
                validation_parameters = best_parameters
                validation = evaluate_factory(
                    evaluator,
                    lambda parameters=validation_parameters: corrected_controller(parameters),
                    seeds=args.validation_seeds,
                    seconds=float(args.seconds),
                )
                passed, reasons = braking_promotion_result(validation, baseline_validation)
                summary.update(
                    validation_trials=[trial.to_dict() for trial in validation],
                    promotion_passed=passed,
                    promotion_reasons=list(reasons),
                )
                if passed:
                    best_promoted = best_parameters
            _append_jsonl(metrics_path, summary)
            _write_json(
                output_dir / "best_training_candidate.json",
                _payload(
                    best_parameters,
                    {
                        **configuration,
                        "champion_generation": best_generation,
                        "training_fitness": best_fitness,
                    },
                ),
            )
            with (output_dir / "optimizer.pkl").open("wb") as stream:
                pickle.dump(strategy, stream)
            print(
                f"generation {generation:03d} best={max(fitnesses):.3f} "
                f"mean={mean(fitnesses):.3f} sigma={float(strategy.sigma):.4f}",
                flush=True,
            )
            if strategy.stop():
                break
    if best_promoted is None:
        print("promotion gate did not pass; centerline_v3 remains retained")
    else:
        _write_json(
            output_dir / "promoted_candidate.json",
            _payload(
                best_promoted,
                {
                    **configuration,
                    "promotion_status": "passed",
                    "champion_generation": best_generation,
                    "training_fitness": best_fitness,
                },
            ),
        )
        print("promotion gate passed; candidate written to promoted_candidate.json")


if __name__ == "__main__":
    main()
