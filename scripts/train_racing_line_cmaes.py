#!/usr/bin/env python3
"""Run planner-only or joint CMA-ES tuning for the structured racing line."""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import pickle
import time
import warnings
from collections.abc import Callable, Mapping, Sequence
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, astuple, dataclass
from pathlib import Path
from statistics import mean
from typing import Any, cast

from controllers.centerline import CenterlineController, CenterlineParameters
from controllers.centerline.braking_tuned_parameters import BRAKING_TUNED_PARAMETERS
from controllers.centerline.parameters import PARAMETER_NAMES
from controllers.centerline_v4 import create_controller as create_centerline_v4
from controllers.racing_line import AUTHORED_RACING_LINE_PARAMETERS, RacingLineParameters, RacingLinePlanner
from controllers.racing_line.combined_parameters import (
    JOINT_PARAMETER_BOUNDS,
    JOINT_PARAMETER_COUNT,
    JOINT_PARAMETER_NAMES,
    JointParameters,
    joint_from_normalized,
    joint_to_normalized,
)
from controllers.racing_line.parameters import (
    RACING_LINE_PARAMETER_BOUNDS,
    RACING_LINE_PARAMETER_COUNT,
    RACING_LINE_PARAMETER_NAMES,
    racing_line_from_normalized,
    racing_line_to_normalized,
)
from controllers.racing_line.planner_tuned_parameters import PLANNER_TUNED_PARAMETERS
from controllers.racing_line_v1 import create_controller as create_racing_line_v1
from controllers.racing_line_v2 import create_controller as create_racing_line_v2
from racing.experiments.centerline_braking import DiagnosedTrial, evaluate_diagnosed
from racing.experiments.neuroevolution import HeadlessPolicyEvaluator
from racing.experiments.racing_line import RacingLineFitness, racing_line_fitness, racing_line_promotion

with warnings.catch_warnings():
    warnings.filterwarnings("ignore", message="Could not import matplotlib.pyplot.*", module="cma.s")
    cma = importlib.import_module("cma")

TRAINING_SEEDS = (19, 53, 97, 149, 269, 401, 587)
DEVELOPMENT_SEEDS = (631, 743, 857, 947, 1063)
PROMOTION_SEEDS = (1171, 1289, 1423, 1559, 1693)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", required=True, choices=("planner", "joint"))
    parser.add_argument("--generations", type=int)
    parser.add_argument("--population-size", type=int)
    parser.add_argument("--sigma", type=float)
    parser.add_argument("--optimizer-seed", type=int)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--training-seeds", type=_seed_list, default=TRAINING_SEEDS)
    parser.add_argument("--development-seeds", type=_seed_list, default=DEVELOPMENT_SEEDS)
    parser.add_argument("--promotion-seeds", type=_seed_list, default=PROMOTION_SEEDS)
    parser.add_argument("--validate-every", type=int, default=5)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--output-root", type=Path, default=Path("artifacts/racing-line-cmaes"))
    parser.add_argument("--resume", action="store_true", help="resume a complete saved generation")
    return parser.parse_args()


def _seed_list(value: str) -> tuple[int, ...]:
    try:
        result = tuple(int(item.strip()) for item in value.split(",") if item.strip())
    except ValueError as error:
        raise argparse.ArgumentTypeError("seeds must be comma-separated integers") from error
    if not result:
        raise argparse.ArgumentTypeError("at least one seed is required")
    return result


def _defaults(args: argparse.Namespace) -> None:
    planner = args.stage == "planner"
    args.generations = args.generations if args.generations is not None else (15 if planner else 20)
    args.population_size = args.population_size if args.population_size is not None else (16 if planner else 24)
    args.sigma = args.sigma if args.sigma is not None else (0.25 if planner else 0.20)
    args.optimizer_seed = (
        args.optimizer_seed if args.optimizer_seed is not None else (2026090701 if planner else 2026090702)
    )
    if (
        min(args.generations, args.population_size, args.validate_every, args.workers) < 1
        or args.seconds <= 0.0
        or args.sigma <= 0.0
    ):
        raise ValueError("generation, population, validation interval, seconds, and sigma must be positive")
    seed_sets = (set(args.training_seeds), set(args.development_seeds), set(args.promotion_seeds))
    if any(left & right for index, left in enumerate(seed_sets) for right in seed_sets[index + 1 :]):
        raise ValueError("training, development, and promotion seeds must be disjoint")


def make_controller(planner: RacingLineParameters, expert: CenterlineParameters) -> CenterlineController:
    return CenterlineController(expert, forward_only_braking=True, line_planner=RacingLinePlanner(planner))


def evaluate_factory(
    evaluator: HeadlessPolicyEvaluator,
    factory: Callable[[], CenterlineController],
    *,
    seeds: Sequence[int],
    seconds: float,
) -> tuple[DiagnosedTrial, ...]:
    return tuple(evaluate_diagnosed(evaluator, factory(), seed=seed, seconds=seconds) for seed in seeds)


@dataclass(frozen=True, slots=True)
class CandidateRequest:
    planner_stage: bool
    normalized: tuple[float, ...]
    initial_expert: CenterlineParameters
    seeds: tuple[int, ...]
    seconds: float


@dataclass(frozen=True, slots=True)
class CandidateEvaluation:
    planner: RacingLineParameters
    expert: CenterlineParameters
    trials: tuple[DiagnosedTrial, ...]
    fitness: RacingLineFitness


def evaluate_candidate(request: CandidateRequest) -> CandidateEvaluation:
    if request.planner_stage:
        planner = racing_line_from_normalized(list(request.normalized))
        expert = request.initial_expert
    else:
        joint = joint_from_normalized(list(request.normalized))
        planner, expert = joint.planner, joint.expert
    with HeadlessPolicyEvaluator() as evaluator:
        trials = evaluate_factory(
            evaluator,
            lambda: make_controller(planner, expert),
            seeds=request.seeds,
            seconds=request.seconds,
        )
    return CandidateEvaluation(planner, expert, trials, racing_line_fitness(trials))


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def _append_jsonl(path: Path, payload: object) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, sort_keys=True) + "\n")


def _checksum(values: Sequence[float]) -> str:
    return hashlib.sha256(",".join(f"{float(value):.17g}" for value in values).encode()).hexdigest()[:16]


def _parameter_payload(
    planner: RacingLineParameters,
    expert: CenterlineParameters,
    normalized: Sequence[float],
    metadata: Mapping[str, object],
) -> dict[str, object]:
    return {
        "schema_version": 1,
        "planner_parameters": dict(zip(RACING_LINE_PARAMETER_NAMES, astuple(planner), strict=True)),
        "expert_parameters": dict(zip(PARAMETER_NAMES, astuple(expert), strict=True)),
        "normalized": list(normalized),
        "metadata": metadata,
    }


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object in {path}")
    return cast(dict[str, Any], value)


def _parameters_from_payload(
    payload: Mapping[str, Any],
) -> tuple[RacingLineParameters, CenterlineParameters, tuple[float, ...]]:
    planner_values = cast(Mapping[str, float], payload["planner_parameters"])
    expert_values = cast(Mapping[str, float], payload["expert_parameters"])
    normalized = tuple(float(value) for value in cast(Sequence[float], payload["normalized"]))
    planner = RacingLineParameters(**{name: float(planner_values[name]) for name in RACING_LINE_PARAMETER_NAMES})
    expert = CenterlineParameters(**{name: float(expert_values[name]) for name in PARAMETER_NAMES})
    return planner, expert, normalized


def _completed_generations(metrics_path: Path) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for line in metrics_path.read_text(encoding="utf-8").splitlines():
        value = json.loads(line)
        if isinstance(value, dict):
            record = cast(dict[str, Any], value)
            if record.get("record_type") == "generation":
                summaries.append(record)
    return summaries


def _assert_resume_configuration(saved: Mapping[str, Any], expected: Mapping[str, object]) -> None:
    mismatches = [key for key, value in expected.items() if saved.get(key) != value]
    if mismatches:
        raise ValueError(f"resume configuration mismatch: {', '.join(mismatches)}")


def _write_optimizer(path: Path, strategy: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        pickle.dump(strategy, stream)
    temporary.replace(path)


def main() -> None:
    args = parse_args()
    _defaults(args)
    planner_stage = args.stage == "planner"
    output_dir = cast(Path, args.output_root) / ("planner-only" if planner_stage else "joint")
    output_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = output_dir / "metrics.jsonl"
    if metrics_path.exists() and not args.resume:
        metrics_path.unlink()
    initial_planner = AUTHORED_RACING_LINE_PARAMETERS if planner_stage else PLANNER_TUNED_PARAMETERS
    initial_expert = BRAKING_TUNED_PARAMETERS
    if planner_stage:
        initial_vector = racing_line_to_normalized(initial_planner)
        names, bounds = RACING_LINE_PARAMETER_NAMES, RACING_LINE_PARAMETER_BOUNDS
    else:
        initial_vector = joint_to_normalized(JointParameters(initial_planner, initial_expert))
        names, bounds = JOINT_PARAMETER_NAMES, JOINT_PARAMETER_BOUNDS
    configuration: dict[str, object] = {
        "stage": args.stage,
        "generations": args.generations,
        "population_size": args.population_size,
        "sigma": args.sigma,
        "optimizer_seed": args.optimizer_seed,
        "seconds": args.seconds,
        "validate_every": args.validate_every,
        "workers": args.workers,
        "training_seeds": list(args.training_seeds),
        "development_seeds": list(args.development_seeds),
        "promotion_seeds": list(args.promotion_seeds),
        "parameter_count": RACING_LINE_PARAMETER_COUNT if planner_stage else JOINT_PARAMETER_COUNT,
        "parameter_names": list(names),
        "parameter_bounds": [list(bound) for bound in bounds],
    }
    optimizer_path = output_dir / "optimizer.pkl"
    best_path = output_dir / "best_training_candidate.json"
    start_generation = 1
    prior_elapsed_seconds = 0.0
    restored_development_pass = False
    if args.resume:
        required = (output_dir / "configuration.json", optimizer_path, best_path, metrics_path)
        missing = [str(path) for path in required if not path.exists()]
        if missing:
            raise FileNotFoundError(f"resume files are missing: {', '.join(missing)}")
        _assert_resume_configuration(_read_json(output_dir / "configuration.json"), configuration)
        summaries = _completed_generations(metrics_path)
        if not summaries:
            raise ValueError("resume metrics contain no complete generations")
        last_summary = summaries[-1]
        last_generation = int(last_summary["generation"])
        with optimizer_path.open("rb") as stream:
            strategy = pickle.load(stream)
        if int(strategy.countiter) != last_generation:
            raise ValueError(f"optimizer iteration {strategy.countiter} does not match generation {last_generation}")
        best_payload = _read_json(best_path)
        best_planner, best_expert, best_vector = _parameters_from_payload(best_payload)
        best_metadata = cast(Mapping[str, Any], best_payload["metadata"])
        best_fitness = float(best_metadata["training_fitness"])
        best_generation = int(best_metadata["champion_generation"])
        passing_generations = [
            int(summary["generation"]) for summary in summaries if summary.get("development_passed") is True
        ]
        restored_development_pass = bool(passing_generations and best_generation <= max(passing_generations))
        start_generation = last_generation + 1
        prior_elapsed_seconds = float(last_summary.get("elapsed_wall_seconds", 0.0))
        _append_jsonl(
            metrics_path,
            {
                "record_type": "resume",
                "completed_generation": last_generation,
                "next_generation": start_generation,
                "champion_generation": best_generation,
                "champion_checksum": _checksum(best_vector),
                "development_candidate_restored": restored_development_pass,
            },
        )
    else:
        _write_json(output_dir / "configuration.json", configuration)
        strategy = cast(Any, cma).CMAEvolutionStrategy(
            list(initial_vector),
            float(args.sigma),
            {
                "seed": int(args.optimizer_seed),
                "popsize": int(args.population_size),
                "bounds": [-1.0, 1.0],
                "verbose": -9,
            },
        )
        best_vector = tuple(initial_vector)
        best_planner, best_expert = initial_planner, initial_expert
        best_fitness = float("-inf")
        best_generation = 0
    development_candidate: tuple[RacingLineParameters, CenterlineParameters, tuple[float, ...]] | None = None
    if restored_development_pass:
        development_candidate = best_planner, best_expert, best_vector
    started = time.monotonic()
    with HeadlessPolicyEvaluator() as evaluator:
        baseline_development = evaluate_factory(
            evaluator, create_centerline_v4, seeds=args.development_seeds, seconds=float(args.seconds)
        )
        incumbent_factory = create_racing_line_v1 if planner_stage else create_racing_line_v2
        incumbent_development = evaluate_factory(
            evaluator, incumbent_factory, seeds=args.development_seeds, seconds=float(args.seconds)
        )
        _write_json(output_dir / "centerline_v4_development.json", [trial.to_dict() for trial in baseline_development])
        _write_json(output_dir / "incumbent_development.json", [trial.to_dict() for trial in incumbent_development])
        if not args.resume:
            initial_trials = evaluate_factory(
                evaluator,
                lambda: make_controller(initial_planner, initial_expert),
                seeds=args.training_seeds,
                seconds=float(args.seconds),
            )
            initial_fitness = racing_line_fitness(initial_trials)
            best_fitness = initial_fitness.fitness
            _append_jsonl(
                metrics_path,
                {
                    "record_type": "initial_mean",
                    "fitness": initial_fitness.fitness,
                    "fitness_components": asdict(initial_fitness),
                    "trials": [trial.to_dict() for trial in initial_trials],
                },
            )
        for generation in range(start_generation, int(args.generations) + 1):
            population = cast(list[Sequence[float]], strategy.ask())
            objectives: list[float] = []
            fitnesses: list[float] = []
            normalized_population = [tuple(float(value) for value in candidate) for candidate in population]
            requests = (
                CandidateRequest(
                    planner_stage,
                    normalized,
                    initial_expert,
                    tuple(args.training_seeds),
                    float(args.seconds),
                )
                for normalized in normalized_population
            )
            with ProcessPoolExecutor(max_workers=int(args.workers), max_tasks_per_child=1) as executor:
                evaluations = tuple(executor.map(evaluate_candidate, requests))
            for individual, (normalized, evaluation) in enumerate(zip(normalized_population, evaluations, strict=True)):
                planner, expert, trials, result = (
                    evaluation.planner,
                    evaluation.expert,
                    evaluation.trials,
                    evaluation.fitness,
                )
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
                        "normalized": list(normalized),
                        "planner_parameters": dict(zip(RACING_LINE_PARAMETER_NAMES, astuple(planner), strict=True)),
                        "expert_parameters": dict(zip(PARAMETER_NAMES, astuple(expert), strict=True)),
                        "fitness_components": asdict(result),
                        "trials": [trial.to_dict() for trial in trials],
                    },
                )
                if result.fitness > best_fitness:
                    best_vector, best_planner, best_expert = normalized, planner, expert
                    best_fitness, best_generation = result.fitness, generation
            strategy.tell(population, objectives)
            summary: dict[str, object] = {
                "record_type": "generation",
                "generation": generation,
                "best_fitness": max(fitnesses),
                "mean_fitness": mean(fitnesses),
                "worst_fitness": min(fitnesses),
                "sigma": float(strategy.sigma),
                "elapsed_wall_seconds": prior_elapsed_seconds + time.monotonic() - started,
            }
            if generation % int(args.validate_every) == 0:
                validation_planner, validation_expert = best_planner, best_expert
                validation = evaluate_factory(
                    evaluator,
                    lambda planner=validation_planner, expert=validation_expert: make_controller(planner, expert),
                    seeds=args.development_seeds,
                    seconds=float(args.seconds),
                )
                promotion = racing_line_promotion(validation, baseline_development, incumbent_development)
                summary.update(
                    development_trials=[trial.to_dict() for trial in validation],
                    development_passed=promotion.passed,
                    development_reasons=list(promotion.reasons),
                )
                if promotion.passed:
                    development_candidate = best_planner, best_expert, best_vector
                    checkpoint_metadata = {
                        **configuration,
                        "champion_generation": best_generation,
                        "training_fitness": best_fitness,
                        "development_status": "passed",
                    }
                    _write_json(
                        output_dir / "development_candidate.json",
                        _parameter_payload(best_planner, best_expert, best_vector, checkpoint_metadata),
                    )
            _append_jsonl(metrics_path, summary)
            metadata = {**configuration, "champion_generation": best_generation, "training_fitness": best_fitness}
            _write_json(
                output_dir / "best_training_candidate.json",
                _parameter_payload(best_planner, best_expert, best_vector, metadata),
            )
            _write_optimizer(optimizer_path, strategy)
            print(
                f"{args.stage} generation {generation:03d} best={max(fitnesses):.3f} "
                f"mean={mean(fitnesses):.3f} sigma={float(strategy.sigma):.4f}",
                flush=True,
            )
            if strategy.stop():
                break
        if development_candidate is None:
            print(f"{args.stage} development gate did not pass; retained incumbent")
            return
        promoted_planner, promoted_expert, promoted_vector = development_candidate
        metadata = {**configuration, "champion_generation": best_generation, "training_fitness": best_fitness}
        _write_json(
            output_dir / "development_candidate.json",
            _parameter_payload(
                promoted_planner, promoted_expert, promoted_vector, {**metadata, "development_status": "passed"}
            ),
        )
        if planner_stage:
            _write_json(
                output_dir / "promoted_candidate.json",
                _parameter_payload(
                    promoted_planner, promoted_expert, promoted_vector, {**metadata, "promotion_status": "passed"}
                ),
            )
            print("planner development gate passed; candidate promoted")
            return
        baseline_promotion = evaluate_factory(
            evaluator, create_centerline_v4, seeds=args.promotion_seeds, seconds=float(args.seconds)
        )
        incumbent_promotion = evaluate_factory(
            evaluator, create_racing_line_v2, seeds=args.promotion_seeds, seconds=float(args.seconds)
        )
        candidate_promotion = evaluate_factory(
            evaluator,
            lambda: make_controller(promoted_planner, promoted_expert),
            seeds=args.promotion_seeds,
            seconds=float(args.seconds),
        )
        promotion = racing_line_promotion(candidate_promotion, baseline_promotion, incumbent_promotion)
        _write_json(
            output_dir / "untouched_promotion.json",
            {
                "passed": promotion.passed,
                "reasons": list(promotion.reasons),
                "centerline_v4": [trial.to_dict() for trial in baseline_promotion],
                "incumbent": [trial.to_dict() for trial in incumbent_promotion],
                "candidate": [trial.to_dict() for trial in candidate_promotion],
            },
        )
        if promotion.passed:
            _write_json(
                output_dir / "promoted_candidate.json",
                _parameter_payload(
                    promoted_planner,
                    promoted_expert,
                    promoted_vector,
                    {**metadata, "promotion_status": "untouched-passed"},
                ),
            )
            print("joint untouched promotion gate passed; candidate promoted")
        else:
            print(f"joint untouched promotion failed: {promotion.reasons}")


if __name__ == "__main__":
    main()
