#!/usr/bin/env python3
"""Optimize reactive refinements around fixed CMA-ES policies, with hard safety selection."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean

import cma

from controllers.cmaes_speed_hybrid import SpeedHybridController
from racing import RobotCommand, RobotSensors
from racing.experiments.neuroevolution import HeadlessPolicyEvaluator

TRAIN_SEEDS = (17, 83, 241, 509)
DEVELOPMENT_SEEDS = (41, 137, 311, 887, 613, 719)


class InstrumentedPolicy:
    def __init__(self, parameters: list[float]) -> None:
        self.policy = SpeedHybridController(parameters)
        self.start_speeds: list[float] = []
        self.corner_speeds: list[float] = []
        self.straight_speeds: list[float] = []
        self.offsets: list[float] = []
        self.corner_throttles: list[float] = []
        self.time_to_10: float | None = None

    def __call__(self, sensors: RobotSensors) -> RobotCommand:
        command = self.policy(sensors)
        time = sensors.tick * sensors.dt_s
        speed = sensors.odometry.speed_mps
        if time < 3.0:
            self.start_speeds.append(speed)
        if self.time_to_10 is None and speed >= 10:
            self.time_to_10 = time
        if time >= 5.0:
            self.offsets.append(abs(sensors.camera.center_offset_m))
            if abs(sensors.imu.yaw_rate_degrees_per_s) >= 20:
                self.corner_speeds.append(speed)
                self.corner_throttles.append(command.throttle)
            else:
                self.straight_speeds.append(speed)
        return command


def evaluate(
    evaluator: HeadlessPolicyEvaluator, parameters: list[float], seeds: tuple[int, ...], seconds: float = 30.0
) -> dict:
    trials = []
    for seed in seeds:
        policy = InstrumentedPolicy(parameters)
        trial = evaluator.evaluate_controller(policy, seed=seed, seconds=seconds).to_dict()
        trial.update(
            start_mean_speed=mean(policy.start_speeds or [0]),
            corner_mean_speed=mean(policy.corner_speeds or [0]),
            straight_mean_speed=mean(policy.straight_speeds or [0]),
            mean_absolute_center_offset=mean(policy.offsets or [0]),
            corner_mean_throttle=mean(policy.corner_throttles or [0]),
            time_to_10=policy.time_to_10,
        )
        trials.append(trial)
    safe = all(
        t["damage"] == 0 and t["wall_contact_seconds"] == 0 and not t["eliminated"] and t["survived"] for t in trials
    )
    distances = [t["raw_distance_m"] for t in trials]
    score = 0.6 * min(distances) + 0.4 * mean(distances)
    score += 2.0 * mean(t["start_mean_speed"] for t in trials)
    score += 0.5 * mean(t["corner_mean_speed"] for t in trials)
    if not safe:
        score = -1000 - sum(100 * t["damage"] + 10 * t["wall_contact_seconds"] + 1000 * t["eliminated"] for t in trials)
    return dict(parameters=parameters, safe=safe, score=score, mean_distance=mean(distances), trials=trials)


def write(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generations", type=int, default=12)
    parser.add_argument("--population", type=int, default=10)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/cmaes-speed-hybrid"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    write(
        args.output_dir / "configuration.json",
        dict(
            training_seeds=TRAIN_SEEDS,
            development_seeds=DEVELOPMENT_SEEDS,
            generations=args.generations,
            population=args.population,
            seconds=30,
            optimizer_seed=20260908,
            sigma=0.35,
            parameter_names=["launch", "cruise", "corner", "turn", "center", "yaw", "shield", "launch_limit"],
            bounds=[[-1, -1, -1, -2, -2, -2, -2, -2], [2, 2, 2, 2, 2, 2, 2, 2]],
            corner_definition="absolute yaw rate >= 20 degrees/s after first 5 seconds",
            safety="any damage, wall contact, elimination or failure to survive is infeasible",
        ),
    )
    strategy = cma.CMAEvolutionStrategy(
        [0.7, 0, 0, 0, 0, 0, 0, 0],
        0.35,
        {
            "popsize": args.population,
            "seed": 20260908,
            "verbose": -9,
            "bounds": [[-1, -1, -1, -2, -2, -2, -2, -2], [2] * 8],
        },
    )
    with HeadlessPolicyEvaluator() as evaluator, (args.output_dir / "metrics.jsonl").open("w") as log:
        baseline = evaluate(evaluator, [0.0] * 8, TRAIN_SEEDS)
        write(args.output_dir / "baseline-training.json", baseline)
        candidates = [baseline]
        for generation in range(args.generations):
            population = strategy.ask()
            records = []
            for candidate in population:
                result = evaluate(evaluator, list(map(float, candidate)), TRAIN_SEEDS)
                result["generation"] = generation + 1
                log.write(json.dumps(result) + "\n")
                log.flush()
                records.append(result)
                if result["safe"]:
                    candidates.append(result)
            strategy.tell(population, [-r["score"] for r in records])
            best = max(candidates, key=lambda r: r["score"])
            write(args.output_dir / "best-training.json", best)
            print(
                f"generation {generation + 1}: safe={sum(r['safe'] for r in records)}/{len(records)} "
                f"best_distance={best['mean_distance']:.2f} score={best['score']:.2f}",
                flush=True,
            )
        ranked = sorted(candidates, key=lambda r: r["score"], reverse=True)[:8]
        development_baseline = evaluate(evaluator, [0.0] * 8, DEVELOPMENT_SEEDS)
        write(args.output_dir / "baseline-development.json", development_baseline)
        finalists = [development_baseline]
        for candidate in ranked:
            result = evaluate(evaluator, candidate["parameters"], DEVELOPMENT_SEEDS)
            finalists.append(result)
            print(f"development: safe={result['safe']} distance={result['mean_distance']:.2f}", flush=True)
        write(args.output_dir / "development.json", finalists)
        selected = max((r for r in finalists if r["safe"]), key=lambda r: r["score"])
        write(args.output_dir / "selected.json", selected)
        print(f"selected: {selected['parameters']} distance={selected['mean_distance']:.2f}", flush=True)


if __name__ == "__main__":
    main()
