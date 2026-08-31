# CMA-ES neuroevolution experiment

This experiment evolves every parameter of a fixed `12 -> 8 -> 2` neural
controller. The deployed policy has 122 parameters and uses pure Python
inference; NumPy and `cma` are training-only development dependencies.

## Observation vector

In order, the normalized inputs are signed speed, yaw rate, center offset,
heading error, three camera lookahead offsets, and wall-only LiDAR at
`-90, -20, 0, 20, 90` degrees. Fixed caps and normalization constants live in
`controllers.cmaes_policy`. Infinite LiDAR readings are replaced by the finite
LiDAR cap.

Absolute heading, tick, odometry distance, pitch, roll, competitors, and contact
state are intentionally excluded. They are redundant for the first single-car
experiment or risk track/time memorization. Contact, damage, and official
progress remain external fitness signals rather than policy observations.

## Minimum experiment

First run a cheap pipeline smoke test:

```bash
uv run python scripts/train_cmaes.py \
  --generations 3 --population-size 8 --seconds 10
```

Then run the planned minimum viable experiment:

```bash
uv run python scripts/train_cmaes.py
```

Defaults are 20 individuals, 25 generations, three deterministic training
seeds, 20 seconds per trial, and five disjoint validation seeds evaluated every
five generations. The known Gradescope seeds 110 and 2026 are deliberately not
used for optimization.

Raw individual, trial, generation, and validation measurements are written to
`artifacts/cmaes/metrics.jsonl`. Configuration and the current champion are
written alongside them. To install a completed champion for watched and grading
runs, use:

```bash
uv run python scripts/train_cmaes.py \
  --export-controller-weights src/controllers/cmaes_weights.json

uv run racing --seed 110 --student-module controllers.cmaes_neuroevolution
```

The export path is relative to the controller module, so it remains valid when
the complete controller directory is packaged.

## Fitness

Per-seed fitness primarily rewards forward partial laps, with a modest bonus
for completing a lap early. It penalizes normalized off-track time, wall-contact
time, damage, and elimination. Multi-seed fitness is:

```text
0.60 * minimum seed score
+ 0.40 * mean seed score
- 0.25 * population standard deviation across seed scores
```

This makes a catastrophic starting position more important than unusually good
performance on one favorable position. Raw metrics are retained separately so
the scalar fitness can be audited.
