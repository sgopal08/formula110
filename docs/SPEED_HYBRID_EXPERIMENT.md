# CMA-ES speed hybrid experiment — September 8, 2026

Objective: improve acceleration from rest, sustained corner speed, and forward
progress while requiring exactly zero damage and wall contact in acceptance runs.

## Controller and search

The previous `cmaes_racing_line` combines two fixed, CMA-ES-evolved 122-parameter
neural policies. This experiment keeps those weights fixed and uses CMA-ES to
optimize eight reactive refinement parameters around them. This is a smaller,
more interpretable search than evolving both networks again at once.

The parameters control launch assistance, cruise throttle, corner throttle,
lookahead steering, center-offset correction, yaw damping, wall fallback distance,
and the speed at which launch assistance fades out. Zero parameters reproduce the
previous hybrid exactly. The controller has no fixed left/right preference and
adds no corner throttle cap. Actual speed still varies with steering and physics.

Launch assistance is proportional to `max(0, 1 - speed / launch_target)` so it
fades with speed, rather than depending on a memorized elapsed time or location.
The layer also observes actual speed above the neural policy's 12 m/s input cap;
it does not change the input normalization expected by the saved neural weights.
Positioning corrections use local camera geometry and yaw rate. No absolute
position, track map, official progress, or future simulator state enters inference.

Safety selection is a hard constraint: a candidate with any damage, wall contact,
elimination, or failure to survive receives an infeasible score. Among feasible
candidates, fitness rewards worst-seed and mean forward distance, initial speed,
and sustained corner speed. This prevents a faster damaging run from compensating
for a failure with a large progress reward. It is an empirical selection rule,
not a mathematical guarantee of safety on every possible state or opponent.

## Reproduction

```bash
.venv/bin/python scripts/train_speed_hybrid.py
.venv/bin/python scripts/validate_speed_hybrid.py
.venv/bin/python -m racing --seed 110 --student-module controllers.cmaes_speed_hybrid
```

The trainer refuses to overwrite an existing experiment directory; use
`--output-dir artifacts/NEW_NAME` and pass the same directory to the validator's
`--experiment` option for a new run. Candidate vectors, complete per-seed metrics,
configuration, development selection, validation and ablations are retained there.

Training uses 12 generations × 10 candidates × 4 seeds = 480 full 30-second
trials, initial sigma 0.35 and optimizer seed 20260908. Training seeds are
17, 83, 241, 509. The eight best feasible training candidates are compared on
six development seeds: 41, 137, 311, 887, 613, 719. These are selection data,
not untouched test data, and some were used to train the original neural policies.

The selected candidate is frozen before testing on 20 fresh seeds:
149, 281, 397, 563, 677, 809, 941, 1063, 1187, 1327, 1451, 1579, 1693, 1823,
1951, 2081, 2203, 2333, 2467, 2591. Official seeds 110 and 2026 are reported
separately. Five 120-second runs test behavior at four times the training horizon.
All evaluations use the same 60 Hz simulator and no marshal recovery.

Corner-speed diagnostics use samples with absolute yaw rate at least 20 degrees/s,
excluding the initial five seconds. Straight-speed diagnostics use the remaining
samples after five seconds. These are behavior-dependent groups, not matched
track segments, so forward distance and lap time remain the primary measures.
Initial speed averages the first three seconds. Center offset is absolute distance
from the centerline, averaged after five seconds. No claim of a globally optimal
racing line follows from this finite local search.

## Results and selection

The fastest feasible development candidate was rejected after the first 20-seed
suite: seed 563 had 0.0333 s wall contact, and seed 1693 had 0.0500 s contact plus
0.0002842 damage. That suite became screening data after this rejection.

The second-ranked feasible development candidate passed all 30 screening seeds
(the original 20 plus four training and six development seeds). It was then
frozen and tested on a new 20-seed suite, formed by adding 3000 to each original
final seed. It passed all 20 runs, both official runs, and five 120-second runs
with exactly zero damage, zero wall contact, and no elimination. These represent
52 distinct tested starting seeds overall, plus five longer runs. This evidence
covers solo driving on the simulator's existing track, not other track layouts
or contact with competitors.

The saved policy is `controllers.cmaes_speed_hybrid`, now selected in
`formula110-submission.json`. The previous controllers and neural artifacts are
preserved. The exported eight coefficients are in
`src/controllers/cmaes_speed_hybrid_weights.json`, including baseline artifact
hashes and acceptance metadata.

### Matched comparison on the 20 fresh seeds

| Metric | Previous racing-line hybrid | Selected speed hybrid | Change |
|---|---:|---:|---:|
| Mean 30-second distance | 476.53 m | 513.51 m | +7.76% |
| Mean time to 10 m/s | 3.126 s | 1.203 s | -61.50% |
| Mean speed during first 3 s | 5.81 m/s | 9.54 m/s | +64.1% |
| Mean corner speed | 15.97 m/s | 16.56 m/s | +3.70% |
| Mean straight speed | 15.77 m/s | 16.38 m/s | +3.90% |
| Mean best lap | 10.383 s | 10.080 s | -2.92% |
| Mean per-run maximum speed | 17.80 m/s | 18.17 m/s | +2.08% |
| Mean absolute center offset | 1.295 m | 1.369 m | +0.074 m |
| Mean off-track time per run | 0.397 s | 0.833 s | +0.436 s |
| Damage / wall contact | 0 / 0 | 0 / 0 | Maintained |

The selected controller travels farther on every one of the 20 fresh seeds.
The off-track increase is a real tradeoff: approximately 2.78% of simulated time,
versus 1.32% previously. Zero damage does not imply zero off-track time.
Official mean distance improves from 474.93 m to 514.96 m (+8.43%). Over
120 seconds, mean distance improves from 2051.39 m to 2142.07 m (+4.42%),
showing a sustained gain after the initial acceleration advantage becomes smaller.

### What positioning and launch contribute

On six development seeds, the complete hybrid averages 514.84 m with zero damage.
Removing launch assistance lowers this to 492.46 m and increases time to 10 m/s
from 1.21 s to 3.03 s. Both remain safe. Launch assistance therefore contributes
about 22.38 m over these 30-second runs.

Removing the lookahead, center-offset and yaw corrections while keeping the
extra throttle produces only 466.75 m, with total damage 0.3706 and 18.83 s wall
contact. This demonstrates that acceleration alone is insufficient; steering
refinement is needed to support the higher pace.

The successful line uses slightly more track width on average. It is not a
permanent left/right offset, and centering the car more closely is not sufficient
to make the faster throttle safe. These ablations test the corrections together;
they do not establish the optimal position at every point on the track.

### Reproduce the selected second-ranked candidate

After running the trainer, use a new output directory:

```bash
.venv/bin/python scripts/validate_speed_hybrid.py \
  --selection-source artifacts/cmaes-speed-hybrid \
  --candidate-rank 2 \
  --experiment artifacts/cmaes-speed-hybrid-safe-replay \
  --seed-offset 3000
```

The validator returns failure if the candidate violates safety or fails to
improve fresh-suite mean progress. The original experiment and rejected candidate
results remain under `artifacts/cmaes-speed-hybrid/`; selected-candidate screening,
acceptance, ablations, and detailed results are under
`artifacts/cmaes-speed-hybrid-safe/`.

### Checks

The repository suite passed with 129 tests. New controller and test files passed
strict Pyright checks, and new Python files passed Ruff lint/format checks.
Whole-repository Pyright additionally reports existing `pytest.approx` unknown-type
errors in unchanged test files; those are unrelated to this experiment.


The saved controller also passed the local isolated-controller grading-worker
path on both official seeds: 514.60 m (110) and 515.31 m (2026), with exactly
zero damage and wall contact. This checks subprocess loading and inference; it
is not a remote Gradescope submission. The complete controller archive is
`artifacts/formula110-speed-hybrid-controllers.zip`.
