## Laboratory Notebook 6

**Date and time:** 9/9/26

**Participants and contributions:** Sanjana

## Question or objective

Can the CMA-ES controller learn a faster racing line that uses more of the track and cuts corners while still avoiding wall contact, damage, and elimination?

## Hypothesis

The previous controller was reliable but conservative. It generally followed the processed centerline and retained a safety margin near corners. Fine-tuning its neural-network weights with a speed-focused fitness should allow it to use more of the track width, approach corner apexes, and complete laps faster. Because aggressive optimization may produce unsafe policies, a wall-based safety mechanism may be necessary to preserve the zero-damage requirement.

## Initial experiment

Preserve the existing fast controller as a checkpoint and initialize a new CMA-ES run from its 122 learned parameters instead of starting from zero. Keep the same `12 -> 8 -> 2` architecture so that the experiment isolates the effect of refinement and fitness rather than changing both the network and the objective.

Use a smaller CMA-ES sigma of `0.10`, a population of 16, and 15 generations. Evaluate each candidate for 30 seconds on seven deterministic training seeds. Use five fresh held-out seeds for validation. The revised racing-line fitness should increase the reward for forward progress and lap speed, reduce the relative off-track penalty, and make damage, wall contact, and elimination much more expensive.

## What we investigated or changed

The training pipeline was extended to accept a saved weights file as the initial CMA-ES mean. The starting controller is evaluated before the first sampled generation and remains preserved unless a better candidate is found. The trainer also records complete parameter vectors so later candidates can be reproduced rather than identified only by a checksum.

We added a separate racing-line fitness profile. It retained forward partial laps as the main reward, increased the early-lap bonus, and used the following safety priorities:

- Damage penalty weight: `5.0`.
- Elimination penalty: `5.0`.
- Wall-contact penalty weight: `3.0`.
- Off-track penalty weight: `0.5`.

The lower off-track weight intentionally allowed the optimizer to use more of the track instead of treating centerline driving as the goal. There was still no direct penalty for center offset, so the neural policy was free to move toward the inside of a corner when useful.

The raw generation-14 champion was substantially faster, but it caused minor wall contact and damage on two fresh validation seeds. We rejected that controller as the final policy. Interpolating between the safe and aggressive neural weights was also tested, but this was unsuccessful because neural-network behavior changed nonlinearly and the interpolated policies still contacted walls.

We then created a hybrid controller called `cmaes_racing_line`. The aggressive evolved policy drives normally, but the preserved safe CMA-ES policy takes over whenever any of seven wall-only LiDAR beams detects a wall within 1.8 meters. The safety check uses beams at `-90`, `-45`, `-20`, `0`, `20`, `45`, and `90` degrees.

## Evidence

- **AI-agent assistance:** Added saved-weight initialization, implemented the racing-line fitness, ran the 1,680-trial CMA-ES refinement, analyzed unsafe candidates, tested weight interpolation, designed the LiDAR safety shield, ran broad held-out validation, and verified the final controller through the isolated grading worker.
- **Commits or code:** Added `src/controllers/cmaes_racing_line.py`, `src/controllers/cmaes_racing_line_weights.json`, and `src/controllers/cmaes_weights_pre_racing_line.json`. Updated `scripts/train_cmaes.py`, `src/racing/experiments/neuroevolution.py`, `tests/test_cmaes_neuroevolution.py`, and `docs/CMAES_EXPERIMENT.md`. No commit hash was available during this session.
- **Experiment configuration:** 16 population members, 15 generations, 7 training seeds, 5 fresh validation seeds, 30-second trials, 60 Hz, initial sigma 0.10, optimizer seed 20260901, and marshal recovery disabled.
- **Experiment output:** The best refinement score increased to `3.2391` at generation 14. Detailed results were saved under `artifacts/cmaes-racing-line/`, including `candidate_weights.json` and `broad-validation.json`.
- **Verification:** The final hybrid completed 17/17 broad-validation runs with two laps, zero damage, zero wall contact, and zero eliminations. The full repository test suite passed with 119 tests, and the changed files passed Ruff and strict Pyright checks.

## What we observed

The CMA-ES population improved during refinement. The best score was `2.8218` in generation 1, passed `3.0` in generation 7, and reached `3.2391` in generation 14. The population mean also became positive, indicating that the distribution around the original champion was learning useful changes rather than producing only one lucky individual.

The unprotected generation-14 champion demonstrated that the new objective could find a much faster racing line. Across five fresh validation seeds and the two official seeds, it averaged 507.05 meters and a 9.67-second best lap. However, it accumulated 0.022 total damage, 0.133 seconds of wall contact, and 9.317 seconds off-track. Since it damaged the car on two fresh seeds, the raw champion did not satisfy the reliability requirement.

Weight interpolation did not provide a safe compromise. Even an interpolation factor of `0.20` caused damage on every comparison seed. This showed that averaging neural weights does not necessarily average controller behavior.

The 1.8-meter LiDAR safety shield successfully combined the strengths of both learned policies. In the initial seven-seed comparison, it averaged 473.60 meters and a 10.38-second best lap while recording zero damage and zero wall contact. A broader evaluation on 15 additional seeds plus official seeds 110 and 2026 produced 17/17 successful runs, an average distance of 477.70 meters, and an average best lap of 10.40 seconds. Every run completed two laps.

The hybrid did use the track more aggressively. It accumulated 6.45 seconds off-track across 510 total simulated seconds, or approximately 0.38 seconds per run. This was a measurable tradeoff, but it did not result in wall contact, damage, or elimination.

## Decision and rationale

We selected the hybrid `cmaes_racing_line` controller rather than the unprotected racing-line champion. The raw champion was faster, but its damage on unseen seeds violated the primary safety requirement. The hybrid retained a meaningful portion of the speed improvement and achieved zero damage and wall contact across a substantially broader test set.

The original controller was not overwritten. Its weights were preserved as `cmaes_weights_pre_racing_line.json`, allowing direct comparisons and providing the safety fallback. This decision also makes the contribution of each component clear: the aggressive policy selects the faster racing line, while the safe policy handles states close to track boundaries.

## Next steps

1. Test the hybrid on additional unseen seeds and head-to-head races.
2. Measure how often the safety fallback activates and where on the track it activates.
3. Fine-tune the aggressive policy with a hard per-run rejection for any nonzero damage or wall contact.
4. Investigate smooth blending or hysteresis around the 1.8-meter threshold to avoid rapid switching between policies.
5. Test safety distances near 1.8 meters on a new seed set rather than reusing the current validation seeds.
6. Preserve both the original and hybrid controllers so future changes can be compared against each one.

## Overall

- **1,680 full 30-second training races completed**
- **Best CMA-ES refinement score: 3.2391 at generation 14**
- **Raw racing-line policy was faster but rejected because it caused minor damage**
- **17/17 hybrid validation runs completed two laps**
- **0 eliminations, 0 damage, and 0 wall contact for the final hybrid**
- **Average broad-validation distance: 477.70 meters**
- **Average broad-validation best lap: 10.40 seconds**
- **Official-seed partial progress increased to 2.58 and 2.61 laps**

## Experiment Configuration

| Field | Value | Purpose |
|---|---|---|
| Starting controller | Previous `+0.15` CMA-ES champion | Begin from a proven fast and reliable policy |
| Architecture | `12 -> 8 -> 2` | Keep model capacity unchanged |
| Evolved parameters | 122 | Optimize all neural weights and biases |
| Population | 16 | Candidates per generation |
| Generations | 15 | Controlled refinement budget |
| Training seeds | 17, 41, 83, 137, 241, 311, 509 | Train across varied starts |
| Held-out seeds | 613, 719, 823, 929, 1031 | Validate without updating CMA-ES |
| Broad-validation seeds | 3, 29, 61, 107, 173, 257, 349, 433, 547, 661, 773, 997, 1109, 1223, 1301, 110, 2026 | Test generalization and official behavior |
| Trial duration | 30 seconds | Match official evaluation |
| Timestep | `1/60` second | Standard simulator rate |
| Initial sigma | 0.10 | Search locally around the previous champion |
| Optimizer seed | 20260901 | Make the run reproducible |
| Marshal recovery | Disabled | Ensure failures are not hidden by resets |

## Parameters

| Parameter | Value | Purpose |
|---|---:|---|
| `SAFETY_DISTANCE_M` | 1.8 m | Switch to the safe policy near a wall |
| `SAFETY_LIDAR_ANGLES` | `(-90, -45, -20, 0, 20, 45, 90)` | Detect walls around the car |
| Racing-line damage weight | 5.0 | Strongly discourage damaging policies |
| Racing-line elimination penalty | 5.0 | Reject catastrophic failure |
| Racing-line wall-contact weight | 3.0 | Discourage touching track barriers |
| Racing-line off-track weight | 0.5 | Permit controlled use of track width |
| Racing-line lap bonus weight | 0.75 | Reward earlier lap completion |

## Official-seed results

| Controller | Seed | Laps | Distance (m) | Partial laps | Best lap (s) | Max speed (m/s) | Damage | Wall contact (s) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Previous fast CMA-ES | 110 | 2 | 441.62 | 2.412 | 10.62 | 18.10 | 0.0000 | 0.000 |
| Racing-line hybrid | 110 | 2 | 472.21 | 2.579 | 10.35 | 17.80 | 0.0000 | 0.000 |
| Previous fast CMA-ES | 2026 | 2 | 434.85 | 2.375 | 10.58 | 18.00 | 0.0000 | 0.000 |
| Racing-line hybrid | 2026 | 2 | 477.65 | 2.609 | 10.40 | 17.81 | 0.0000 | 0.000 |

## Broad-validation results

| Seed | Laps | Distance (m) | Best lap (s) | Damage | Wall contact (s) | Off-track (s) |
|---:|---:|---:|---:|---:|---:|---:|
| 3 | 2 | 473.90 | 10.45 | 0.0000 | 0.000 | 0.617 |
| 29 | 2 | 466.13 | 10.30 | 0.0000 | 0.000 | 0.317 |
| 61 | 2 | 485.32 | 10.37 | 0.0000 | 0.000 | 0.267 |
| 107 | 2 | 484.79 | 10.45 | 0.0000 | 0.000 | 0.500 |
| 173 | 2 | 484.84 | 10.37 | 0.0000 | 0.000 | 0.283 |
| 257 | 2 | 472.79 | 10.37 | 0.0000 | 0.000 | 0.350 |
| 349 | 2 | 471.52 | 10.45 | 0.0000 | 0.000 | 0.633 |
| 433 | 2 | 473.39 | 10.37 | 0.0000 | 0.000 | 0.267 |
| 547 | 2 | 479.59 | 10.40 | 0.0000 | 0.000 | 0.283 |
| 661 | 2 | 488.49 | 10.42 | 0.0000 | 0.000 | 0.333 |
| 773 | 2 | 466.29 | 10.43 | 0.0000 | 0.000 | 0.250 |
| 997 | 2 | 484.06 | 10.45 | 0.0000 | 0.000 | 0.433 |
| 1109 | 2 | 482.76 | 10.32 | 0.0000 | 0.000 | 0.367 |
| 1223 | 2 | 470.13 | 10.40 | 0.0000 | 0.000 | 0.417 |
| 1301 | 2 | 486.99 | 10.45 | 0.0000 | 0.000 | 0.500 |
| 110 | 2 | 472.21 | 10.35 | 0.0000 | 0.000 | 0.350 |
| 2026 | 2 | 477.65 | 10.40 | 0.0000 | 0.000 | 0.283 |
