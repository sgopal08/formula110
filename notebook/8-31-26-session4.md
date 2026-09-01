## Laboratory Notebook 4

**Date and time:** 8/31/26

**Participants and contributions:** Sanjana

## Question or objective

Can CMA-ES evolve all weights of a small fixed neural network into a fast and reliable Formula 110 controller that generalizes across starting positions?

## Hypothesis

The simulator's processed camera geometry, wall-only LiDAR, speed, and yaw rate contain enough local information for a small neural network to follow the track. CMA-ES should be able to optimize the complete network without gradients if the model remains small and fitness strongly penalizes inconsistent, damaging, or off-track behavior.

## Initial experiment

Implement a fixed `12 -> 8 -> 2` multilayer perceptron and use CMA-ES to optimize all 122 weights and biases. Evaluate every individual on the same three deterministic training seeds and reserve five different seeds for held-out validation. Use a scalar fitness that rewards forward progress and early lap completion while penalizing off-track time, wall contact, damage, elimination, and inconsistent performance across seeds.

The minimum viable run used 20 individuals for 25 generations. Each training trial lasted 20 simulated seconds at 60 Hz with marshal recovery disabled.

## What we investigated or changed

We first inspected the controller API, complete sensor reference, race runtime, progress tracker, headless evaluation utilities, example controllers, and autograder constraints. The controller can use public sensor snapshots but cannot observe official progress or mutable physics state. Official progress, contact, damage, off-track time, lap time, and survival can still be measured externally for fitness.

We selected a compact 12-value observation vector:

1. Signed speed.
2. Yaw rate.
3. Track-center offset.
4. Track-heading error.
5. Camera lookahead offset at 4 m.
6. Camera lookahead offset at 9 m.
7. Camera lookahead offset at 16 m.
8. Wall LiDAR at -90 degrees.
9. Wall LiDAR at -20 degrees.
10. Wall LiDAR at 0 degrees.
11. Wall LiDAR at 20 degrees.
12. Wall LiDAR at 90 degrees.

Absolute world heading, tick, odometry distance, pitch, roll, competitor readings, and contact state were excluded from the initial policy. They were redundant for single-car track following, described failure after it occurred, or could encourage track/time memorization. Infinite LiDAR readings were replaced with a finite cap before normalization.

We implemented dependency-free controller inference and kept `cma` and NumPy as training-only dependencies. Each evaluation received a fresh physics world, car, lap tracker, sensor state, and controller. Repeated evaluation of the same weights and seed produced exactly identical metrics.

## Evidence

- **AI-agent assistance:** Inspected repository documentation and tests; designed the observation vector, network, individual encoding, fitness, and seed split; implemented the controller and evaluator; added automated tests; executed the full CMA-ES run; and performed isolated Gradescope-style verification.
- **Commits or code:** Added `src/controllers/cmaes_policy.py`, `src/controllers/cmaes_neuroevolution.py`, `src/controllers/cmaes_weights.json`, `src/racing/experiments/neuroevolution.py`, `scripts/train_cmaes.py`, `tests/test_cmaes_neuroevolution.py`, and `docs/CMAES_EXPERIMENT.md`. No commit hash was available during this session.
- **Experiment configuration:** 20 population members, 25 generations, 3 training seeds, 5 held-out validation seeds, 20-second training trials, 60 Hz, CMA-ES optimizer seed 110, initial sigma 0.35, and parameter bounds `[-5, 5]`.
- **Experiment output:** 500 individuals and 1,500 seeded training trials completed in approximately 870 seconds. Raw records were written to `artifacts/cmaes-mvp/metrics.jsonl`; final evaluation was written to `artifacts/cmaes-mvp/final_evaluation.json`.
- **Verification:** All 117 repository tests passed. The new files passed Ruff lint/format checks and strict Pyright checks. The isolated controller worker reproduced the official-seed results.

## What we observed

The first populations mostly drove off-track or contacted walls, so early robust fitness values were negative. The best generation-1 fitness was `-1.2253`. Fitness improved substantially during evolution: generation 7 reached `-0.0931`, generation 9 reached `0.2846`, and generation 10 produced the best-ever individual with fitness `1.5571`.

The champion was generation 10, individual 14, with parameter checksum `ef6aa46e014b22bd`. Later generations did not surpass this individual, but the population distribution continued improving. The generation mean increased from approximately `-1.87` at the beginning to positive values in the final part of the run. Held-out validation fitness improved from `-1.3597` at generation 5 to `1.5138` at generation 10.

In independent 30-second evaluation before any later speed adjustment, all three training seeds, all five held-out seeds, and both official seeds completed two laps with zero damage, zero wall contact, and zero off-track time. Held-out best laps were approximately 10.95 to 11.02 seconds. The results were almost identical across seed groups, providing evidence that the controller learned a general local track-following policy rather than memorizing one start.

## Decision and rationale

The minimum viable CMA-ES experiment succeeded and justified continuing with neuroevolution. It produced a controller that was faster and safer than the manually written reactive baseline while using only public observations. On official seeds, the evolved controller's best laps were approximately 11.0 seconds compared with approximately 12.4 seconds for `reactive_v1`, and the evolved controller had zero damage and wall contact.

We retained the generation-10 champion rather than the final generation's best individual because checkpointing tracked the best result across the entire run. We also retained the small architecture because 122 parameters were sufficient; increasing network size was not supported by current evidence.

## Next steps

1. Test whether a small increase in throttle can improve distance and lap time without introducing damage or wall contact.
2. Change only one output parameter first so the effect can be attributed clearly.
3. Evaluate every candidate adjustment on the same held-out and official seeds.
4. Reject aggressive settings that gain speed by crashing, contacting walls, or failing from some starts.
5. Preserve the original evolved weights so the safety/speed tradeoff remains reproducible.

## Overall

- **25 generations and 500 CMA-ES individuals evaluated**
- **1,500 deterministic training trials completed**
- **10/10 independent 30-second runs completed across training, validation, and official seeds**
- **2 laps completed on every independent run**
- **0 eliminations, 0 damage, 0 wall contact, and 0 off-track time**
- **Best official lap approximately 10.97 seconds**
- **CMA-ES was successful enough to continue as the primary refinement approach**

## Experiment Configuration

| Field | Value | Purpose |
|---|---|---|
| Architecture | `12 -> 8 -> 2` | Small fixed MLP suitable for black-box optimization |
| Evolved parameters | 122 | All network weights and biases |
| Population | 20 | Candidate policies per generation |
| Generations | 25 | Minimum viable optimization budget |
| Training seeds | 17, 83, 241 | Optimize across multiple starting positions |
| Validation seeds | 41, 137, 311, 509, 887 | Measure generalization without updating CMA-ES |
| Official test seeds | 110, 2026 | Final Gradescope-style verification only |
| Training trial duration | 20 seconds | Balance behavioral evidence and computation |
| Final trial duration | 30 seconds | Match official evaluation |
| Timestep | `1/60` second | Standard simulator control rate |
| Initial sigma | 0.35 | Initial CMA-ES search spread |
| Parameter bounds | `[-5, 5]` | Prevent extreme network parameters |
| Optimizer seed | 110 | Make the evolutionary run reproducible |

## Parameters

| Parameter | Value | Purpose |
|---|---:|---|
| `INPUT_SIZE` | 12 | Compact normalized sensor vector |
| `HIDDEN_SIZE` | 8 | Small nonlinear representation |
| `OUTPUT_SIZE` | 2 | Steering and throttle |
| `PARAMETER_COUNT` | 122 | CMA-ES search-vector length |
| `SPEED_CAP_MPS` | 12.0 | Speed normalization cap |
| `YAW_RATE_CAP_DEGREES_PER_S` | 180.0 | Yaw-rate normalization cap |
| `CENTER_OFFSET_CAP_M` | 6.0 | Center-offset normalization cap |
| `LOOKAHEAD_OFFSET_CAP_M` | 12.0 | Lookahead normalization cap |
| `LIDAR_CAP_M` | 20.0 | Finite replacement and range cap |
| `MIN_THROTTLE` | 0.05 | Prevent completely stationary random policies |
| Wall LiDAR angles | `(-90, -20, 0, 20, 90)` | Side and forward wall clearance |
| Hidden activation | `tanh` | Bounded signed hidden representation |
| Steering activation | `tanh` | Produces steering in `[-1, 1]` |
| Throttle activation | sigmoid | Produces forward throttle in `[0.05, 1]` |

## Five-seed held-out validation results

These results are from the original CMA-ES champion before the Session 5 throttle-bias refinement.

| Seed | Laps | Distance (m) | Best lap (s) | Max speed (m/s) | Damage | Wall contact (s) | Off-track (s) |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 41 | 2 | 424.07 | 11.00 | 16.93 | 0.0000 | 0.000 | 0.000 |
| 137 | 2 | 408.84 | 10.95 | 17.19 | 0.0000 | 0.000 | 0.000 |
| 311 | 2 | 413.23 | 11.02 | 17.22 | 0.0000 | 0.000 | 0.000 |
| 509 | 2 | 425.43 | 11.00 | 16.90 | 0.0000 | 0.000 | 0.000 |
| 887 | 2 | 410.29 | 10.97 | 17.11 | 0.0000 | 0.000 | 0.000 |

