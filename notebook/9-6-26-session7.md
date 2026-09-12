## Laboratory Notebook 7

**Date and time:** 9/6/26

**Participants and contributions:** Yewon

## Question or objective

Can an auditable, stateful centerline controller with explicitly separated sensor translation, expert control,
safety, and parameter-optimization layers improve on `reactive_v1` while completing every validation run with zero
damage and zero wall contact?

## Hypothesis

Filtering public track geometry across a bounded history, exposing interpretable path-bend features, and using active
braking against a curvature-dependent target speed should produce a safer and more stable centerline controller than
the earlier snapshot-only reactive controller. CMA-ES should then improve the controller by tuning a small vector of
meaningful gains without changing its authored structure.

## Initial experiment

Implement two preserved artifacts with the same controller architecture. `centerline_v1` uses hand-authored defaults;
`centerline_v2` will contain parameters promoted by CMA-ES only if the candidate passes held-out safety and performance
gates. Training uses seven deterministic seeds, validation uses five disjoint seeds, and official seeds 110 and 2026
remain final-audit-only.

## What we investigated or changed

The runtime controller was divided into explicit layers:

1. `SensorTranslator` sanitizes and filters public observations and derives lookahead slopes, path bend, bend change,
   temporal rates, and finite wall clearances.
2. `CenterlineExpert` targets zero lateral offset and exposes its individual steering terms and target speed.
3. `SafetyFilter` applies fixed side-wall correction and frontal emergency braking after nominal command smoothing.
4. `CenterlineController` owns bounded per-race state and composes the layers through `create_controller()` factories.

The written plan described a twelve-parameter optimization vector but explicitly listed thirteen values. The
implementation preserves all thirteen named parameters and records this correction rather than silently dropping one.

## Evidence

- **AI-agent assistance:** Designed and implemented the typed layer contracts, controller artifacts, parameter mapping,
  safety-tier fitness, CMA-ES training harness, tests, evaluation, and documentation.
- **Commits or code:** Added the `src/controllers/centerline/` package, `src/controllers/centerline_v1.py`,
  `src/controllers/centerline_v2.py`, `src/racing/experiments/centerline.py`, and
  `scripts/train_centerline_cmaes.py`. Added `scripts/evaluate_centerline.py` and
  `tests/test_centerline_controller.py`. Updated the controller exporter so selected submissions follow nested local
  controller packages as well as single helper modules.
- **Experiment configuration:** Population 16, 20 generations, 30-second trials, training
  seeds 17/41/83/137/241/311/509, validation seeds 613/719/823/929/1031, sigma 0.25, and optimizer seed 20260906.
- **Experiment output:** Evaluated 320 CMA-ES candidates in 2,240 seeded training trials. The complete run took
  approximately 1,851 seconds. Configuration, JSONL metrics, optimizer state, best training parameters, promoted
  parameters, and final audits are under `artifacts/centerline-cmaes/`.
- **Verification:** The 10 new controller tests passed. All 129 runnable repository tests passed; one existing
  autograder diagnostic test fails on Windows because its `bash` child exits differently than on Linux. Strict Pyright
  reported zero errors, and all changed files passed Ruff lint and format checks. Two identical seed-3 evaluations
  produced exactly identical metric dictionaries. The controller process used approximately 44 MB resident memory
  after 1,800 inference calls.

## What we observed

The authored `centerline_v1` was exceptionally conservative. It completed one lap on every validation seed with zero
damage, wall contact, and off-track time, but its best laps were approximately 24.2 seconds and its mean distance was
230.92 meters. This was substantially slower than `reactive_v1`, whose validation mean was 402.02 meters with best
laps near 12.4 seconds and nonzero contact and damage on every run.

CMA-ES first produced a candidate in the complete two-lap safety tier during generation 7. Generation 10 passed the
held-out promotion gate. Generation 17 was the first population with a positive mean safety-tier fitness, showing that
the distribution had learned a broadly viable parameter region rather than producing only one successful individual.
The final champion was generation 20, individual 8, checksum `a09b0cbe8179171d`, with fitness `534.9773`.

The final champion completed two laps on every training and validation seed with zero damage and wall contact. Its
validation mean distance was 435.13 meters, an 8.2% improvement over the `reactive_v1` validation mean, and no
validation seed regressed. Four validation runs had zero off-track time; seed 613 recorded 0.300 seconds.

Across 15 broad-validation seeds and the two official seeds, `centerline_v2` completed 17/17 two-lap runs with zero
eliminations, damage, and wall contact. It averaged 437.96 meters and a 12.138-second best lap. Total off-track time was
0.333 seconds. The preserved `cmaes_v2` stretch benchmark remained faster at 477.63 meters and 10.399 seconds but
accumulated 6.367 seconds off-track over the same runs.

## Decision and rationale

We promoted the generation-20 parameters to `centerline_v2` because they satisfied every required held-out gate:
survival, two laps, exactly zero damage and wall contact, higher mean distance than `reactive_v1`, and no per-seed
distance regression greater than one meter. The authored configuration remains independently available as
`centerline_v1`.

The official submission remains `controllers.cmaes_v2` because it is substantially faster and the centerline artifact
was intended as an auditable control baseline rather than an automatic submission replacement. The official seeds
were evaluated only after the tuned parameters had been frozen.

## Next steps

1. Inspect watched centerline runs to relate expert decision terms to entry, apex, and exit behavior.
2. Use the same translated state and feedback/safety layers for a separate racing-line target planner.
3. Preserve `centerline_v2` as the controlled comparison for any lateral-target changes.
4. Perform Linux Gradescope execution when the actual grading environment is available; the local Windows worker
   cannot use its required `pass_fds`, and the available WSL distribution does not contain Panda3D.

## Overall

- **320 CMA-ES candidates and 2,240 seeded training trials completed**
- **Generation-20 champion passed every held-out promotion requirement**
- **17/17 broad and official audits completed two laps**
- **0 eliminations, 0 damage, and 0 wall contact for `centerline_v2`**
- **Mean broad/official distance: 437.96 meters**
- **Mean broad/official best lap: 12.138 seconds**
- **Exact deterministic replay confirmed**
- **Approximate controller-process resident memory: 44 MB**

## Experiment Configuration

| Field | Value | Purpose |
|---|---:|---|
| Architecture | Translated state -> centerline expert -> smoothing -> safety | Keep responsibilities auditable |
| Tunable parameters | 13 | Preserve every value enumerated in the plan |
| Population | 16 | Candidates per generation |
| Generations | 20 | Fixed optimization budget |
| Training seeds | 17, 41, 83, 137, 241, 311, 509 | Update CMA-ES |
| Validation seeds | 613, 719, 823, 929, 1031 | Promotion without optimizer updates |
| Broad-validation seeds | 3, 29, 61, 107, 173, 257, 349, 433, 547, 661, 773, 997, 1109, 1223, 1301 | Frozen audit |
| Official seeds | 110, 2026 | Final frozen audit only |
| Trial duration | 30 seconds | Match grading duration |
| Initial sigma | 0.25 | Search around authored normalized gains |
| Optimizer seed | 20260906 | Reproducibility |

## Promoted Parameters

| Parameter | Authored v1 | Tuned v2 |
|---|---:|---:|
| Center gain | 0.220000 | 0.128575 |
| Heading gain | 0.012000 | 0.016427 |
| Bend gain | 0.550000 | 1.073785 |
| Bend-change gain | 1.000000 | 2.320606 |
| Yaw damping | 0.001500 | 0.000686 |
| Straight target speed (m/s) | 16.000000 | 19.957649 |
| Corner-speed reduction (m/s) | 10.500000 | 3.088504 |
| Center speed penalty | 2.000000 | 0.317142 |
| Heading speed penalty | 0.040000 | 0.081373 |
| Acceleration gain | 0.140000 | 0.156540 |
| Braking gain | 0.300000 | 0.310693 |
| Steering smoothing (s) | 0.050000 | 0.038549 |
| Throttle smoothing (s) | 0.080000 | 0.137578 |

## Validation Results

| Seed | Laps | Distance (m) | Best lap (s) | Damage | Wall contact (s) | Off-track (s) |
|---:|---:|---:|---:|---:|---:|---:|
| 613 | 2 | 430.49 | 12.13 | 0.0000 | 0.000 | 0.300 |
| 719 | 2 | 434.28 | 12.15 | 0.0000 | 0.000 | 0.000 |
| 823 | 2 | 438.14 | 12.15 | 0.0000 | 0.000 | 0.000 |
| 929 | 2 | 437.62 | 12.13 | 0.0000 | 0.000 | 0.000 |
| 1031 | 2 | 435.11 | 12.13 | 0.0000 | 0.000 | 0.000 |

## Official-Seed Results

| Seed | Laps | Distance (m) | Partial laps | Best lap (s) | Max speed (m/s) | Damage | Wall contact (s) |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 110 | 2 | 435.16 | 2.3770 | 12.13 | 19.79 | 0.0000 | 0.000 |
| 2026 | 2 | 436.95 | 2.3868 | 12.13 | 19.54 | 0.0000 | 0.000 |

## Post-run braking diagnosis

Manual review identified several stop-like events near large bends in both centerline artifacts. I traced the
translated state, nominal expert decision, final command, target speed, steering, camera visibility, and wall
clearances at 60 Hz. The events are not consistently caused by the fixed safety filter: representative runs had
front and side clearances well above its thresholds, and most events contained no safety intervention.

The root cause is a mismatch between the controller's assumed and actual throttle semantics. The centerline expert
treats negative throttle as a releasable proportional brake. In `resolve_vehicle_actuator_command`, however, a
negative command while moving forward starts a pending direction change. Subsequent commands continue applying the
brake until vehicle speed falls below the 1 km/h direction-change threshold. Returning the policy output to positive
throttle changes the pending direction back to forward, but does not release the brake before the vehicle has nearly
stopped. Thus even a short corner-entry negative-throttle pulse can commit the actuator layer to a full stop.

This was directly visible in a seed-719 `centerline_v2` trace. The nominal bend response briefly commanded negative
throttle while speed fell from roughly 19 m/s toward its 16.87 m/s corner target. The policy had returned to strong
positive throttle by roughly 27.1 seconds, yet measured forward speed continued down through 1.3 m/s at 28.05 seconds
before recovering. `centerline_v1` exhibited the same transition repeatedly and more severely because its corner
target floor is only 5.5 m/s and its negative commands are longer and stronger.

Large bends correlate with the symptom because path-bend demand lowers the target speed and initiates the negative
command; they are not themselves a consistent emergency condition. High steering and yaw can amplify the visible
slowdown, but the persistent direction-transition braking explains why the vehicle reaches almost exactly zero after
the policy has already requested acceleration.

No controller change was made during this diagnostic pass. Any correction must be treated as a new controller
revision and revalidated because removing negative throttle affects stopping distance, safety, and the meaning of the
previous CMA-ES-tuned braking gain.
