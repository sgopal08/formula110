## Laboratory Notebook 7

**Date:** 9/8/26

**Objective:** Improve initial acceleration, sustained corner speed and track positioning using CMA-ES and reactive control, while accepting only zero-damage, zero-wall-contact controllers.

## Method

Kept the two previously evolved neural policies fixed and evolved eight reactive refinement coefficients with CMA-ES. These adjust launch assistance, cruise/corner throttle, lookahead steering, center correction, yaw damping, safety distance and launch fade-out speed. There is no fixed corner throttle cap or permanent left/right target. Zero coefficients exactly reproduce the previous racing-line hybrid.

Ran 12 generations of 10 candidates on four training seeds (480 full 30-second training races). Selected among the eight best feasible training candidates using six development seeds. All damaging/contacting candidates were infeasible regardless of distance. Inference stays pure Python and uses public local sensors only.

The first selected candidate failed the initial 20-seed test, with tiny wall contact on seeds 563 and 1693 and nonzero damage on 1693. It was rejected. The second-ranked candidate passed those 20 seeds plus the ten training/development seeds, then passed another 20 fresh seeds, both official seeds, and five 120-second runs.

## Results on 20 fresh seeds

| Measurement | Previous hybrid | New speed hybrid |
|---|---:|---:|
| Mean 30-second progress | 476.53 m | 513.51 m (+7.76%) |
| Mean time to 10 m/s | 3.126 s | 1.203 s (-61.50%) |
| Mean corner speed | 15.97 m/s | 16.56 m/s (+3.70%) |
| Mean best lap | 10.383 s | 10.080 s (-2.92%) |
| Mean absolute center offset | 1.295 m | 1.369 m |
| Mean off-track time | 0.397 s | 0.833 s |
| Damage / wall contact | 0 / 0 | 0 / 0 |

The new controller travels farther on all 20 fresh seeds. It also retains a 4.42% distance gain over 120-second trials, so the improvement is not solely from the faster launch. Across 52 distinct tested starts and five additional extended runs, the selected candidate records zero damage/contact and no elimination. These are solo tests on the existing track; other tracks and racing with competitors remain untested.

## Positioning and acceleration findings

Removing launch assistance from the selected controller lowers six-seed mean distance from 514.84 m to 492.46 m while remaining safe. Removing steering/centering/yaw refinement while retaining the extra throttle causes damage and major wall contact. The positioning correction is necessary to support the extra pace in this comparison.

The faster line uses slightly more track width on average rather than staying exactly centered. The experiment does not establish a globally optimal racing line, nor isolate each steering coefficient's individual contribution. Corner speed is measured by yaw rate (at least 20 degrees/s after the first five seconds); actual speed still varies through a lap.

## Decision

Saved `controllers.cmaes_speed_hybrid`, preserved the previous controllers and neural weights, and selected the new hybrid in `formula110-submission.json`. Accepted the small increase in off-track time while maintaining exact zero damage and wall contact in the acceptance suite.

129 repository tests passed. New controller/tests passed strict Pyright; new Python files passed Ruff lint/format. Existing unrelated `pytest.approx` typing errors remain in unchanged tests during whole-repository Pyright checking.

Detailed configuration, reproduction commands, selection history and ablations: [experiment report](../docs/SPEED_HYBRID_EXPERIMENT.md). Versioned benchmark evidence: [results](../docs/SPEED_HYBRID_RESULTS.json). Full experiment logs: `artifacts/cmaes-speed-hybrid/` and `artifacts/cmaes-speed-hybrid-safe/`.

The saved controller was additionally verified through the local grading worker on seeds 110 and 2026, reproducing 514.60 m and 515.31 m with zero damage/contact. A complete controller archive was exported as `artifacts/formula110-speed-hybrid-controllers.zip`.
