## Laboratory Notebook 5

**Date and time:** 8/31/26

**Participants and contributions:** Sanjana

## Question or objective

Can the CMA-ES champion be made faster with a small, controlled output adjustment while preserving its zero-damage and zero-wall-contact reliability?

## Hypothesis

The original evolved controller had best laps near 11 seconds and zero contact, suggesting that it may have retained a small safety margin. Increasing only the throttle output bias should improve acceleration and progress without changing the learned steering policy. Small increases may preserve reliability, while larger increases are expected to produce off-track driving, contact, and damage.

## Initial experiment

Keep all 122 evolved weights fixed except the final throttle-output bias. Evaluate several positive offsets under identical 30-second conditions on five held-out validation seeds and the two official seeds. Compare mean distance, best-lap time, maximum speed, damage, wall contact, off-track time, and minimum completed laps.

The first sweep tested offsets of `0.00`, `0.15`, `0.30`, `0.50`, and `0.75`. A narrower follow-up sweep tested `0.04`, `0.07`, `0.10`, `0.12`, and `0.14` to make the speed/safety boundary explicit.

## What we investigated or changed

We first compared the original CMA-ES champion with `reactive_v1` on official seeds 110 and 2026. The reactive controller reached a higher momentary top speed of approximately 18.0 m/s, but its best laps were slower at approximately 12.4 seconds and it accumulated wall contact and damage. The CMA-ES policy had a lower top speed of approximately 17.2 m/s but completed best laps near 11.0 seconds with no damage or contact. This showed that lap speed, not momentary maximum speed, was the correct performance target.

We then changed only the final neural-network output bias controlling throttle. Steering weights, sensor preprocessing, network architecture, fitness, and all other evolved parameters remained fixed. The selected adjustment was:

```text
POST_TRAINING_THROTTLE_BIAS_OFFSET = +0.15
```

The final stored throttle-output bias changed from approximately `-1.38972` to `-1.23972`. The change was recorded in `cmaes_weights.json` metadata.

## Evidence

- **AI-agent assistance:** Benchmarked the original controller against `reactive_v1`; ran coarse and fine throttle-bias sweeps; compared distance, lap time, maximum speed, damage, contact, and off-track time; updated the saved artifact; and reran held-out and isolated official evaluations.
- **Commits or code:** Updated `src/controllers/cmaes_weights.json`. The learned steering policy and controller source code were unchanged. No commit hash was available during this session.
- **Experiment configuration:** Seven evaluation seeds per candidate adjustment: held-out seeds 41, 137, 311, 509, and 887, plus official seeds 110 and 2026. Each trial lasted 30 seconds at 60 Hz with marshal recovery disabled.
- **Experiment output:** The `+0.15` setting increased mean distance from 417.05 m to 436.95 m and reduced mean best-lap time from 10.986 seconds to 10.605 seconds across seven runs.
- **Verification:** The final controller completed every training, validation, and official run with two laps, zero damage, zero wall contact, and no elimination. All 117 repository tests passed; new code continued to pass Ruff and strict Pyright checks.

## What we observed

Increasing throttle produced a clear speed/reliability tradeoff:

| Throttle-bias offset | Mean distance (m) | Mean best lap (s) | Maximum speed (m/s) | Total damage | Wall contact (s) | Off-track (s) | Minimum laps |
| -------------------: | ----------------: | ----------------: | ------------------: | -----------: | ---------------: | ------------: | -----------: |
|                 0.00 |            417.05 |            10.986 |               17.25 |        0.000 |            0.000 |         0.000 |            2 |
|                 0.15 |            436.95 |            10.605 |               18.10 |        0.000 |            0.000 |         1.900 |            2 |
|                 0.30 |            433.29 |            11.071 |               18.99 |        0.330 |            1.017 |         3.750 |            2 |
|                 0.50 |            373.35 |          10.343\* |               20.29 |        1.325 |           29.183 |        31.500 |            0 |
|                 0.75 |            252.16 |           6.881\* |               22.06 |        3.161 |           85.650 |        88.183 |            0 |

`*` The apparently fast lap averages at aggressive offsets are misleading because some runs failed to complete a lap. These settings had substantial collision and off-track penalties and were not reliable controllers.

The `+0.15` setting improved mean 30-second progress by approximately 4.8%, reduced mean best-lap time by approximately 0.38 seconds, and increased top speed by about 0.85 m/s. It preserved zero damage and zero wall contact across all seven development/test runs. Its cost was 1.9 seconds of total off-track time across 210 simulated seconds, less than 1% of evaluation time.

The narrow sweep showed that even a `+0.04` increase introduced 0.117 seconds of total off-track time. Therefore, there was no tested positive throttle adjustment that was both meaningfully faster and retained exactly zero off-track time. The choice was a deliberate tradeoff rather than a free improvement.

## Decision and rationale

We selected the `+0.15` throttle-output bias because it provided the largest reliable improvement before damage and wall contact appeared. The `+0.30` and larger settings were rejected because they converted extra speed into collisions, damage, and inconsistent completion. The `+0.04` through `+0.14` settings provided smaller speed improvements but still introduced some off-track time.

The final controller favors faster leaderboard-relevant progress while preserving the most important safety requirements: survival, zero damage, and zero wall contact. We documented the small off-track tradeoff explicitly rather than describing the adjustment as an unconditional improvement.

## Next steps

1. Use another CMA-ES refinement run centered on the champion to learn steering changes that support the higher throttle without leaving the track.
2. Keep the `+0.15` controller and original zero-off-track controller as separate checkpoints for controlled comparison.
3. Evaluate future candidates on additional unseen seeds rather than repeatedly tuning against the five current validation seeds.
4. Test head-to-head performance against human control and the reactive baseline.
5. Consider adding a stronger off-track penalty during fine-tuning if exact track containment is more important than the current speed gain.

## Overall

- **10/10 final runs completed across training, validation, and official seeds**
- **2 laps completed on every run**
- **0 eliminations, 0 damage, and 0 wall contact**
- **Mean seven-seed progress improved by approximately 4.8%**
- **Mean seven-seed best lap improved from 10.986 to 10.605 seconds**
- **Official-seed progress increased from 422.35/415.12 m to 441.62/434.85 m**
- **Small documented off-track tradeoff: less than 1% of total seven-seed evaluation time**

## Experiment Configuration

| Field                  | Value                             | Purpose                                         |
| ---------------------- | --------------------------------- | ----------------------------------------------- |
| Base controller        | Generation-10 CMA-ES champion     | Preserve learned steering and sensor behavior   |
| Changed parameter      | Final throttle-output bias only   | Isolate the cause of performance change         |
| Coarse offsets         | 0.00, 0.15, 0.30, 0.50, 0.75      | Identify the useful and unsafe regions          |
| Fine offsets           | 0.04, 0.07, 0.10, 0.12, 0.14      | Examine the speed/off-track boundary            |
| Selected offset        | `+0.15`                           | Best tested gain before damage/contact appeared |
| Evaluation seeds       | 41, 137, 311, 509, 887, 110, 2026 | Held-out and official positions                 |
| Final additional seeds | 17, 83, 241                       | Confirm behavior on training positions          |
| Trial duration         | 30 seconds                        | Match official evaluation                       |
| Timestep               | `1/60` second                     | Standard simulator control rate                 |
| Marshal recovery       | Disabled                          | Prevent automatic reset from hiding failure     |

## Parameters

| Parameter                          |         Before |     After | Purpose                                   |
| ---------------------------------- | -------------: | --------: | ----------------------------------------- |
| Post-training throttle-bias offset |           0.00 |   `+0.15` | Increase acceleration and sustained speed |
| Stored throttle-output bias        |       -1.38972 |  -1.23972 | Neural-network throttle logit bias        |
| Network architecture               | `12 -> 8 -> 2` | unchanged | Preserve the learned policy               |
| Other evolved parameters           |     121 values | unchanged | Make this a one-variable experiment       |

## Five-seed held-out validation results

| Seed | Laps | Distance (m) | Best lap (s) | Damage | Wall contact (s) | Off-track (s) |
| ---: | ---: | -----------: | -----------: | -----: | ---------------: | ------------: |
|   41 |    2 |       444.71 |        10.62 | 0.0000 |            0.000 |         0.250 |
|  137 |    2 |       427.22 |        10.55 | 0.0000 |            0.000 |         0.317 |
|  311 |    2 |       434.38 |        10.65 | 0.0000 |            0.000 |         0.167 |
|  509 |    2 |       446.15 |        10.63 | 0.0000 |            0.000 |         0.250 |
|  887 |    2 |       429.75 |        10.58 | 0.0000 |            0.000 |         0.283 |

## Official-seed results

| Seed | Laps | Distance (m) | Best lap (s) | Max speed (m/s) | Damage | Wall contact (s) |
| ---: | ---: | -----------: | -----------: | --------------: | -----: | ---------------: |
|  110 |    2 |       441.62 |        10.62 |           18.10 | 0.0000 |            0.000 |
| 2026 |    2 |       434.85 |        10.58 |           18.00 | 0.0000 |            0.000 |
