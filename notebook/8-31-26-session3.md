## Laboratory Notebook 3

**Date and time:** 8/31/26

**Participants and contributions:** Yewon

## Question or objective

Can predictive slowing from processed lookahead geometry reduce wall contact and damage while preserving the reliability of `reactive_v1`?

## Hypothesis

The v1 controller reacts primarily to current steering demand and nearby walls. Using the magnitude of future centerline offsets to reduce throttle before a corner should reduce corner-entry speed and collision exposure without changing the steering policy.

## Initial experiment

Create `reactive_v2` by preserving v1 steering and wall recovery, then add a single behavior: throttle caps based on normalized lookahead offset (`abs(offset) / lookahead distance`). Compare v2 directly with v1 under identical deterministic conditions.

## What we investigated or changed

We added predictive corner slowing using the public `camera.lookahead_offsets_m` and `camera.lookahead_distances_m` observations. No learning, optimization, privileged state, or external dependency was introduced.

## Evidence

- **AI-agent assistance:** Implemented `reactive_v2`, ran static checks, ran the full test suite, and evaluated v1 and v2 on five fixed seeds.
- **Commits or code:** Added `src/controllers/reactive_v2.py`; exact version is the file contents in this working tree. No commit hash was available.
- **Experiment configuration:** One 30-second race per seed, seeds 1–5, 60 Hz, marshal recovery disabled, team-sum scoring, 1 m win margin, v2 versus v1.

## What we observed

Both versions completed all five runs and two laps per run. V2 lost speed and distance on seeds 1, 4, and 5, and won on seeds 2 and 3. Its wall contact remained low, but damage was not consistently lower. The predictive throttle therefore traded away performance without establishing a reliable safety gain.

## Decision and rationale

The lookahead-based change is retained as a documented negative experiment, but v1 remains the better baseline. The next optimization should first instrument how often each throttle branch activates and collect per-tick traces before selecting another controller change.

## Next steps

Add lightweight branch-activation and sensor-summary logging to the experiment harness, then use that evidence to choose one parameter or behavior for the next controlled experiment.

## Overall

- **5/5 v2 runs completed**
- **0 eliminations**
- **2/5 wins against v1 in the comparison runner**
- **No measured improvement over v1; v2 was slower overall**

## Experiment Configuration

| Field | Value | Where verified |
|---|---|---|
| Controller version | `reactive_v2.py` compared with `reactive_v1.py` | Controller files |
| Seeds | 1, 2, 3, 4, 5 | Test commands |
| Race count | 1 per seed | `--races 1` |
| Round duration | 30.0 seconds | `--round-seconds 30` and race output |
| Timestep | 0.016666666666666666 seconds | Race output and `GameConfig` |
| Race rules | Team-sum; win margin 1.0 m; marshal disabled | `HeadToHeadRaceRules`, `--no-marshal`, race output |

## Parameters

| Parameter | Value | Purpose |
|---|---:|---|
| `LOOKAHEAD_CURVE_THRESHOLD` | 0.18 | Starts moderate predictive slowing |
| `LOOKAHEAD_SHARP_CURVE_THRESHOLD` | 0.42 | Starts sharp predictive slowing |
| `LOOKAHEAD_CURVE_THROTTLE` | 0.30 | Moderate-corner throttle cap |
| `LOOKAHEAD_SHARP_CURVE_THROTTLE` | 0.16 | Sharp-corner throttle cap |

All other steering, wall, and speed parameters were kept equal to v1.

## Five-seed test results

| Version | Seed | Laps | Distance (m) | Best lap (s) | Max speed (m/s) | Damage | Wall contact (s) | Off-track (s) | Result |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| v2 | 1 | 2 | 394.73 | 15.70 | 17.88 | 0.0098 | 0.083 | 0.500 | Lost |
| v2 | 2 | 2 | 392.73 | 15.90 | 17.99 | 0.0096 | 0.100 | 0.433 | Won |
| v2 | 3 | 2 | 401.12 | 15.50 | 17.94 | 0.0092 | 0.083 | 0.417 | Won |
| v2 | 4 | 2 | 355.85 | 18.52 | 17.96 | 0.0117 | 0.083 | 0.350 | Lost |
| v2 | 5 | 2 | 380.53 | 16.77 | 17.94 | 0.0076 | 0.083 | 0.400 | Lost |

The v2 results were not identical to v1. They show lower distance and slower best laps in the direct comparison, while wall contact was equal or lower and off-track time was lower on seeds 2, 4, and 5.
