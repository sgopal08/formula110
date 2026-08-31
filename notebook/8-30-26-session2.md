## Laboratory Notebook 2

**Date and time:** 8/30 8:00 PM

**Participants and contributions:** Yewon

## Question or objective

Develop a minimum viable reactive controller as the baseline for later CMA-ES optimization.

## Hypothesis

Local track and sensor observations are enough to determine an appropriate steering direction and throttle level without using a learned model. A simple reactive controller should be able to stay on the track by using nearby sensor information to estimate where the track is going and how sharply the car should turn.

## Initial experiment

Implement a basic reactive controller that uses sensor information to keep the car near the center of the track. Steering will be determined from the relative space or track position on the left and right sides of the car. Throttle will depend on steering intensity: the car will use higher throttle on straighter sections and reduce throttle during sharper turns.

The controller will then be evaluated on five fixed random seeds to measure whether it can drive consistently from different starting positions.

## What we investigated or changed

We investigated how a reactive controller could use the car’s available sensor information to make steering and throttle decisions without machine learning.

We identified two main behaviors for the first controller:

1. Maintaining a reasonable position within the track.
2. Adjusting speed based on how aggressively the car needs to steer.

We also decided to begin with a small number of manually chosen controller parameters, such as steering gain, turn thresholds, and throttle levels. These parameters provide a baseline that can later be optimized automatically using CMA-ES.

## Evidence

- **AI-agent assistance:** Implemented and tested a deterministic reactive controller using centerline geometry, lookahead, wall LiDAR, speed, and front-wall safety logic.
- **Commits or code:** Added `src/controllers/reactive_v1.py`. Commit hash: `076b018`.
- **Experiment configuration:** Shown in the Experiment Data with controller `reactive_v1`.
- **Experiment output:** Two laps completed on every seed; distances ranged from 386.49 m to 418.40 m; best lap times ranged from 14.45 s to 16.07 s; no eliminations; minor wall contact and damage occurred on all runs.

## What we observed

The reactive controller successfully completed every test run, showing that local sensor and track information was sufficient for basic, reliable driving without a learned model. The controller behaved consistently across all five starting seeds and maintained enough control to complete two laps each time.

However, every run included a small amount of wall contact, off-track time, and damage. This suggests that the controller can recover from dangerous situations, but its safety behavior is still mostly reactive. It often corrects after getting too close to a wall rather than anticipating the turn early enough to avoid the situation entirely.

Seed 3 produced the fastest best lap at **14.45 seconds**, while Seed 4 produced the slowest best lap at **16.07 seconds** and the greatest amount of off-track time at **0.600 seconds**. This shows that starting position still has some effect on performance, even though the controller completed every run.

No repeated manual parameter tuning was performed, so these results represent a genuine baseline rather than a heavily optimized controller.

## Decision and rationale

We decided to keep the reactive controller as the foundation of our approach because it demonstrated strong reliability across all five seeds. It completed every race, avoided elimination, and consistently defeated the `crash_fast` baseline.

The results support our original hypothesis that local track and sensor observations are sufficient to produce a functional controller without a learned model.

However, the remaining wall contact, off-track time, and variation in lap times indicate that the manually selected parameters are probably not optimal. Because the controller already works reliably, it provides a good foundation for automated parameter optimization.

For this reason, we will keep the reactive controller architecture and later use CMA-ES to search for better values for parameters such as steering gain, throttle levels, turn thresholds, target speed, and wall-avoidance thresholds. This should allow us to improve speed and consistency while preserving the reliability of the current controller.

## Next steps

The current controller and its five-seed results will be preserved as the baseline so that future CMA-ES versions can be compared directly against it.

1. Reduce wall contact and off-track time.
2. Reduce unnecessary steering corrections.
3. Improve corner entry and exit speed.
4. Improve lap-time consistency across different seeds.
5. Increase overall race speed without reducing completion reliability.

## Experiment Data
**Controller:** `reactive_v1`  
**Seeds:** 1, 2, 3, 4, 5

## Parameters

| Parameter | Value |
|---|---:|
| `STEERING_GAIN` | `0.22` |
| `HEADING_GAIN` | `0.012` |
| `EMERGENCY_CORRECTION_GAIN` | `0.75` |
| `STRAIGHT_THROTTLE` | `0.62` |
| `MODERATE_TURN_THROTTLE` | `0.38` |
| `SHARP_TURN_THROTTLE` | `0.18` |
| `MODERATE_TURN_THRESHOLD` | `0.28` |
| `SHARP_TURN_THRESHOLD` | `0.58` |
| `TARGET_SPEED_MPS` | `5.5` |
| `WALL_DANGER_THRESHOLD_M` | `1.35` |
| `WALL_EMERGENCY_THRESHOLD_M` | `0.72` |

## Results

| Seed | Laps | Distance (m) | Best Lap (s) |
|---:|---:|---:|---:|
| 1 | 2 | 403.13 | 15.45 |
| 2 | 2 | 399.12 | 15.43 |
| 3 | 2 | 418.40 | 14.45 |
| 4 | 2 | 386.49 | 16.07 |
| 5 | 2 | 398.48 | 15.40 |

## Overall

- **5/5 completed**
- **0 eliminations**
- **5/5 wins vs `crash_fast`**
- Small wall contact and minor damage occurred on every run.
"""