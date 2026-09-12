## Laboratory Notebook 8

**Date and time:** 9/6/26

**Participants and contributions:** Yewon

## Question or objective

Can a controller-side brake interlock remove the unintended full stops caused by signed drive-direction semantics,
while preserving centerline safety and improving on the tuned `centerline_v2` pace?

## Hypothesis

Isolated negative brake-intent pulses followed by mandatory neutral ticks will clear the simulator's pending direction
change before forward power resumes. Retuning the same 13 expert parameters should recover the best steering-speed
balance without permitting intentional reverse propulsion.

## Initial experiment

Preserve `centerline_v1` and `centerline_v2`. Create `centerline_v3` as a fix-only ablation using the exact `v2`
parameters, then tune the corrected architecture into `centerline_v4` using the original training/validation split and
CMA-ES budget. Official seeds remain final-audit-only.

## What we investigated or changed

- **AI-agent assistance:** Diagnosed the signed-throttle transition and implemented the planned braking experiment.
- **Runtime design:** Added an opt-in forward-only adapter after smoothing and safety. A negative brake-intent pulse is
  followed by a neutral release tick, and negative output is suppressed at or below 0.5 m/s.
- **Artifact lineage:** Legacy controller construction remains the default. Only `centerline_v3` and `centerline_v4`
  explicitly enable corrected semantics.

## Evidence

- **Files and implementation:** Added the forward-only adapter in `src/controllers/centerline/drive.py`, opt-in
  composition in the shared controller, preserved entry points `centerline_v3.py` and `centerline_v4.py`, braking
  diagnostics/fitness, separate training and evaluation scripts, and regression tests. The active submission remains
  `controllers.cmaes_v2`.
- **Experiment configuration:** CMA-ES population 16, 20 generations, sigma 0.25, optimizer seed 20260906, 30-second
  trials, training seeds 17/41/83/137/241/311/509, and validation seeds 613/719/823/929/1031. All 13 parameters were
  initialized from the promoted `centerline_v2` vector. Outputs are under `artifacts/centerline-braking-cmaes/`.
- **Experiment output:** Evaluated 320 candidates in 2,240 seeded training trials. The run took approximately 1,984
  seconds. Generation 17 individual 10, checksum `8c8dd8f42dd285da`, was the training champion with fitness
  `687.5380`; it passed the final generation-20 held-out gate.
- **Verification:** Strict Pyright reported zero errors. Scoped Ruff lint and format checks passed. The repository suite
  passed 138 tests with the previously documented Windows-only bash-startup diagnostic deselected. Deterministic
  repeated seed-3 evaluations produced identical dictionaries. Selective export produced an 8,118-byte archive.
  After 1,800 calls, the controller process used 42.58 MiB resident memory.

## What we observed

The fix-only `centerline_v3` completed two laps on every training and validation seed with zero damage, wall contact,
reverse time, or non-emergency stop episodes. Its validation mean rose from `centerline_v2`'s 435.13 meters to 512.59
meters without changing the learned parameters, demonstrating that the actuator-semantic correction—not retuning—was
responsible for the first large improvement.

The generation-17 `centerline_v4` champion passed all five held-out seeds with zero damage, wall contact, reverse
time, and non-emergency stop episodes. Validation mean distance was 557.13 meters, 28.0% above `centerline_v2`, and
best laps were approximately 9.57-9.58 seconds. It had no per-seed distance regression and only seed 613 recorded
off-track time (0.300 seconds total).

The exact diagnostic threshold is intentionally strict: three consecutive filtered readings below 1.2 m/s after
launch. The visually stop-like `centerline_v2` events did not always cross that exact boundary, so its diagnostic
episode count is zero even though its existing `low_progress_seconds` remained materially higher than corrected runs.
Corrected candidates consistently recorded only the one- or two-tick startup floor in the simulator metric.

Across 15 broad-validation seeds and official seeds 110/2026, `centerline_v4` completed at least two laps on all 17,
recorded zero reverse and non-emergency stop episodes, and averaged 557.77 meters. Broad seed 1109 exposed a small
negative result: 0.0167 seconds wall contact, 0.0205 damage, and 0.400 seconds off track. This occurred only after the
candidate was frozen and was not used for further tuning. Both official seeds remained clean, completed three laps,
covered 558.11 and 558.17 meters, and recorded 9.583-second best laps.

## Decision and rationale

Promote the generation-17 parameters into `centerline_v4` because they satisfy every predefined training and held-out
promotion gate and deterministically remove the unintended direction-transition stops. Preserve `centerline_v3` as
the safer fix-only ablation. Do not hide the seed-1109 broad-audit regression or tune against it after the frozen audit;
it should inform the later racing-line safety comparison.

## Next steps

1. Visually inspect `centerline_v3` and `centerline_v4` to confirm the stop behavior is absent in rendered runs.
2. Use `centerline_v3` as the conservative safety reference and `centerline_v4` as the pace reference for the
   racing-line artifact.
3. Treat broad seed 1109 as a frozen audit observation, not a new tuning seed.

## Overall

- **320 CMA-ES candidates and 2,240 seeded training trials completed**
- **Generation-17 champion passed the held-out promotion gate**
- **Validation mean: 557.13 m; best laps: approximately 9.57 s**
- **Zero validation damage, wall contact, reverse, and non-emergency stops**
- **17/17 broad/official runs completed; one small broad-seed contact documented**
- **Both official seeds clean with three laps completed**
- **Deterministic replay, 138 tests, Pyright, Ruff, export, and 42.58 MiB RSS checks passed**
