## Laboratory Notebook 9

**Date and time:** 9/7/26

**Participants and contributions:** Yewon

## Question or objective

Can an auditable outside-inside-outside planner, derived only from public local sensors, improve on the corrected
`centerline_v4` controller while remaining fully on track and avoiding damage, wall contact, reverse, and stops?

## Hypothesis

The relative strength of far and near lookahead bends can identify corner entry, apex, and exit. A bounded lateral
target should reduce effective corner radius, and staged CMA-ES should first improve the authored planner and then
recover the best joint planner/control balance.

## Initial experiment

Create three preserved artifacts. `racing_line_v1` uses authored planner parameters over frozen `centerline_v4`
expert gains. `racing_line_v2` tunes only ten planner parameters. `racing_line_v3` jointly tunes those ten parameters
and the 13 expert parameters. Use fresh training, development, and untouched promotion seeds. Opponents and private
track geometry are excluded.

## What we investigated or changed

- **AI-agent assistance:** Designed and implemented the structured planner, staged and resumable optimizer,
  diagnostics, promotion gates, export checks, tests, experiment recovery, and this experiment record.
- **Sign convention:** Positive desired lateral offset means right of the centerline. The expert follows
  `center_offset + desired_lateral_offset`.
- **Planner:** Far-dominant bend indicates entry and selects the outside; balanced bend selects the inside apex;
  near-dominant bend indicates exit and selects the outside. Conflicting S-bend signals fall back toward center.
- **Files:** Added the shared `controllers.racing_line` planner/parameter package, three thin controller entry points,
  racing-line fitness and promotion logic, staged training and final-audit scripts, and focused runtime, parameter,
  gate, resume, and export tests. Extended the centerline controller and expert with an optional line target while
  retaining the literal legacy zero-offset path when no planner is installed.
- **Planner configuration:** Authored values were maximum offset 1.20 m, bend activation 0.08, saturation width 0.55,
  phase gain 3.0, entry/apex/exit fractions 0.80/1.00/0.65, filter time 0.15 s, offset rate 3.0 m/s, and wall margin
  1.60 m. The established 1.35 m danger and 0.72 m emergency thresholds and forward-only brake adapter stayed fixed.

## Evidence

- **Seed partitions:** Training 19/53/97/149/269/401/587; development 631/743/857/947/1063; untouched promotion
  1171/1289/1423/1559/1693; final broad 3/29/61/107/173/257/349/433/547/661/773/997/1109/1223/1301; official
  110/2026. All trials lasted 30 simulated seconds.
- **Stage 1:** Population 16, 15 generations, sigma 0.25, optimizer seed 2026090701. The generation-15 champion,
  checksum `ec3dae56de3c82b8`, scored 689.6371 on training and passed development with 560.83 m mean and 559.68 m
  worst-seed distance. It was frozen as `racing_line_v2`.
- **Stage 2:** Population 24, 20 generations, sigma 0.20, optimizer seed 2026090702. The generation-20 champion,
  checksum `41d7bd13306d062a`, scored 696.3887 on training and passed development with 565.29 m mean and 563.96 m
  worst-seed distance. Its exact ten planner and thirteen expert parameters were frozen in Python for
  `racing_line_v3`.
- **Untouched promotion:** `racing_line_v3` was clean on all five seeds and averaged 564.41 m, versus 556.20 m for
  `centerline_v4` and 453.20 m for `racing_line_v2`. The planner-only controller exposed a severe seed-1289 failure
  (21.84 m, wall contact, damage, off-track, and a non-emergency stop); the joint candidate did not.
- **Artifacts:** Complete vectors, per-seed trials, generation summaries, configurations, optimizer states, promotion
  results, and final audits are under `artifacts/racing-line-cmaes/`. The raw concurrent log is retained as
  `joint/metrics.concurrent-raw.jsonl`; the canonical `metrics.jsonl` contains one record for each generation.
- **Commands:** `uv run python scripts/train_racing_line_cmaes.py --stage planner`,
  `uv run python scripts/train_racing_line_cmaes.py --stage joint`, the recovery command
  `uv run python scripts/train_racing_line_cmaes.py --stage joint --resume`,
  `uv run python scripts/evaluate_racing_line.py`, and selective export through
  `scripts/export_student_controllers.py controllers.racing_line_v3`.
- **Verification:** Strict Pyright reported zero errors. All 161 runnable repository tests passed; the existing
  Linux-specific bash startup diagnostic was deselected on Windows. All 17 changed Python files passed scoped Ruff
  lint and format checks. A repository-wide Ruff audit also identified eight unrelated, pre-existing import-order,
  line-length, and comparison-style findings in graphics/capture files and 17 unrelated files that do not currently
  match Ruff formatting; these were not rewritten as part of the controller artifact.

## What we observed

The authored planner was slower and less robust than the corrected centerline baseline. Planner-only CMA-ES found a
small development improvement but still failed badly on untouched seed 1289. Joint tuning changed the useful
steering/speed balance and removed that failure while improving every promotion aggregate.

The initial in-process training approach accumulated approximately 911 MiB because repeated Panda evaluator resets
did not return all memory. Candidate evaluation was therefore isolated in fresh worker processes. A user interruption
occurred after the joint generation-10 validation; the optimizer checkpoint later reached generation 19 in the
background. An explicit resume also evaluated generation 20, resulting in two identical deterministic copies of that
generation and two executions of the untouched gate. No parameter selection used the duplicate. The unmodified raw
log is preserved, the canonical log was deduplicated, and this deviation is recorded rather than represented as a
single untouched execution.

Across the 15 broad and two official seeds, `racing_line_v3` completed at least three laps in all 17 runs with exactly
zero damage, wall contact, off-track time, reverse time, and non-emergency stops. It averaged 565.18 m and its mean
best lap was 9.467 seconds. It beat `centerline_v4` on every seed by at least 6.04 m (7.41 m mean) and beat
`racing_line_v2` on every seed by at least 3.48 m (4.65 m mean). `racing_line_v2` averaged 560.53 m but accumulated
0.30 seconds off track on seed 1109;
the authored v1 averaged 554.12 m and contacted the wall on seed 1109. The known `centerline_v4` seed-1109 result was
reproduced: 0.0205 damage, 0.0167 seconds contact, and 0.40 seconds off track.

Repeated seed-631 evaluations of `racing_line_v3` produced identical metric dictionaries (563.9593 m, 9.467-second
best lap). After 1,800 calls, its controller process used 42.57 MiB resident memory. Selective export produced an
11,527-byte archive containing only the entry point and required Python controller packages; inference imports no CMA
or simulator experiment modules.

## Decision and rationale

Promote the generation-20 joint candidate as `racing_line_v3`. It clears every predefined safety and pace gate,
generalizes across the final broad and official audits, and materially repairs the planner-only seed-1289 failure.
Keep v1 and v2 as authored and planner-only ablations. Do not tune against seed 1109 or any final-audit observation.
Keep `formula110-submission.json` on `controllers.cmaes_v2`; racing-line promotion does not alter the submitted entry
point without a separate request.

## Next steps

1. Manually inspect representative `racing_line_v3` runs for qualitative outside-inside-outside behavior.
2. Compare `racing_line_v3` against the learned MLP CMA-ES artifact as a separate experiment if desired.
3. Change the submission entry point only after an explicit promotion decision.

## Overall

- **Implementation and staged tuning complete; `racing_line_v3` promoted within the artifact family**
