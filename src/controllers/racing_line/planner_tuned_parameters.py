"""Planner-only parameters promoted from the 2026-09-07 CMA-ES run."""

from typing import Final

from controllers.racing_line.parameters import RacingLineParameters

PLANNER_TUNED_PARAMETERS: Final[RacingLineParameters] = RacingLineParameters(
    max_offset_m=1.401060964334048,
    bend_activation=0.18127306412980426,
    bend_saturation_width=0.3414921674269415,
    phase_gain=0.8535741090393517,
    entry_outside_fraction=0.013077690542242504,
    apex_inside_fraction=0.9102711318548435,
    exit_outside_fraction=1.0709132837498136,
    offset_filter_time_constant_s=0.06654149572698892,
    max_offset_rate_mps=4.226890078941478,
    wall_margin_m=1.501319062056659,
)
PLANNER_TUNING_STATUS: Final = "promoted; generation 15; checksum ec3dae56de3c82b8"
