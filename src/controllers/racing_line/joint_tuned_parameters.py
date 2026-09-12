"""Joint parameters promoted through the untouched 2026-09-07 gate."""

from typing import Final

from controllers.centerline.parameters import CenterlineParameters
from controllers.racing_line.parameters import RacingLineParameters

JOINT_PLANNER_PARAMETERS: Final[RacingLineParameters] = RacingLineParameters(
    max_offset_m=1.4450062976519984,
    bend_activation=0.1300553033653349,
    bend_saturation_width=0.3356651147105859,
    phase_gain=0.7988796872146589,
    entry_outside_fraction=0.011789638852035544,
    apex_inside_fraction=1.045954133081367,
    exit_outside_fraction=1.210722227634817,
    offset_filter_time_constant_s=0.06252725839572756,
    max_offset_rate_mps=5.052097794224866,
    wall_margin_m=1.7359552732291368,
)
JOINT_EXPERT_PARAMETERS: Final[CenterlineParameters] = CenterlineParameters(
    center_gain=0.10221067916210759,
    heading_gain=0.004665506214453519,
    bend_gain=1.2173605686703213,
    bend_change_gain=2.4654140714599495,
    yaw_damping=0.0004943084742962383,
    straight_target_speed_mps=19.926528097402674,
    corner_speed_reduction_mps=4.606024007236412,
    center_speed_penalty=0.00366833864134386,
    heading_speed_penalty=0.05955102423807939,
    acceleration_gain=0.32683319985057824,
    braking_gain=0.3409731554466744,
    steering_smoothing_s=0.028753099964285045,
    throttle_smoothing_s=0.11110281350746443,
)
JOINT_TUNING_STATUS: Final = "promoted; generation 20; checksum 41d7bd13306d062a"
