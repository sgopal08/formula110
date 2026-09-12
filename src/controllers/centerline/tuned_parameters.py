"""Generation-20 CMA-ES parameters that passed the held-out promotion gate."""

from typing import Final

from controllers.centerline.parameters import CenterlineParameters

TUNED_PARAMETERS: Final = CenterlineParameters(
    center_gain=0.1285754215230586,
    heading_gain=0.01642685761748922,
    bend_gain=1.0737847135364103,
    bend_change_gain=2.320605969191592,
    yaw_damping=0.0006863064515836311,
    straight_target_speed_mps=19.95764894882859,
    corner_speed_reduction_mps=3.0885041321908258,
    center_speed_penalty=0.3171417304056936,
    heading_speed_penalty=0.08137344861863965,
    acceleration_gain=0.15654044599167152,
    braking_gain=0.3106933085725701,
    steering_smoothing_s=0.03854889284164696,
    throttle_smoothing_s=0.1375779909777321,
)
TUNING_STATUS: Final = "generation-20-held-out-passed"
