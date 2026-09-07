"""Generation-17 CMA-ES parameters for corrected centerline braking."""

from typing import Final

from controllers.centerline.parameters import CenterlineParameters

BRAKING_TUNED_PARAMETERS: Final = CenterlineParameters(
    center_gain=0.0853375606307272,
    heading_gain=0.007385020763846642,
    bend_gain=1.2490074266695563,
    bend_change_gain=2.117096548828603,
    yaw_damping=0.0007484209107116637,
    straight_target_speed_mps=19.94524191970028,
    corner_speed_reduction_mps=3.5383559215969864,
    center_speed_penalty=0.019757932597233907,
    heading_speed_penalty=0.09922902185285526,
    acceleration_gain=0.3069096730211081,
    braking_gain=0.35039444955967103,
    steering_smoothing_s=0.02546183296624531,
    throttle_smoothing_s=0.1516569717182138,
)
TUNING_STATUS: Final = "generation-17-held-out-passed"
