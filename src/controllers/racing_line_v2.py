"""Planner-only CMA-ES structured racing-line controller."""

from controllers.centerline import CenterlineController
from controllers.centerline.braking_tuned_parameters import BRAKING_TUNED_PARAMETERS
from controllers.racing_line import RacingLinePlanner
from controllers.racing_line.planner_tuned_parameters import PLANNER_TUNED_PARAMETERS

RACING_NAME: str = "Racing Line v2 Planner-Tuned"
RACING_COLOR: str = "#B58900"


def create_controller() -> CenterlineController:
    return CenterlineController(
        BRAKING_TUNED_PARAMETERS,
        forward_only_braking=True,
        line_planner=RacingLinePlanner(PLANNER_TUNED_PARAMETERS),
    )
