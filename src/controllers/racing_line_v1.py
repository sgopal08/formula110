"""Hand-authored structured racing-line controller."""

from controllers.centerline import CenterlineController
from controllers.centerline.braking_tuned_parameters import BRAKING_TUNED_PARAMETERS
from controllers.racing_line import AUTHORED_RACING_LINE_PARAMETERS, RacingLinePlanner

RACING_NAME: str = "Racing Line v1 Authored"
RACING_COLOR: str = "#6C71C4"


def create_controller() -> CenterlineController:
    return CenterlineController(
        BRAKING_TUNED_PARAMETERS,
        forward_only_braking=True,
        line_planner=RacingLinePlanner(AUTHORED_RACING_LINE_PARAMETERS),
    )
