"""Jointly CMA-ES-tuned structured racing-line controller."""

from controllers.centerline import CenterlineController
from controllers.racing_line import RacingLinePlanner
from controllers.racing_line.joint_tuned_parameters import JOINT_EXPERT_PARAMETERS, JOINT_PLANNER_PARAMETERS

RACING_NAME: str = "Racing Line v3 Joint-Tuned"
RACING_COLOR: str = "#CB4B16"


def create_controller() -> CenterlineController:
    return CenterlineController(
        JOINT_EXPERT_PARAMETERS,
        forward_only_braking=True,
        line_planner=RacingLinePlanner(JOINT_PLANNER_PARAMETERS),
    )
