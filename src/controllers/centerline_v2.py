"""Stateful centerline controller with CMA-ES-tuned expert gains."""

from controllers.centerline import CenterlineController
from controllers.centerline.tuned_parameters import TUNED_PARAMETERS

RACING_NAME: str = "Centerline v2 Tuned"
RACING_COLOR: str = "#31688E"


def create_controller() -> CenterlineController:
    return CenterlineController(TUNED_PARAMETERS)
