"""CMA-ES-tuned centerline controller with forward-only braking semantics."""

from controllers.centerline import CenterlineController
from controllers.centerline.braking_tuned_parameters import BRAKING_TUNED_PARAMETERS

RACING_NAME: str = "Centerline v4 Brake-Tuned"
RACING_COLOR: str = "#D33682"


def create_controller() -> CenterlineController:
    return CenterlineController(BRAKING_TUNED_PARAMETERS, forward_only_braking=True)
