"""Fix-only centerline artifact with forward-only braking semantics."""

from controllers.centerline import CenterlineController
from controllers.centerline.tuned_parameters import TUNED_PARAMETERS

RACING_NAME: str = "Centerline v3 Brake Interlock"
RACING_COLOR: str = "#2AA198"


def create_controller() -> CenterlineController:
    return CenterlineController(TUNED_PARAMETERS, forward_only_braking=True)
