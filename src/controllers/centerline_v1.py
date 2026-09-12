"""Stateful, hand-authored centerline controller artifact."""

from controllers.centerline import AUTHORED_PARAMETERS, CenterlineController

RACING_NAME: str = "Centerline v1 Authored"
RACING_COLOR: str = "#35B779"


def create_controller() -> CenterlineController:
    return CenterlineController(AUTHORED_PARAMETERS)
