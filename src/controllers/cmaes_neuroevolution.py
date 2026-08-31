"""Deployable Formula 110 controller trained by CMA-ES neuroevolution."""

from __future__ import annotations

import json
from pathlib import Path
from typing import cast

from controllers.cmaes_policy import FixedMLPPolicy, initial_parameters

RACING_NAME: str = "CMA-ES Neuroevolution"
RACING_COLOR: str = "#A855F7"


def _load_parameters() -> tuple[float, ...]:
    artifact_path = Path(__file__).with_name("cmaes_weights.json")
    if not artifact_path.exists():
        return initial_parameters()
    payload = cast(dict[str, object], json.loads(artifact_path.read_text(encoding="utf-8")))
    raw_parameters = cast(list[float], payload["parameters"])
    return tuple(float(value) for value in raw_parameters)


def create_controller() -> FixedMLPPolicy:
    """Create an independent policy instance for each race car and trial."""
    return FixedMLPPolicy(_load_parameters())
