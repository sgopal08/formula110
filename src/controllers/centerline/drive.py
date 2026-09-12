"""Forward-only translation of brake intent into public drive commands."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Literal

from racing import RobotCommand

BRAKE_CUTOFF_MPS: Final = 0.5
DriveEvent = Literal["pass_through", "brake_pulse", "neutral_release", "low_speed_hold"]


@dataclass(frozen=True, slots=True)
class DriveAdapterDecision:
    """Physical command and the interlock action used to produce it."""

    command: RobotCommand
    event: DriveEvent


class ForwardOnlyDriveAdapter:
    """Apply brake intent without allowing Bullet's reverse transition to latch."""

    def __init__(self) -> None:
        self._neutral_release_required = False

    def reset(self) -> None:
        self._neutral_release_required = False

    def apply(self, *, speed_mps: float, command: RobotCommand) -> DriveAdapterDecision:
        if self._neutral_release_required:
            self._neutral_release_required = False
            return DriveAdapterDecision(RobotCommand(throttle=0.0, steer=command.steer), "neutral_release")

        if command.throttle < 0.0:
            if speed_mps <= BRAKE_CUTOFF_MPS:
                return DriveAdapterDecision(RobotCommand(throttle=0.0, steer=command.steer), "low_speed_hold")
            self._neutral_release_required = True
            return DriveAdapterDecision(command, "brake_pulse")

        return DriveAdapterDecision(command, "pass_through")
