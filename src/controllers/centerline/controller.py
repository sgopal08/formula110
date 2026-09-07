"""Composition root for the stateful centerline controller."""

from __future__ import annotations

from controllers.centerline.drive import DriveAdapterDecision, ForwardOnlyDriveAdapter
from controllers.centerline.parameters import CenterlineParameters
from controllers.centerline.policy import CenterlineExpert, ExpertDecision, SafetyFilter
from controllers.centerline.state import CenterlineState, SensorTranslator
from racing import RobotCommand, RobotSensors


class CenterlineController:
    """Compose translation, nominal control, smoothing, and fixed safety."""

    def __init__(self, parameters: CenterlineParameters, *, forward_only_braking: bool = False) -> None:
        self.parameters = parameters
        self.forward_only_braking = forward_only_braking
        self.translator = SensorTranslator()
        self.expert = CenterlineExpert(parameters)
        self.safety = SafetyFilter(emergency_brake_intent=-1.0 if forward_only_braking else -0.2)
        self.drive_adapter = ForwardOnlyDriveAdapter() if forward_only_braking else None
        self.last_state: CenterlineState | None = None
        self.last_decision: ExpertDecision | None = None
        self.last_drive_decision: DriveAdapterDecision | None = None
        self._previous_command = RobotCommand()

    def __call__(self, sensors: RobotSensors) -> RobotCommand:
        state = self.translator.update(sensors)
        if self.last_state is not None and state.tick <= self.last_state.tick:
            self._previous_command = RobotCommand()
            if self.drive_adapter is not None:
                self.drive_adapter.reset()
        decision = self.expert.decide(state)
        steering_alpha = state.dt_s / (self.parameters.steering_smoothing_s + state.dt_s)
        throttle_alpha = state.dt_s / (self.parameters.throttle_smoothing_s + state.dt_s)
        smoothed = RobotCommand(
            throttle=_blend(self._previous_command.throttle, decision.command.throttle, throttle_alpha),
            steer=_blend(self._previous_command.steer, decision.command.steer, steering_alpha),
        )
        safe_intent = self.safety.apply(state, smoothed)
        if self.drive_adapter is None:
            command = safe_intent
            self.last_drive_decision = None
            self._previous_command = command
        else:
            self.last_drive_decision = self.drive_adapter.apply(speed_mps=state.speed_mps, command=safe_intent)
            command = self.last_drive_decision.command
            self._previous_command = safe_intent
        self.last_state = state
        self.last_decision = decision
        return command


def _blend(previous: float, current: float, alpha: float) -> float:
    return previous + alpha * (current - previous)
