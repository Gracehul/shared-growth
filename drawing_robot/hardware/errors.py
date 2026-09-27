"""Typed Stage-4A hardware failures."""


class HardwareIntegrationError(RuntimeError):
    code = "BACKEND_API_ERROR"


class PreflightRejected(HardwareIntegrationError):
    code = "PREFLIGHT_REJECTED"

    def __init__(self, message, gate_results=()):
        super().__init__(message)
        self.gate_results = tuple(gate_results)


class StaleTelemetry(HardwareIntegrationError):
    code = "STALE_TELEMETRY"


class TemperatureAbort(HardwareIntegrationError):
    code = "TEMPERATURE_ABORT"


class StopFailed(HardwareIntegrationError):
    code = "STOP_FAILED"


class CommunicationLost(HardwareIntegrationError):
    code = "COMMUNICATION_LOST"
    robot_motion_unknown = True
    stop_confirmed = False
    manual_intervention_required = True
