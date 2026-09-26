"""Configuration for the standalone camera baseline."""

from dataclasses import dataclass


@dataclass(frozen=True)
class CameraConfig:
    device: str = "/dev/video0"
    width: int = 640
    height: int = 480
    requested_fps: float = 30.0
    pixel_format: str = "MJPG"
    duration_s: float = 5.0
    preview: bool = False

    def validate(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera dimensions must be positive")
        if self.requested_fps <= 0:
            raise ValueError("requested_fps must be positive")
        if self.duration_s <= 0:
            raise ValueError("duration_s must be positive")
        if len(self.pixel_format) != 4:
            raise ValueError("pixel_format must be a four-character code")
