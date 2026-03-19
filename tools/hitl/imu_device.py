"""IMU device abstraction for HITL tests."""

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from tools.hitl.protocol import HitlProtocolClient


@dataclass
class ImuSample:
    """A single IMU sample."""

    accel_x: int
    accel_y: int
    accel_z: int
    timestamp_us: int = 0


class ImuDevice:
    """Abstraction for the IMU on the target device."""

    def __init__(self, protocol: "HitlProtocolClient") -> None:
        """Initialize IMU device with a protocol client.

        Args:
            protocol: HitlProtocolClient instance for sending/receiving.
        """
        self._protocol = protocol

    def read_sample(self, timeout: float | None = None) -> ImuSample:
        """Read a single IMU sample from the device.

        Args:
            timeout: Optional timeout in seconds.

        Returns:
            Parsed ImuSample.
        """
        raise NotImplementedError

    def read_samples(self, count: int, timeout: float | None = None) -> list[ImuSample]:
        """Read multiple IMU samples.

        Args:
            count: Number of samples to read.
            timeout: Optional timeout for the whole operation.

        Returns:
            List of ImuSample.
        """
        raise NotImplementedError
