"""IMU data validation for HITL tests."""

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from tools.hitl.imu_device import ImuSample


@dataclass
class ValidationResult:
    """Result of validating one or more IMU samples."""

    passed: bool
    message: str
    details: dict[str, Any] | None = None


class ImuValidator:
    """Validates IMU data against expectations or invariants."""

    def validate_sample(self, sample: "ImuSample") -> ValidationResult:
        """Validate a single IMU sample.

        Args:
            sample: The sample to validate.

        Returns:
            ValidationResult with pass/fail and message.
        """
        raise NotImplementedError

    def validate_samples(self, samples: list["ImuSample"]) -> ValidationResult:
        """Validate a sequence of IMU samples (e.g. range, consistency).

        Args:
            samples: List of samples to validate.

        Returns:
            Aggregate ValidationResult.
        """
        raise NotImplementedError
