"""IMU data validation for HITL tests."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from tools.hitl.imu_device import ImuSample

# MPU-6050 ±2g mode: 16384 LSB/g
LSB_PER_G = 16384.0

# Standard gravity in m/s²
G_MPS2 = 9.80665

# 10% tolerance
G_TOL_PCT = 0.1
G_TOL_MPS2 = G_TOL_PCT * G_MPS2


class ValidationError(Exception):
    """Raised when IMU sample validation fails."""

    pass


class ImuValidator:
    """Validates IMU sample integrity and physical correctness (e.g. gravity magnitude)."""

    def validate(self, samples: list["ImuSample"]) -> None:
        """Validate a list of IMU samples.

        Checks (in order):
        1. Non-empty list.
        2. Indices strictly increasing from 0 (0, 1, 2, ...).
        3. Timestamps strictly increasing.
        4. Gravity magnitude for each sample within ±5% of 9.80665 m/s²

        Args:
            samples: List of ImuSample to validate.

        Raises:
            ValidationError: On any failed check, with a clear message.
        """
        if not samples:
            raise ValidationError("Sample list is empty")

        for i, s in enumerate(samples):
            if s.index != i:
                raise ValidationError(
                    f"Index mismatch at position {i}: expected index {i}, got {s.index}"
                )

        for i in range(1, len(samples)):
            if samples[i].timestamp_us <= samples[i - 1].timestamp_us:
                raise ValidationError(
                    f"Timestamps not strictly increasing at index {i}: "
                    f"{samples[i - 1].timestamp_us} -> {samples[i].timestamp_us}"
                )

        for i, s in enumerate(samples):
            ax_mps2 = (s.ax / LSB_PER_G) * G_MPS2
            ay_mps2 = (s.ay / LSB_PER_G) * G_MPS2
            az_mps2 = (s.az / LSB_PER_G) * G_MPS2

            magnitude_mps2 = math.sqrt(
                ax_mps2 * ax_mps2 + ay_mps2 * ay_mps2 + az_mps2 * az_mps2
            )

            if (
                magnitude_mps2 < G_MPS2 - G_TOL_MPS2
                or magnitude_mps2 > G_MPS2 + G_TOL_MPS2
            ):
                raise ValidationError(
                    f"Gravity magnitude out of range at index {i}: "
                    f"got {magnitude_mps2:.4f} m/s², expected [{G_MPS2 - G_TOL_MPS2:.4f}, {G_MPS2 + G_TOL_MPS2:.4f}]"
                )
