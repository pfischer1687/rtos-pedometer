"""IMU device abstraction for HITL tests."""

from __future__ import annotations

import logging
from dataclasses import dataclass
import time
from tools.hitl.protocol import HitlProtocolClient, ProtocolError
from tools.common.logging import HITL_LOGGER_NAME

logger = logging.getLogger(HITL_LOGGER_NAME)

DEFAULT_CMD_TIMEOUT_S = 5.0
DEFAULT_READ_N_TIMEOUT_S = 30.0


@dataclass
class ImuSample:
    """A single IMU sample from the device."""

    index: int
    ax: int
    ay: int
    az: int
    timestamp_us: int


class ImuDevice:
    """Wraps HitlProtocolClient with IMU-specific commands and READ_N parsing."""

    def __init__(self, protocol: HitlProtocolClient) -> None:
        """Initialize IMU device with a protocol client.

        Args:
            protocol: HitlProtocolClient instance for sending/receiving.
        """
        self._protocol = protocol

    def ping(self, timeout_s: float = DEFAULT_CMD_TIMEOUT_S) -> None:
        """Sends `PING` and expects `PONG`."""
        self._protocol.command_expect("PING", "PONG", timeout_s)

    def init(self, timeout_s: float = DEFAULT_CMD_TIMEOUT_S) -> None:
        """Sends `INIT` and expects `IMU_INIT_OK`."""
        self._protocol.command_expect("INIT", "IMU_INIT_OK", timeout_s)

    def configure(self, timeout_s: float = DEFAULT_CMD_TIMEOUT_S) -> None:
        """Sends `CONFIGURE` and expects `IMU_CONFIG_OK`."""
        self._protocol.command_expect("CONFIGURE", "IMU_CONFIG_OK", timeout_s)

    def start(self, timeout_s: float = DEFAULT_CMD_TIMEOUT_S) -> None:
        """Sends `START` and expects `IMU_START_OK`."""
        self._protocol.command_expect("START", "IMU_START_OK", timeout_s)

    def stop(self, timeout_s: float = DEFAULT_CMD_TIMEOUT_S) -> None:
        """Sends `STOP` and expects `IMU_STOP_OK`."""
        self._protocol.command_expect("STOP", "IMU_STOP_OK", timeout_s)

    def read_n(
        self,
        n: int = 10,
        timeout_s: float = DEFAULT_READ_N_TIMEOUT_S,
    ) -> list[ImuSample]:
        """Requests `n` samples and returns them as a list of `ImuSample` objects.

        SAMPLE format: "SAMPLE i ax ay az timestamp" (space-separated integers).

        Args:
            n: Number of samples to read (default: 10).
            timeout_s: Maximum seconds for the whole READ_N sequence.

        Returns:
            List of `n` `ImuSample` objects in order.
        """
        if n <= 0:
            raise ValueError("n must be positive")

        self._protocol.command_expect(f"READ_N_BYTES {n}", f"READ_START {n}", timeout_s)

        deadline = time.monotonic() + timeout_s
        samples: list[ImuSample] = []
        for i in range(n):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ProtocolError(f"Timeout while reading samples ({i}/{n})")

            while True:
                line = self._protocol.read_line(timeout_s=remaining)

                if line.startswith("ECHO:") or line.startswith(">"):
                    continue

                if line == "READ_TIMEOUT":
                    raise ProtocolError(
                        f"Device reported READ_TIMEOUT at sample {i}/{n}"
                    )

                if line == "READ_FAIL":
                    raise ProtocolError(f"Device reported READ_FAIL at sample {i}/{n}")

                try:
                    sample = _parse_sample_line(line, expected_index=i)
                    break
                except ProtocolError:
                    logger.debug("Skipping non-sample line: %r", line)

            samples.append(sample)
            logger.debug("SAMPLE[%d]: %s", i, sample)

        remaining = deadline - time.monotonic()
        self._protocol.expect_line("READ_DONE", remaining)

        return samples

    def reset(self, timeout_s: float = DEFAULT_CMD_TIMEOUT_S) -> None:
        """Sends `RESET` and expects `IMU_RESET_OK`."""
        self._protocol.command_expect("RESET", "IMU_RESET_OK", timeout_s)


def _parse_sample_line(line: str, expected_index: int) -> ImuSample:
    """Parses a single "SAMPLE i ax ay az timestamp" line.

    Args:
        line: Raw line (stripped of CR/LF).
        expected_index: Expected value of i (must match).

    Returns:
        ImuSample with index, ax, ay, az, timestamp_us.

    Raises:
        ProtocolError: Wrong format, parse failure, or index mismatch.
    """
    parts = line.split()
    if len(parts) != 6:
        raise ProtocolError(
            f"Malformed SAMPLE line: expected 6 fields (SAMPLE i ax ay az timestamp_us), "
            f"got {len(parts)}: {line!r}"
        )
    if parts[0] != "SAMPLE":
        raise ProtocolError(
            f"Malformed SAMPLE line: expected leading 'SAMPLE', got {parts[0]!r}"
        )

    try:
        index = int(parts[1])
        ax = int(parts[2])
        ay = int(parts[3])
        az = int(parts[4])
        timestamp_us = int(parts[5])
    except (ValueError, IndexError) as e:
        raise ProtocolError(
            f"Malformed SAMPLE line: could not parse integers: {line!r}"
        ) from e

    if index != expected_index:
        raise ProtocolError(
            f"SAMPLE index mismatch: expected {expected_index}, got {index}"
        )

    return ImuSample(
        index=index,
        ax=ax,
        ay=ay,
        az=az,
        timestamp_us=timestamp_us,
    )
