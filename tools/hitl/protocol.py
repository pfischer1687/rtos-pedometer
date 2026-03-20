"""HITL protocol client for command/response framing."""

from __future__ import annotations

import logging
import time
from tools.hitl.transport import SerialTransport
from tools.common.logging import HITL_LOGGER_NAME

logger = logging.getLogger(HITL_LOGGER_NAME)


class ProtocolError(Exception):
    """Raised when a protocol response does not match expectations."""

    pass


class HitlProtocolClient:
    """Client for the HITL wire protocol (command/response lines)."""

    def __init__(self, transport: SerialTransport) -> None:
        """Initialize protocol client with a transport instance.

        Args:
            transport: SerialTransport instance for I/O.
        """
        self._transport = transport

    def send_command(self, cmd: str) -> None:
        """Send a protocol command to the device.

        Args:
            cmd: Command string to send (one line).
        """
        logger.debug("CMD: %s", cmd)
        self._transport.drain()
        self._transport.write_line(cmd)

    def read_line(self, timeout_s: float | None = None) -> str:
        """Read one line from the device.

        Args:
            timeout_s: Seconds to wait for a line; uses instance default if None.

        Returns:
            The next non-empty stripped line.
        """
        return self._transport.read_line(timeout_s=timeout_s)

    def expect_line(self, expected: str, timeout_s: float) -> None:
        """Wait for a line that exactly matches the expected string.

        Args:
            expected: Exact string the next line must equal (after strip).
            timeout_s: Maximum seconds to wait for a line.

        Raises:
            ProtocolError: If the received line does not match expected.
        """
        deadline = time.monotonic() + timeout_s
        last_line = ""

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ProtocolError(
                    f"Protocol mismatch: expected {expected!r}, got {last_line or 'timeout'}"
                )

            line = self.read_line(timeout_s=remaining)
            last_line = line

            if line.startswith("ECHO:"):
                logger.debug("Ignoring echo: %r", line)
                continue

            if line == ">":
                logger.debug("Ignoring prompt")
                continue

            logger.debug("RECV LINE: %r", line)

            if line == expected:
                return

            logger.debug("Skipping line while waiting for %r: %r", expected, line)

    def command_expect(self, cmd: str, expected: str, timeout_s: float) -> None:
        """Send a command and expect a specific response line.

        Args:
            cmd: Command string to send.
            expected: Exact string the response line must equal (after strip).
            timeout_s: Maximum seconds to wait for a line.
        """
        self.send_command(cmd)
        self.expect_line(expected, timeout_s)
