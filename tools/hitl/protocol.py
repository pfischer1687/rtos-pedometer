"""HITL protocol client for command/response framing."""

from typing import Any
from tools.hitl.transport import SerialTransport


class HitlProtocolClient:
    """Client for the HITL wire protocol."""

    def __init__(self, transport: SerialTransport) -> None:
        """Initialize protocol client with a transport instance.

        Args:
            transport: SerialTransport instance for I/O.
        """
        self._transport = transport

    def send_command(self, cmd: bytes | str, *args: Any) -> None:
        """Send a protocol command to the device.

        Args:
            cmd: Command identifier (bytes or string).
            *args: Optional payload or parameters.
        """
        raise NotImplementedError

    def receive_response(self, timeout: float | None = None) -> bytes | dict[str, Any]:
        """Receive and parse a protocol response.

        Args:
            timeout: Optional timeout in seconds.

        Returns:
            Raw response bytes or parsed response dict.
        """
        raise NotImplementedError

    def exchange(self, cmd: bytes | str, *args: Any) -> bytes | dict[str, Any]:
        """Send command and wait for response (request-response round trip).

        Args:
            cmd: Command identifier.
            *args: Optional payload.

        Returns:
            Response from the device.
        """
        raise NotImplementedError
