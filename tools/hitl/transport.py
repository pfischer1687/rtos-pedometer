"""Serial transport layer for HITL communication."""

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from tools.hitl.protocol import SerialTransport


class SerialTransport:
    """Low-level serial I/O for communicating with the target device."""

    def __init__(self, port: str = "", baud_rate: int = 115200) -> None:
        """Initialize transport with serial connection parameters.

        Args:
            port: Serial port path (e.g. COM3, /dev/ttyUSB0).
            baud_rate: Baud rate for the connection.
        """
        self._port = port
        self._baud_rate = baud_rate

    def open(self) -> None:
        """Open the serial connection."""
        raise NotImplementedError

    def close(self) -> None:
        """Close the serial connection."""
        raise NotImplementedError

    def send(self, data: bytes) -> int:
        """Send raw bytes to the device.

        Args:
            data: Bytes to send.

        Returns:
            Number of bytes written.
        """
        raise NotImplementedError

    def receive(self, size: int = 1, timeout: float | None = None) -> bytes:
        """Receive raw bytes from the device.

        Args:
            size: Maximum number of bytes to read.
            timeout: Read timeout in seconds; None for blocking.

        Returns:
            Bytes received (may be fewer than size if timeout or EOF).
        """
        raise NotImplementedError

    def __enter__(self) -> "SerialTransport":
        """Context manager entry."""
        self.open()
        return self

    def __exit__(self, *args: Any) -> None:
        """Context manager exit."""
        self.close()
