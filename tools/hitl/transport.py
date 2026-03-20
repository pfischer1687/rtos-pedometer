"""Serial transport layer for HITL communication."""

from __future__ import annotations

import logging
import time
from typing import Any
from tools.common.logging import HITL_LOGGER_NAME
import serial

logger = logging.getLogger(HITL_LOGGER_NAME)

READ_LINE_WAIT_TIME_S = 0.005
MAX_LINE_LENGTH = 4096
MIN_READ_LINE_WAIT_TIME_S = 0.001


class SerialTransport:
    """Low-level serial I/O for communicating with the target device."""

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        timeout_s: float = 5.0,
    ) -> None:
        """Initialize transport with serial connection parameters.

        Args:
            port: Serial port path (e.g. COM3, /dev/ttyUSB0).
            baud: Baud rate for the connection.
            timeout_s: Default read timeout in seconds for per-byte reads.
        """
        self._port = port
        self._baud = baud
        self._timeout_s = timeout_s
        self._ser: serial.Serial | None = None
        self._rx_buf = bytearray()

    def is_open(self) -> bool:
        """Check if the serial connection is open."""
        return self._ser is not None and self._ser.is_open

    def _ensure_open(self) -> serial.Serial:
        """Return the underlying serial instance.

        Raises:
            RuntimeError: If the serial port is not open.
        """
        if not self.is_open():
            raise RuntimeError("Serial port is not open")

        return self._ser  # type: ignore

    def open(self) -> None:
        """Open the serial connection.

        Raises:
            RuntimeError: If the serial port cannot be opened.
        """
        if self.is_open():
            return

        logger.info("Opening serial port %s @ %d", self._port, self._baud)

        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                timeout=self._timeout_s,
                write_timeout=self._timeout_s,
            )
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open serial port: {e}") from e

        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._rx_buf.clear()

    def close(self) -> None:
        """Close the serial connection."""
        if self._ser is None:
            return

        logger.info("Closing serial port %s", self._port)

        if self._ser.is_open:
            self._ser.close()

        self._ser = None

    def write_line(self, line: str) -> None:
        """Send a line of text; appends '\\n' and flushes.

        Args:
            line: String to send; a single trailing '\\n' is always appended.

        Raises:
            RuntimeError: If the port is not open.
        """
        ser = self._ensure_open()
        cleaned_line = line.strip("\r\n")
        to_send = (cleaned_line + "\n").encode("utf-8", errors="replace")

        try:
            ser.write(to_send)
            ser.flush()
        except serial.SerialException as e:
            raise RuntimeError(f"Serial write failed: {e}") from e

        logger.debug("SENT: %s", cleaned_line)

    def read_line(self, timeout_s: float | None = None) -> str:
        """Read one non-empty line (stripped of CR/LF) before timeout.

        Reads in a non-blocking loop until a full line is received or
        the given timeout is exceeded. Empty lines are ignored.

        Args:
            timeout_s: Seconds to wait for a line; uses instance default if None.

        Returns:
            The first non-empty stripped line (no CR/LF).

        Raises:
            RuntimeError: If the port is not open.
            TimeoutError: If timeout is exceeded before a non-empty line.
        """
        ser = self._ensure_open()
        wait_s = self._timeout_s if timeout_s is None else timeout_s
        deadline = time.monotonic() + wait_s

        while True:
            for sep in (b"\r\n", b"\n", b"\r"):
                idx = self._rx_buf.find(sep)
                if idx != -1:
                    line = self._rx_buf[:idx]
                    self._rx_buf = self._rx_buf[idx + len(sep) :]

                    text = line.decode("utf-8", errors="replace").strip()
                    if text:
                        logger.debug("RECV: %s", text)
                        return text

            now = time.monotonic()
            if now >= deadline:
                raise TimeoutError(
                    f"read_line timed out after {wait_s}s without a complete line"
                )

            try:
                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    logger.debug("RAW RECV: %r", chunk)
                    self._rx_buf.extend(chunk)
            except serial.SerialException as e:
                raise RuntimeError(f"Serial read failed: {e}") from e

            if not chunk:
                sleep_time = min(READ_LINE_WAIT_TIME_S, deadline - now)
                time.sleep(max(MIN_READ_LINE_WAIT_TIME_S, sleep_time))

            if len(self._rx_buf) > MAX_LINE_LENGTH:
                raise RuntimeError("Line buffer overflow")

    def drain(self) -> None:
        """Clear any buffered incoming data from the serial port.

        Raises:
            RuntimeError: If the port is not open.
        """
        ser = self._ensure_open()

        while True:
            n = ser.in_waiting
            if not n:
                break
            data = ser.read(n or 1)
            logger.debug("DRAINED: %r", data)

    def __enter__(self) -> SerialTransport:
        """Context manager entry."""
        self.open()
        return self

    def __exit__(self, *args: Any) -> None:
        """Context manager exit."""
        self.close()
