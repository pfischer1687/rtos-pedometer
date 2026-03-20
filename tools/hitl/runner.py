"""HITL test runner orchestration."""

from __future__ import annotations

import logging
from dataclasses import dataclass
import time
from tools.common.logging import HITL_LOGGER_NAME
from tools.hitl.imu_device import ImuDevice, ImuSample
from tools.hitl.protocol import HitlProtocolClient, ProtocolError
from tools.hitl.transport import SerialTransport
from tools.hitl.validator import ImuValidator, ValidationError

logger = logging.getLogger(HITL_LOGGER_NAME)


@dataclass
class HitlResult:
    """Result of a HITL run."""

    success: bool
    message: str | None = None
    iterations_completed: int = 0


@dataclass
class RunnerConfig:
    """Configuration for a HITL run."""

    port: str
    baud_rate: int = 115200
    sample_count: int = 10
    timeout: float = 30.0
    iterations: int = 1
    cmd_timeout_s: float = 5.0


class HitlRunner:
    """Orchestrates HITL iterations over serial: IMU commands, capture, validate."""

    def __init__(self, config: RunnerConfig) -> None:
        """Wire transport, protocol, device, and validator from config.

        Args:
            config: Run parameters (port, samples, timeouts, iterations).
        """
        self._config = config
        self._transport = SerialTransport(
            port=config.port,
            baud=config.baud_rate,
            timeout_s=config.cmd_timeout_s,
        )
        self._protocol = HitlProtocolClient(self._transport)
        self._imu = ImuDevice(self._protocol)
        self._validator = ImuValidator()

    def run_iteration(self, iteration: int) -> list[ImuSample]:
        """One full cycle: init -> configure -> start -> read_n -> validate -> stop.

        Args:
            iteration: The iteration number.

        Returns:
            Samples collected in this iteration.
        """
        cfg = self._config
        cmd_t = cfg.cmd_timeout_s
        read_t = cfg.timeout

        assert self._transport.is_open()

        self._imu.init(timeout_s=cmd_t)
        self._imu.configure(timeout_s=cmd_t)
        self._imu.start(timeout_s=cmd_t)
        try:
            samples = self._imu.read_n(n=cfg.sample_count, timeout_s=read_t)
            self._validator.validate(samples)
            return samples
        finally:
            try:
                self._imu.stop(timeout_s=cmd_t)
            except Exception as exc:
                logger.warning(
                    "Iteration %d: IMU stop failed during cleanup: %s",
                    iteration + 1,
                    exc,
                )

    def _fail(self, message: str, completed: int) -> HitlResult:
        """Fail the HITL run with a message and return the result.

        Args:
            message: The message to fail the HITL run with.
            completed: The number of iterations completed.

        Returns:
            HitlResult of the HITL run.
        """
        logger.error(message)
        return HitlResult(
            success=False, message=message, iterations_completed=completed
        )

    def run(self) -> HitlResult:
        """Run `config.iterations` iterations; stop on first failure.

        Returns:
            HitlResult of the HITL run.
        """
        completed = 0
        run_start = time.monotonic()

        if self._config.sample_count <= 0:
            return self._fail("sample_count must be > 0", 0)

        if self._config.iterations <= 0:
            return self._fail("iterations must be > 0", 0)

        try:
            logger.info(
                "Opening serial port %s @ %d", self._config.port, self._config.baud_rate
            )
            with self._transport:
                logger.info("Performing pre-flight ping")
                self._imu.ping(timeout_s=self._config.cmd_timeout_s)

                for it in range(self._config.iterations):
                    logger.info(
                        "HITL iteration %d / %d starting",
                        it + 1,
                        self._config.iterations,
                    )
                    start = time.monotonic()

                    try:
                        self.run_iteration(it)
                        completed += 1
                    except (
                        ProtocolError,
                        ValidationError,
                        TimeoutError,
                        RuntimeError,
                    ) as exc:
                        return self._fail(
                            message=f"HITL iteration {it + 1} / {self._config.iterations} failed: {exc}",
                            completed=completed,
                        )
                    except Exception as exc:
                        return self._fail(
                            message=f"HITL iteration {it + 1} / {self._config.iterations} had unexpected error: {exc}",
                            completed=completed,
                        )

                    duration = time.monotonic() - start
                    logger.info(
                        "HITL iteration %d / %d completed successfully in %.2fs",
                        it + 1,
                        self._config.iterations,
                        duration,
                    )

                total = time.monotonic() - run_start
                logger.info("HITL run completed in %.2fs", total)
                return HitlResult(
                    success=True, message="OK", iterations_completed=completed
                )

        except KeyboardInterrupt:
            logger.warning("HITL run interrupted")
            return self._fail("Interrupted by user", completed)
