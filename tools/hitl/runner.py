"""HITL test runner orchestration."""

from dataclasses import dataclass
from typing import Any


@dataclass
class RunnerConfig:
    """Configuration for a HITL run."""

    port: str = ""
    baud_rate: int = 115200
    sample_count: int = 0
    timeout: float = 30.0
    no_flash: bool = False


@dataclass
class RunnerResult:
    """Result of a HITL run."""

    success: bool
    message: str
    samples_read: int = 0
    validation_passed: bool = False
    details: dict[str, Any] | None = None


class HitlRunner:
    """Orchestrates a full HITL test run."""

    def __init__(self, config: RunnerConfig | None = None) -> None:
        """Initialize runner with config or keyword overrides.

        Args:
            config: RunnerConfig instance.
        """
        self._config = config

    def run(self) -> RunnerResult:
        """Execute one HITL test run.

        Returns:
            RunnerResult with success status and details.
        """
        raise NotImplementedError
