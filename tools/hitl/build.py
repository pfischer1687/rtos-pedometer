"""Build and flash management for HITL."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import shutil

BUILD_DIR_NAME = "build"


@dataclass
class BuildResult:
    """Result of a build/flash operation."""

    success: bool
    message: str
    build_dir: Path | None = None
    artifact_path: Path | None = None
    details: dict[str, Any] | None = None


class BuildManager:
    """Handles building the firmware and flashing the device."""

    def __init__(
        self,
        project_root: Path,
        build_type: str = "Develop",
        target: str = "NUCLEO_F767ZI",
        test_entry: bool = True,
        upload_method: str = "STM32CUBE",
    ):
        """Initialize build manager with project root and options.

        Args:
            project_root: Path to the project root.
            build_type: The build type (Develop, Debug, Release).
            target: The target board (NUCLEO_F767ZI).
            test_entry: Whether to test the entry point.
            upload_method: The upload method (STM32CUBE).
        """
        self._project_root = project_root
        self.build_type = build_type
        self.target = target
        self.test_entry = test_entry
        self.upload_method = upload_method
        self.build_dir = self._project_root / BUILD_DIR_NAME / f"{target}-{build_type}"

    def clean(self) -> None:
        """Clean the build directory."""
        if self.build_dir.exists():
            shutil.rmtree(self.build_dir)

    def build(self) -> BuildResult:
        """Run the build (compile only).

        Returns:
            BuildResult with artifact path and success status.
        """
        raise NotImplementedError

    def flash(self, image_path: Path | str | None = None) -> BuildResult:
        """Flash the firmware to the connected device.

        Args:
            image_path: Path to binary to flash; if None, use last build artifact.

        Returns:
            BuildResult indicating flash success/failure.
        """
        raise NotImplementedError

    def build_and_flash(self) -> BuildResult:
        """Build then flash in one step.

        Returns:
            BuildResult for the combined operation.
        """
        raise NotImplementedError
