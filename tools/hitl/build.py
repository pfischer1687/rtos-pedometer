"""Build and flash management for HITL."""

from __future__ import annotations

import logging
import os
import shutil
import stat
import subprocess
from pathlib import Path
from tools.common.logging import HITL_LOGGER_NAME

BUILD_DIR_REL_PARENT = Path("build-hitl")
BUILD_DIR_REL = BUILD_DIR_REL_PARENT / "NUCLEO_F767ZI-Develop"
TARGET_NAME = "rtos-pedometer"
TIMEOUT_S = 300.0
BUILD_TYPE_DEFAULT = "Develop"
TARGET_DEFAULT = "NUCLEO_F767ZI"
UPLOAD_METHOD_DEFAULT = "STM32CUBE"
NUM_FLASH_RETRIES = 2

logger = logging.getLogger(HITL_LOGGER_NAME)


class BuildError(RuntimeError):
    """Raised when a build step fails."""

    pass


class FlashError(RuntimeError):
    """Raised when a flash step fails."""

    pass


class BuildManager:
    """Clean, configure, build, and flash firmware using CMake."""

    def __init__(
        self,
        repo_root: Path,
        build_type: str = BUILD_TYPE_DEFAULT,
        target: str = TARGET_DEFAULT,
        upload_method: str = UPLOAD_METHOD_DEFAULT,
        target_name: str = TARGET_NAME,
    ) -> None:
        """Initialize build manager.

        Args:
            repo_root: Repository root path containing top-level CMakeLists.txt.
            build_type: Build type to use.
            target: Target to build.
            upload_method: Upload method to use.
            target_name: Target name to use.
        """
        self.repo_root = repo_root.resolve()
        self.build_dir = (self.repo_root / BUILD_DIR_REL).resolve()
        self.build_type = build_type
        self.target = target
        self.upload_method = upload_method
        self.target_name = target_name

    def _artifact_candidates(self) -> list[Path]:
        """Return a list of possible artifact paths."""
        return [
            self.build_dir / f"{self.target_name}.elf",
            self.build_dir / f"{self.target_name}.bin",
            self.build_dir / f"{self.target_name}.hex",
        ]

    def _run_cmd(
        self,
        cmd: list[str],
        step: str,
        cwd: Path | None = None,
        env: dict[str, str] | None = None,
        error_type: type[RuntimeError] = RuntimeError,
        timeout_s: float = TIMEOUT_S,
    ) -> None:
        """Run a subprocess command and fail fast on errors.

        Args:
            cmd: Command vector to execute.
            step: Step label for logging.
            cwd: Working directory to run the command in.
            env: Environment variables to set.
            error_type: Type of error to raise if the command fails.
            timeout_s: Timeout in seconds.

        Raises:
            RuntimeError: If command cannot be started or exits non-zero.
        """
        logger.info("[%s] Running", step)
        logger.debug("Command: %s", " ".join(cmd))

        cwd = cwd or self.repo_root
        proc = None
        env = {**os.environ, **(env or {})}

        try:
            with subprocess.Popen(
                cmd,
                cwd=cwd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=env,
            ) as proc:
                if proc.stdout:
                    for line in proc.stdout:
                        logger.debug(line.rstrip())
                proc.wait(timeout=timeout_s)
                if proc.returncode != 0:
                    raise error_type(f"{step} failed with exit code {proc.returncode}")
        except FileNotFoundError as exc:
            raise error_type(f"{step} failed: {exc}") from exc
        except subprocess.TimeoutExpired:
            if proc:
                proc.kill()
                proc.wait()
            raise error_type(f"{step} timed out")

        logger.info("[%s] Succeeded", step)

    def _rmtree_onexc(self, _func: object, path: str, exc: BaseException) -> None:
        """Remove read-only files safely during cleanup.

        Args:
            _func: Function that raised the exception.
            path: Path to the file or directory to remove.
            exc: Exception that was raised.
        """
        if isinstance(exc, PermissionError):
            logger.warning(
                "Read-only path during clean; clearing attribute and retrying: %s",
                path,
            )
            os.chmod(path, stat.S_IWRITE)
            os.unlink(path)
            return
        raise exc

    def clean(self) -> None:
        """Delete the build/ directory safely."""
        logger.info("[clean] Removing build directory: %s", self.build_dir)

        build_root = (self.repo_root / BUILD_DIR_REL_PARENT).resolve()
        if not build_root.exists():
            logger.info("[clean] Build root directory does not exist; skipping")
            return

        shutil.rmtree(build_root, onexc=self._rmtree_onexc)
        self.build_dir.mkdir(parents=True, exist_ok=True)

        logger.info("[clean] Succeeded")

    def configure(self) -> None:
        """Configure CMake/Ninja for test-entry target build."""
        self._run_cmd(
            [
                "cmake",
                "-B",
                str(self.build_dir),
                "-S",
                str(self.repo_root),
                "-G",
                "Ninja",
                f"-DCMAKE_BUILD_TYPE={self.build_type}",
                f"-DMBED_TARGET={self.target}",
                "-DTEST_ENTRY=ON",
                f"-DUPLOAD_METHOD={self.upload_method}",
            ],
            step="configure",
            cwd=self.repo_root,
        )

    def build(self) -> None:
        """Build firmware target."""
        self._run_cmd(
            [
                "cmake",
                "--build",
                str(self.build_dir),
                "--target",
                self.target_name,
            ],
            step="build",
            cwd=self.build_dir,
            error_type=BuildError,
        )

    def flash(self, retries: int = NUM_FLASH_RETRIES) -> None:
        """Flash firmware to the target."""
        for attempt in range(retries + 1):
            try:
                self._run_cmd(
                    ["ninja", f"flash-{self.target_name}"],
                    step="flash",
                    cwd=self.build_dir,
                    error_type=FlashError,
                )

                return
            except RuntimeError as e:
                if attempt == retries:
                    raise FlashError(
                        f"Flash failed after {retries + 1} attempts: {e}"
                    ) from e

                logger.warning(
                    "Flash failed (%s), retrying (%d/%d)",
                    e,
                    attempt + 1,
                    retries,
                )

    def _check_dependencies(self) -> None:
        """Check if the required tools are installed."""
        for tool in ["cmake", "ninja"]:
            if shutil.which(tool) is None:
                raise BuildError(f"Missing required tool: {tool}")
        logger.info("[check_dependencies] All dependencies found")

    def _verify_artifact(self) -> None:
        """Verify that the artifact exists."""
        candidates = self._artifact_candidates()
        artifact = next((p for p in candidates if p.exists()), None)
        if not artifact:
            raise BuildError(
                f"Build succeeded but no artifact found. Checked: {candidates}"
            )
        logger.info("[verify_artifact] Artifact found: %s", artifact)

    def run_all(self, clean: bool = False) -> None:
        """Run clean, configure, build, and flash steps in order.

        Args:
            clean: Whether to clean the build directory before running.

        Raises:
            RuntimeError: If the build succeeded but the artifact is missing.
        """
        logger.info("[run_all] Starting full firmware flash pipeline")

        self._check_dependencies()

        if clean:
            self.clean()

        self.configure()
        self.build()
        self._verify_artifact()
        self.flash()

        logger.info("[run_all] Completed firmware flash pipeline successfully")
