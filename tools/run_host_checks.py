"""
Host and target quality gates for rtos-pedometer.

Orchestrates formatting (clang-format), host pipeline (test/build/), and
target pipeline (build/).
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import platform
import shutil
import stat
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from argparse import Namespace

LOGGER_NAME = "run_host_checks"
DIR_TEST = "test"
DIR_BUILD = "build"
DIR_LOGS = ".logs"
DIR_MBED_OS = "mbed-os"
DIR_COVERAGE = ".coverage"
HOST_BUILD_CONFIG = "RelWithDebInfo"
TARGET_VARIANT_DIR = "NUCLEO_F767ZI-Develop"
TARGET_PROJECT_NAME = "rtos-pedometer"
MBED_TARGET_DEFAULT = "NUCLEO_F767ZI"

_FORMAT_EXTENSIONS = frozenset(
    {
        ".c",
        ".h",
        ".cpp",
        ".hpp",
        ".cc",
        ".cxx",
        ".hxx",
        ".hh",
        ".c++",
        ".h++",
        ".cppm",
        ".ixx",
        ".cxxm",
        ".ccm",
        ".c++m",
    }
)
_FORMAT_EXTENSIONS_LOWER = {e.lower() for e in _FORMAT_EXTENSIONS}

REPO_ROOT = Path(__file__).resolve().parents[1]
EXCLUDE_DIRS = (
    REPO_ROOT / DIR_MBED_OS,
    REPO_ROOT / DIR_BUILD,
    REPO_ROOT / DIR_TEST / DIR_BUILD,
)


def _logger() -> logging.Logger:
    """Return the shared logger (configured by setup_logging when run as main)."""
    return logging.getLogger(LOGGER_NAME)


def setup_logging(repo_root: Path) -> Path:
    """
    Configure logging: console at INFO, file at DEBUG.
    Log file: repo_root/{DIR_LOGS}/host_gate_<TIMESTAMP>.log.
    """
    log_dir = repo_root / DIR_LOGS
    log_dir.mkdir(exist_ok=True)
    log_file = log_dir / f"host_gate_{datetime.now():%Y%m%d_%H%M%S}.log"

    logger = logging.getLogger(LOGGER_NAME)
    logger.setLevel(logging.DEBUG)
    logger.handlers.clear()
    formatter = logging.Formatter(
        "%(asctime)s %(levelname)s: %(message)s", datefmt="%H:%M:%S"
    )

    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(formatter)
    logger.addHandler(ch)

    fh = logging.FileHandler(log_file, encoding="utf-8")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(formatter)
    logger.addHandler(fh)

    return log_file


def run_cmd(
    cmd: list[str],
    cwd: Path,
    step_name: str,
    env: dict[str, str] | None = None,
) -> bool:
    """
    Run a subprocess command. Logs the command before execution, then success or failure.
    Returns True on success, False on non-zero exit or FileNotFoundError.
    """
    log = _logger()
    log.info("[%s] Running: %s", step_name, " ".join(cmd))
    env = {**os.environ, **(env or {})}
    try:
        with subprocess.Popen(
            cmd,
            cwd=cwd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        ) as proc:
            if proc.stdout:
                for line in proc.stdout:
                    log.info(line.rstrip())
            proc.wait()
            if proc.returncode != 0:
                log.error("[%s] Failed with exit code %d", step_name, proc.returncode)
                return False
    except FileNotFoundError as e:
        log.error("[%s] Failed: %s", step_name, e)
        return False
    log.info("[%s] Succeeded", step_name)
    return True


def get_sources_from_compile_commands(build_dir: Path) -> list[Path]:
    """
    Load compile_commands.json from build_dir, extract "file" entries,
    normalize paths, deduplicate, and return a sorted list.
    Raises FileNotFoundError if compile_commands.json is missing.
    """
    path = build_dir / "compile_commands.json"
    if not path.exists():
        raise FileNotFoundError(
            f"compile_commands.json not found in build directory: {build_dir}. "
            "Run CMake configure first."
        )
    try:
        raw = path.read_text(encoding="utf-8")
        entries = json.loads(raw)
    except OSError as e:
        raise OSError(f"Failed to read {path}: {e}") from e
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in {path}: {e}") from e
    if not isinstance(entries, list):
        raise ValueError(
            f"compile_commands.json must be a JSON array; got {type(entries).__name__}"
        )
    seen: set[Path] = set()
    sources: list[Path] = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        file_path = entry.get("file")
        work_dir = entry.get("directory", "")
        if not file_path:
            continue
        p = Path(file_path)
        if not p.is_absolute() and work_dir:
            p = (Path(work_dir) / p).resolve()
        else:
            p = p.resolve()
        if any(p.is_relative_to(d) for d in EXCLUDE_DIRS):
            continue
        if p not in seen:
            seen.add(p)
            sources.append(p)
    _logger().info(
        "Found %d unique source files in compile_commands.json", len(sources)
    )
    return sorted(sources)


def _run_clang_tidy_on_sources(
    sources: list[Path],
    build_dir: Path,
    repo_root: Path,
    step_prefix: str,
) -> bool:
    """
    Run clang-tidy on each source file with -p=build_dir and --warnings-as-errors=*.
    Logs and fails fast on first failure. Used by both host and target pipelines.
    """
    for src in sources:
        step_name = f"{step_prefix} {src.name}"
        if not run_cmd(
            [
                "clang-tidy",
                str(src),
                f"-p={build_dir}",
                "--warnings-as-errors=*",
            ],
            repo_root,
            step_name,
        ):
            return False
    return True


def run_clang_format(repo_root: Path, fix: bool) -> bool:
    """
    Format all C/C++ (including header) files in the repo. Excludes mbed-os/,
    build/, and test/build/.
    """
    log = _logger()
    exclude_dirs = (
        repo_root / DIR_MBED_OS,
        repo_root / DIR_BUILD,
        repo_root / DIR_TEST / DIR_BUILD,
    )
    sources = [
        p
        for p in repo_root.rglob("*")
        if p.is_file()
        and p.suffix.lower() in _FORMAT_EXTENSIONS_LOWER
        and not any(p.is_relative_to(d) for d in exclude_dirs)
    ]
    sources = sorted(sources)
    if not sources:
        log.warning("No C/C++ files found for clang-format; skipping.")
        return True
    log.info("[clang-format] Starting: %d files (fix=%s)", len(sources), fix)
    cmd = ["clang-format", "-i" if fix else "--dry-run", *[str(p) for p in sources]]
    if not fix:
        cmd.append("--Werror")
    return run_cmd(cmd, repo_root, "clang-format" + (" (fix)" if fix else " (check)"))


def _rmtree_onexc(_func: object, path: str, exc: BaseException) -> None:
    """
    onexc callback for shutil.rmtree. On PermissionError (read-only on Windows),
    clear the read-only bit and retry. Other errors are logged and re-raised.
    """
    log = _logger()
    if isinstance(exc, PermissionError):
        log.warning(
            "Read-only file encountered, clearing attribute and retrying: %s", path
        )
        os.chmod(path, stat.S_IWRITE)
        os.unlink(path)
        return
    log.error("Failed to remove %s: %s", path, exc)
    raise exc


def _remove_directory(path: Path, step_name: str) -> bool:
    """
    Remove a directory tree using shutil.rmtree(..., onexc=_rmtree_onexc).
    Handles read-only files on Windows. Returns True on success, False on error.
    Errors are logged clearly.
    """
    log = _logger()
    if not path.exists():
        log.info("Directory does not exist; skipping remove: %s", path)
        return True
    if not path.is_dir():
        log.error("%s: path is not a directory: %s", step_name, path)
        return False
    log.info("Removing directory: %s", path)
    try:
        shutil.rmtree(path, onexc=_rmtree_onexc)
        log.info("Directory removed: %s", path)
        return True
    except OSError as e:
        log.error("%s failed: %s", step_name, e)
        return False


class HostPipeline:
    """Host quality gate pipeline: clean, configure, build, clang-tidy, unit tests, coverage."""

    def __init__(self, repo_root: Path, args: Namespace):
        self.repo_root = repo_root
        self.args = args
        self.log = _logger()
        self.host_build = repo_root / DIR_TEST / DIR_BUILD
        self.test_dir = repo_root / DIR_TEST
        self.coverage_dir = repo_root / DIR_COVERAGE
        self.html_dir = self.coverage_dir / "html"
        self.profdata_file = self.coverage_dir / "coverage.profdata"

    def clean(self) -> bool:
        return _remove_directory(self.host_build, "Clean (host)")

    def configure(self, enable_coverage: bool = False) -> bool:
        cmd = ["cmake", "--preset", "host-check"]
        if enable_coverage:
            cmd.append("-DENABLE_COVERAGE=ON")
        return run_cmd(cmd, self.test_dir, "CMake configure (host)")

    def build(self) -> bool:
        return run_cmd(
            ["cmake", "--build", "--preset", "host-check"],
            self.test_dir,
            "Build (host)",
        )

    def clang_tidy(self) -> bool:
        try:
            sources = get_sources_from_compile_commands(self.host_build)
        except (FileNotFoundError, OSError, ValueError) as e:
            self.log.error("[clang-tidy] %s", e)
            return False
        return _run_clang_tidy_on_sources(
            sources, self.host_build, self.repo_root, "clang-tidy (host)"
        )

    def ctest(self) -> bool:
        test_file = self.host_build / "CTestTestfile.cmake"
        if not test_file.exists():
            self.log.error(
                "[Unit tests] CTest not available; configure and build first."
            )
            return False
        return run_cmd(
            ["ctest", "--output-on-failure", "-C", HOST_BUILD_CONFIG],
            self.host_build,
            "Unit tests (ctest)",
        )

    def coverage(self) -> bool:
        """Generate LLVM coverage report for host unit tests using llvm-profdata + llvm-cov."""
        self.coverage_dir.mkdir(parents=True, exist_ok=True)
        self.html_dir.mkdir(parents=True, exist_ok=True)

        profraw = self.host_build / HOST_BUILD_CONFIG / "coverage.profraw"
        env = os.environ.copy()
        env["LLVM_PROFILE_FILE"] = str(profraw)

        self.log.info("[Coverage] Running tests to generate .profraw files")
        if not run_cmd(
            ["ctest", "--output-on-failure", "-C", HOST_BUILD_CONFIG],
            self.host_build,
            "Unit tests (coverage)",
            env=env,
        ):
            return False

        self.log.info("[Coverage] Merging .profraw → .profdata")
        merge_cmd = [
            "llvm-profdata",
            "merge",
            "-sparse",
            str(profraw),
            "-o",
            str(self.profdata_file),
        ]
        if not run_cmd(merge_cmd, self.repo_root, "llvm-profdata merge"):
            return False

        self.log.info("[Coverage] Generating HTML report")
        exe_path = self.host_build / HOST_BUILD_CONFIG / "runUnitTests"
        if platform.system() == "Windows":
            exe_path = exe_path.with_suffix(".exe")
        cov_cmd = [
            "llvm-cov",
            "show",
            str(exe_path),
            f"-instr-profile={self.profdata_file}",
            "-format=html",
            f"-output-dir={self.html_dir}",
            "-Xdemangler",
            "c++filt",
            "-show-line-counts-or-regions",
            "-show-expansions",
            "-show-instantiations",
        ]
        if not run_cmd(cov_cmd, self.repo_root, "llvm-cov show"):
            return False

        self.log.info("[Coverage] Report available at: %s", self.html_dir)
        return True

    def run(self) -> bool:
        """Orchestrate the host pipeline based on args: test, lint, coverage, or full gate."""
        test_only = getattr(self.args, "test", False)
        lint_only = getattr(self.args, "lint", False)
        coverage_only = getattr(self.args, "coverage", False)
        full_gate = not test_only and not lint_only and not coverage_only

        completed: list[str] = []

        steps: list[tuple[str, Callable[[], bool]]] = []

        if not lint_only:
            steps.append(("clean", self.clean))

        steps.append(
            (
                "configure",
                lambda: self.configure(enable_coverage=(full_gate or coverage_only)),
            )
        )

        if not lint_only:
            steps.append(("build", self.build))

        if lint_only or full_gate:
            steps.append(("clang-tidy", self.clang_tidy))

        if test_only:
            steps.append(("test", self.ctest))

        if full_gate or coverage_only:
            steps.append(("coverage", self.coverage))

        for name, func in steps:
            if not func():
                self.log.error("[Host pipeline] Step failed: %s", name)
                return False
            completed.append(name)

        self.log.info("[Host pipeline] Completed: %s", ", ".join(completed))
        return True


def run_target_pipeline(repo_root: Path, args: Namespace) -> bool:
    """
    Run the target (firmware) quality gate. Build directory: {DIR_BUILD}/.
    Steps: clean, configure (same as VSCode), build, clang-tidy on sources
    from compile_commands.json. Fully independent of host.
    """
    log = _logger()
    log.info("[Target pipeline] Starting")
    target_build_root = repo_root / DIR_BUILD
    target_build_dir = target_build_root / TARGET_VARIANT_DIR

    if not _remove_directory(target_build_root, "Clean (target)"):
        return False

    configure_cmd = [
        "cmake",
        "-B",
        str(target_build_dir),
        "-S",
        str(repo_root),
        "-G",
        "Ninja",
        f"-DMBED_TARGET={MBED_TARGET_DEFAULT}",
    ]
    if not run_cmd(configure_cmd, repo_root, "CMake configure (target)"):
        return False

    build_cmd = [
        "cmake",
        "--build",
        str(target_build_dir),
        "--target",
        TARGET_PROJECT_NAME,
    ]
    if not run_cmd(build_cmd, repo_root, "Build (target)"):
        return False

    log.info("[Target pipeline] Completed: clean, configure, build, clang-tidy.")
    return True


def main() -> int:
    """
    Entry point: parse args, run formatting then requested pipelines.
    Exit 0 if all requested steps succeed, 1 if any step fails.
    """
    repo_root = REPO_ROOT
    log_file = setup_logging(repo_root)
    log = _logger()

    parser = argparse.ArgumentParser(
        description="Host and target quality gates: formatting, build, test, lint, coverage.",
    )
    parser.add_argument(
        "--target",
        action="store_true",
        help="Run only firmware pipeline (formatting + target)",
    )
    parser.add_argument(
        "--host", action="store_true", help="Run only host pipeline (formatting + host)"
    )
    parser.add_argument(
        "--fix-format",
        action="store_true",
        help="Auto-format with clang-format instead of check (--dry-run --Werror)",
    )
    parser.add_argument(
        "-t", "--test", action="store_true", help="Run unit tests only (host pipeline)"
    )
    parser.add_argument(
        "-l",
        "--lint",
        action="store_true",
        help="Run formatting check + linting only (host pipeline)",
    )
    parser.add_argument(
        "-c",
        "--coverage",
        action="store_true",
        help="Generate coverage only (host pipeline)",
    )

    args = parser.parse_args()

    flags = [args.test, args.lint, args.coverage]
    if sum(bool(f) for f in flags) > 1:
        log.error("Flags --test, --lint, and --coverage are mutually exclusive.")
        return 1
    if not any(flags):
        args.test = True

    test_cmake = repo_root / DIR_TEST / "CMakeLists.txt"
    if not test_cmake.exists():
        log.error("Repo layout invalid: %s not found; run from repo root.", test_cmake)
        return 1

    run_target_requested = args.target
    run_host_requested = args.host or not args.target

    format_ok = run_clang_format(repo_root, fix=args.fix_format)

    target_ok = True
    if run_target_requested:
        target_ok = run_target_pipeline(repo_root, args) if format_ok else False

    host_ok = True
    if run_host_requested:
        if format_ok and target_ok:
            pipeline = HostPipeline(repo_root, args)
            host_ok = pipeline.run()
        else:
            host_ok = False

    success = format_ok and target_ok and host_ok
    if success:
        log.info("All requested steps completed successfully. See logs in %s", log_file)
        if args.coverage:
            log.info(
                "Coverage report available at %s",
                repo_root / DIR_COVERAGE / "html/index.html",
            )
        return 0
    log.error("One or more steps failed. See logs in %s", log_file)
    return 1


if __name__ == "__main__":
    sys.exit(main())
