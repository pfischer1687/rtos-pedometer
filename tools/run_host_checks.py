"""Host-quality gate for rtos-pedometer: build, test, lint, coverage."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path
import logging
from typing import Callable
import stat
import platform

REPO_ROOT: Path = Path(__file__).resolve().parents[1]
INCL_EXT = {
    ".cpp",
    ".cxx",
    ".cc",
    ".c++",
    ".hpp",
    ".h",
    ".hh",
    ".hxx",
    ".h++",
    ".cppm",
    ".ixx",
    ".c",
    ".cxxm",
    ".c++m",
    ".ccm",
}
BUILD_DIR = REPO_ROOT / "test" / "build"
EXCL_DIRS = {REPO_ROOT / "mbed-os", REPO_ROOT / "build", BUILD_DIR}
CMAKE_PRESET = "host-check"

LOG_DIR = REPO_ROOT / ".logs"
LOG_DIR.mkdir(exist_ok=True)
LOG_FILE = LOG_DIR / f"host_gate_{datetime.now():%Y%m%d_%H%M%S}.log"

logger = logging.getLogger("host_gate")
logger.setLevel(logging.INFO)

formatter = logging.Formatter("%(asctime)s %(levelname)s: %(message)s", "%H:%M:%S")

ch = logging.StreamHandler()
ch.setFormatter(formatter)
logger.addHandler(ch)

fh = logging.FileHandler(LOG_FILE)
fh.setFormatter(formatter)
logger.addHandler(fh)

log = logger


def _handle_remove_readonly(func, path, exc):
    """Fix read-only files on Windows during rmtree."""
    if not isinstance(exc, PermissionError):
        raise exc
    log.warning("Changing permissions to delete: %s", path)
    os.chmod(path, stat.S_IWRITE)
    func(path)


def get_lint_sources() -> list[Path]:
    """Return all C/C++ source/header files outside excluded directories."""
    sources = sorted(
        p
        for p in REPO_ROOT.rglob("*")
        if p.is_file()
        and p.suffix.lower() in {ext.lower() for ext in INCL_EXT}
        and not any(p.is_relative_to(d) for d in EXCL_DIRS)
    )
    log.info("Found %d source files for linting", len(sources))
    return sources


def run_cmd(
    cmd: list[str], cwd: Path, step_name: str, env: dict[str, str] | None = None
) -> bool:
    """Run a subprocess command; return True on success."""
    log.info("Running: %s", " ".join(cmd))
    env = {**os.environ, **(env or {})}
    with subprocess.Popen(
        cmd,
        cwd=cwd,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    ) as proc:
        for line in proc.stdout or []:
            log.info(line.rstrip())
        proc.wait()
        if proc.returncode != 0:
            log.error("%s failed with exit code %d", step_name, proc.returncode)
            return False
    log.info("%s succeeded", step_name)
    return True


def step_clean_build() -> bool:
    if not BUILD_DIR.exists():
        log.info("Build directory does not exist; skipping clean.")
        return True
    log.info("Removing build directory: %s", BUILD_DIR)
    try:
        shutil.rmtree(BUILD_DIR, onexc=_handle_remove_readonly)
        log.info("Build directory removed.")
        return True
    except OSError as e:
        log.error("Failed to remove build directory: %s", e)
        return False


def step_configure(enable_coverage: bool = False) -> bool:
    cmd = ["cmake", "--preset", CMAKE_PRESET]
    if enable_coverage:
        cmd.append("-DENABLE_COVERAGE=ON")
    return run_cmd(cmd, REPO_ROOT / "test", "CMake configure")


def step_build() -> bool:
    return run_cmd(
        ["cmake", "--build", "--preset", CMAKE_PRESET], REPO_ROOT / "test", "Build"
    )


def step_ctest() -> bool:
    ctest_file = BUILD_DIR / "CTestTestfile.cmake"
    if not ctest_file.exists():
        log.error("CTest not available; configure and build first.")
        return False
    return run_cmd(
        ["ctest", "--output-on-failure", "-C", "RelWithDebInfo"],
        BUILD_DIR,
        "Unit tests (ctest)",
    )


def step_clang_format(sources: list[Path], fix: bool = False) -> bool:
    if not sources:
        log.warning("No sources for clang-format; skipping.")
        return True
    cmd = ["clang-format", "-i" if fix else "--dry-run", *map(str, sources)]
    if not fix:
        cmd.append("--Werror")
    return run_cmd(cmd, REPO_ROOT, "clang-format check" + (" (fix)" if fix else ""))


def step_clang_tidy(sources: list[Path]) -> bool:
    if not sources:
        log.warning("No sources for clang-tidy; skipping.")
        return True
    compile_commands = BUILD_DIR / "compile_commands.json"
    if not compile_commands.exists():
        log.error("compile_commands.json missing; configure with host-check first.")
        return False
    success = True
    for src in sources:
        if not run_cmd(
            ["clang-tidy", str(src), f"-p={BUILD_DIR}", "--warnings-as-errors=*"],
            REPO_ROOT,
            f"clang-tidy {src.name}",
        ):
            success = False
    return success


def step_coverage() -> bool:
    if platform.system() == "Windows":
        log.warning("Coverage not supported on Windows; skipping.")
        return True

    run_dir = BUILD_DIR / "RelWithDebInfo"
    if not run_dir.is_dir():
        log.error("Coverage run directory not found: %s", run_dir)
        return False

    coverage_dir = REPO_ROOT / "coverage"
    info_dir = coverage_dir / "lcov"
    html_dir = coverage_dir / "html"
    info_dir.mkdir(parents=True, exist_ok=True)
    html_dir.mkdir(parents=True, exist_ok=True)

    raw = info_dir / "coverage.info"
    filtered = info_dir / "coverage_filtered.info"

    cmds = [
        (
            [
                "lcov",
                "--capture",
                "--directory",
                str(run_dir),
                "--output-file",
                str(raw),
                "--rc",
                "lcov_branch_coverage=1",
            ],
            "lcov capture",
        ),
        (
            [
                "lcov",
                "--remove",
                str(raw),
                "*/mbed-os/*",
                "*/test/*",
                "--output-file",
                str(filtered),
                "--rc",
                "lcov_branch_coverage=1",
            ],
            "lcov filter",
        ),
        (
            [
                "genhtml",
                str(filtered),
                "--output-directory",
                str(html_dir),
                "--branch-coverage",
            ],
            "genhtml",
        ),
    ]

    for cmd, name in cmds:
        if not run_cmd(cmd, REPO_ROOT, name):
            return False

    log.info("Coverage report available at: %s", html_dir)
    return True


def run_full_gate(fix_format: bool = False) -> bool:
    completed: list[str] = []
    lint_sources = get_lint_sources()
    steps: list[tuple[str, Callable[[], bool]]] = [
        ("clean", step_clean_build),
        ("configure", lambda: step_configure(enable_coverage=True)),
        ("build", step_build),
        ("test", step_ctest),
        ("clang-format", lambda: step_clang_format(lint_sources, fix=fix_format)),
        ("clang-tidy", lambda: step_clang_tidy(lint_sources)),
        ("coverage", step_coverage),
    ]

    success = True
    for name, step in steps:
        if not step():
            log.error("Step '%s' failed; aborting.", name)
            success = False
            break
        completed.append(name)

    log.info("Completed steps: %s", ", ".join(completed))
    return success


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Host-quality gate: build, test, lint, coverage."
    )
    parser.add_argument("-t", "--test", action="store_true", help="Run unit tests only")
    parser.add_argument(
        "-l",
        "--lint",
        action="store_true",
        help="Run formatting + static analysis only",
    )
    parser.add_argument(
        "-c", "--coverage", action="store_true", help="Generate coverage only"
    )
    parser.add_argument(
        "--fix-format",
        action="store_true",
        help="Automatically fix clang-format issues in source files",
    )

    args = parser.parse_args()

    if not (REPO_ROOT / "test" / "CMakeLists.txt").exists():
        log.error("test/CMakeLists.txt not found; run from repo root.")
        return 1

    if not any([args.test, args.lint, args.coverage]):
        success = run_full_gate(fix_format=args.fix_format)
    else:
        success = True
        if args.test:
            success &= step_ctest()
        if args.lint:
            lint_sources = get_lint_sources()
            success &= step_clang_format(lint_sources, fix=args.fix_format)
            success &= step_clang_tidy(lint_sources)
        if args.coverage:
            success &= step_coverage()

    if success:
        log.info("All requested steps completed successfully.")
        return 0
    else:
        log.error("One or more steps failed. See logs: %s", LOG_FILE)
        return 1


if __name__ == "__main__":
    sys.exit(main())
