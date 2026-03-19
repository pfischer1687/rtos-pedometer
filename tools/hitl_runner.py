#!/usr/bin/env python3
"""Top-level entry point for running HITL tests.

Usage example:
    python tools/hitl_runner.py [--port COM5] [options]
"""

import argparse
import sys
from tools.hitl.runner import HitlRunner, RunnerConfig
from tools.hitl.build import BuildManager
from pathlib import Path
import logging
from tools.common.logging import setup_logging

LOGGER_NAME = "hitl_runner"
REPO_ROOT = Path(__file__).resolve().parents[1]


def _logger() -> logging.Logger:
    """Get the logger for the HITL runner."""
    return logging.getLogger(LOGGER_NAME)


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for the HITL runner.

    Returns:
        Parsed namespace with port, baud_rate, sample_count, timeout, etc.
    """
    parser = argparse.ArgumentParser(description="Run HITL tests for RTOS Pedometer.")
    parser.add_argument(
        "--port",
        type=str,
        required=True,
        help="Serial port (e.g. COM5).",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=100,
        help="Number of IMU samples to collect.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Overall timeout for the run in seconds.",
    )
    parser.add_argument(
        "--no-flash",
        action="store_true",
        help="If set, do not build and flash before running.",
    )
    return parser.parse_args()


def main() -> int:
    """Entry point: parse args, run HITL, exit with 0 on success else 1."""
    args = parse_args()

    log_file = setup_logging(LOGGER_NAME, REPO_ROOT)
    log = _logger()

    log.info("Starting HITL runner")
    log.info(
        "Config: port=%s baud=%d samples=%d timeout=%.1f no_flash=%s",
        args.port,
        args.baud,
        args.samples,
        args.timeout,
        args.no_flash,
    )

    if args.samples <= 0:
        log.error("--samples must be > 0")
        log.info("Log file: %s", log_file)
        return 3
    if args.timeout <= 0:
        log.error("--timeout must be > 0")
        log.info("Log file: %s", log_file)
        return 3

    config = RunnerConfig(
        port=args.port,
        baud_rate=args.baud,
        sample_count=args.samples,
        timeout=args.timeout,
        no_flash=args.no_flash,
    )
    runner = HitlRunner(config=config)

    try:
        if not config.no_flash:
            build_manager = BuildManager(project_root=REPO_ROOT)
            log.info("Building and flashing firmware...")
            build_manager.build_and_flash()

        result = runner.run()
    except Exception:
        log.exception("Infrastructure failure")
        log.info("Log file: %s", log_file)
        return 2

    if result.success:
        log.info("HITL run passed: %s", result.message)
        log.info("Log file: %s", log_file)
        return 0
    log.error("HITL run failed: %s", result.message)
    log.info("Log file: %s", log_file)
    return 1


if __name__ == "__main__":
    sys.exit(main())
