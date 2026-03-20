"""Logging setup for the project."""

import logging
from pathlib import Path
from datetime import datetime

HITL_LOGGER_NAME = "hitl_runner"


def setup_logging(name: str, repo_root: Path, log_subdir: str = ".logs") -> Path:
    """Setup logging for the project."""
    log_dir = repo_root / log_subdir
    log_dir.mkdir(exist_ok=True)

    log_file = log_dir / f"{name}_{datetime.now():%Y%m%d_%H%M%S}.log"

    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    logger.handlers.clear()

    formatter = logging.Formatter(
        "%(asctime)s %(levelname)s [%(name)s]: %(message)s",
        datefmt="%H:%M:%S",
    )

    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(formatter)

    fh = logging.FileHandler(log_file, encoding="utf-8")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(formatter)

    logger.addHandler(ch)
    logger.addHandler(fh)

    return log_file
