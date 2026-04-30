"""Stream DSP debug CSV from firmware over serial into data/debug_<timestamp>.csv.

Run from the repository root, for example:
    python tools/debug_stream.py --port COM5
    python tools/debug_stream.py --port COM5 --baud 115200
"""

from __future__ import annotations

import argparse
import logging
from datetime import datetime
from pathlib import Path

from tools.common.logging import setup_logging, DSP_DEBUG_STREAM_LOGGER_NAME
from tools.hitl.transport import SerialTransport

_REPO_ROOT = Path(__file__).resolve().parents[1]
CSV_HEADER = "timestampUs,ax,ay,az,mag,slope\n"
FLUSH_EVERY_N = 50


def _logger() -> logging.Logger:
    return logging.getLogger(DSP_DEBUG_STREAM_LOGGER_NAME)


def _is_noise_line(line: str) -> bool:
    """Lines to drop before CSV parsing (firmware prompts / status)."""
    s = line.strip()
    if s == "OK" or s == ">":
        return True
    if s.startswith("DEBUG_"):
        return True
    return False


def _is_valid_dsp_csv(line: str) -> bool:
    """True only for timestampUs (unsigned int) + five float fields, six columns."""
    if _is_noise_line(line):
        return False
    parts = line.split(",")
    if len(parts) != 6:
        return False
    ts = parts[0].strip()
    if not ts.isdigit():
        return False
    try:
        for p in parts[1:]:
            float(p.strip())
    except ValueError:
        return False
    return True


def _send_stop_safe(transport: SerialTransport) -> None:
    if not transport.is_open():
        return
    try:
        transport.write_line("STOP")
    except (OSError, RuntimeError):
        pass


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Log DSP debug CSV from serial.")
    p.add_argument("--port", type=str, required=True, help="Serial port (e.g. COM5).")
    p.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200).",
    )
    p.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable verbose logging (DEBUG on console).",
    )
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    log_file = setup_logging(DSP_DEBUG_STREAM_LOGGER_NAME, _REPO_ROOT)
    log = _logger()

    if args.verbose:
        for handler in log.handlers:
            if isinstance(handler, logging.StreamHandler):
                handler.setLevel(logging.DEBUG)

    data_dir = _REPO_ROOT / "data"
    data_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = data_dir / f"debug_{stamp}.csv"

    log.info("Starting DSP debug stream")
    log.info("Port=%s baud=%d", args.port, args.baud)
    log.info("Output file: %s", out_path)
    log.info("Press Ctrl+C to stop")

    transport = SerialTransport(args.port, baud=args.baud, timeout_s=0.3)
    outfile = None
    lines_since_flush = 0
    received_any = False
    total_lines = 0

    try:
        log.debug("Opening serial transport")
        transport.open()
        transport.drain()
        log.debug("Sending DEBUG_START")
        transport.write_line("DEBUG_START")

        outfile = out_path.open("w", encoding="utf-8", newline="")
        outfile.write(CSV_HEADER)
        outfile.flush()

        read_timeout = 0.5
        while True:
            try:
                line = transport.read_line(timeout_s=read_timeout)
            except TimeoutError:
                continue
            if not _is_valid_dsp_csv(line):
                continue
            if not received_any:
                log.info("Receiving valid DSP data...")
                received_any = True
            outfile.write(line.strip() + "\n")
            total_lines += 1
            if total_lines % 500 == 0:
                log.info("Collected %d samples...", total_lines)
            lines_since_flush += 1
            if lines_since_flush >= FLUSH_EVERY_N:
                outfile.flush()
                log.debug("Flushed %d lines (total=%d)", FLUSH_EVERY_N, total_lines)
                lines_since_flush = 0
    except KeyboardInterrupt:
        log.info("Interrupted by user (Ctrl+C)")
    finally:
        _send_stop_safe(transport)
        if outfile is not None:
            outfile.flush()
            outfile.close()
        if transport.is_open():
            transport.close()
        if total_lines == 0:
            log.warning("No valid DSP data received")
        else:
            log.info("Finished: collected %d samples", total_lines)
        log.info("CSV saved to: %s", out_path.resolve())
        log.info("Log file: %s", log_file)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
