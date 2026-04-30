"""Stream DSP debug CSV from firmware over serial into data/debug_<timestamp>.csv.

Run from the repository root, for example:
    python tools/debug_stream.py --port COM5
    python tools/debug_stream.py --port COM5 --baud 115200
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path
from tools.hitl.transport import SerialTransport

_REPO_ROOT = Path(__file__).resolve().parents[1]
CSV_HEADER = "timestampUs,ax,ay,az,mag,slope\n"
FLUSH_EVERY_N = 50


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
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    data_dir = _REPO_ROOT / "data"
    data_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = data_dir / f"debug_{stamp}.csv"

    transport = SerialTransport(args.port, baud=args.baud, timeout_s=0.3)
    outfile = None
    lines_since_flush = 0

    try:
        transport.open()
        transport.drain()
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
            outfile.write(line.strip() + "\n")
            lines_since_flush += 1
            if lines_since_flush >= FLUSH_EVERY_N:
                outfile.flush()
                lines_since_flush = 0
    except KeyboardInterrupt:
        print(file=sys.stderr)
    finally:
        _send_stop_safe(transport)
        if outfile is not None:
            outfile.flush()
            outfile.close()
        if transport.is_open():
            transport.close()
        if outfile is not None:
            print(str(out_path.resolve()), flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
