# rtos-pedometer

RTOS-based pedometer firmware featuring deterministic execution, interrupt-driven IMU sampling, and real-time step
detection on STM32 Nucleo-F767ZI with an MPU-6050.

## Features

- Real-time IMU sampling via MPU-6050 interrupt pin (data-ready signals)
- Step detection using a threshold and timing algorithm
- Session management: start, pause, resume, stop
- Signal processing pipeline: high-pass and low-pass filtering of accelerometer data
- USB command interface for telemetry and session control
- LED feedback for recording and status indication
- Watchdog integration for system reliability

## Hardware

- MCU: STM32 Nucleo-F767ZI (Cortex-M7 @ 216 MHz)
- Sensor: MPU-6050 IMU
- LED: On-board red LED for recording indication
- USB: Debug and CDC interface for commands and telemetry

## Adding Mbed CE submodule

```bash
git submodule add --depth 1 https://github.com/mbed-ce/mbed-os.git mbed-os
```

## Usage

- Plug in USB to PC for debugging/command interface.
- LED indicates recording status:
  - Off/Slow blink: idle
  - Solid: recording
  - Fast blink: error
- Commands (over USB CDC interface):
  - `START`: begin session
  - `STOP`: end session
  - `STATUS`: query current session metrics
  - `RESET`: reset step count and session

## Debugging Unit Tests on Host Device

Unit tests are built and debugged on the host using CMake and LLDB.

From the repo root run:

```powershell
.\scripts\host-debug.ps1
```

This will:

- Clean `test/build`
- Configure using the `host-debug` CMake preset
- Build the Debug configuration
- Switch VS Code to the host debug launch configuration

Then set your breeakpoints and press F5.

## Tools

There are Python utility scripts for unit and HITL (Hardware In The Loop) testing. You can build
your virtual environment with [uv](https://docs.astral.sh/uv/):

```sh
uv venv --python 3.14
.\.venv\Scripts\activate
uv pip install -r requirements.txt
```

### Host-quality gate (CI / local checks)

From the repo root, use Python 3.14 and uv to run the full gate (build, test, lint, coverage) or individual steps:

```bash
uv run tools/run_host_checks.py [--test/-t] [--lint/-l] [--coverage/-c]
```

- **No flags**: full gate (clean `test/build`, configure with host-check preset, build, run unit tests, clang-format check, clang-tidy, lcov coverage).
- **`-t` / `--test`**: run unit tests only (configures and builds with host-check if `test/build` is missing).
- **`-l` / `--lint`**: run clang-format and clang-tidy on app sources only (no build).
- **`-c` / `--coverage`**: generate coverage report via lcov (assumes tests were run with coverage enabled).

Flags can be combined (e.g. `-t -l`).

### Running HITL Tests

The Hardware-in-the-Loop (HITL) test runner builds, flashes, and runs automated tests of firmware functionality on hardware.

#### Basic Usage

```bash
python tools/hitl_runner.py --port <SERIAL_PORT>
```

Example:

```bash
python tools/hitl_runner.py --port COM5
```

This builds/flashes the firmware and runs 1 iteration with 100 IMU samples by default.

#### Optional Arguments

| Argument     | Description                                             |
| ------------ | ------------------------------------------------------- |
| `--port`     | Serial port of the target board (required)              |
| `--baud`     | Serial baud rate (default: 115200)                      |
| `--samples`  | Number of IMU samples per iteration (default: 100)      |
| `--timeout`  | Timeout in seconds for reading IMU data (default: 30.0) |
| `--no-flash` | Skip build/flash; only run HITL tests                   |
| `--verbose`  | Set the logging verbosity from `INFO` to `DEBUG`        |

Example:

```bash
python tools/hitl_runner.py --port /dev/ttyUSB0 --baud 115200 --samples 200 --timeout 60 --no-flash -v
```

#### Exit Codes

| Code | Meaning                                                |
| ---- | ------------------------------------------------------ |
| `0`  | HITL tests passed                                      |
| `1`  | HITL tests failed (validation/protocol error)          |
| `2`  | Infrastructure failure (build, flash, or serial error) |
| `3`  | Invalid command-line arguments                         |

### Log Output

Logs for unit and HITL tests are saved to a timestamped file. Location is shown at the end of the run (something like `.logs/hitl_runner_YYYY-MM-DD_HH-MM-SS.log`)

## References

- [Mbed CE Documentation](https://mbed-ce.dev)
- [STM32 Nucleo-F767ZI Documentation](https://www.st.com/en/evaluation-tools/nucleo-f767zi.html#documentation)
- [STM32F767ZIT6 Documentation](https://www.digikey.com/en/products/detail/stmicroelectronics/STM32F767ZIT6/6004736)
- [MPU-6050 Product Details](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [MPU-6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU-6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
