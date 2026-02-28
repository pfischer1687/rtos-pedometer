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

## Host-quality gate (CI / local checks)

From the repo root, use Python 3.14 and uv to run the full gate (build, test, lint, coverage) or individual steps:

```bash
uv venv --python 3.14
uv pip install -r requirements.txt
uv run tools/run_host_checks.py [--test/-t] [--lint/-l] [--coverage/-c]
```

- **No flags**: full gate (clean `test/build`, configure with host-check preset, build, run unit tests, clang-format check, clang-tidy, lcov coverage).
- **`-t` / `--test`**: run unit tests only (configures and builds with host-check if `test/build` is missing).
- **`-l` / `--lint`**: run clang-format and clang-tidy on app sources only (no build).
- **`-c` / `--coverage`**: generate coverage report via lcov (assumes tests were run with coverage enabled).

Flags can be combined (e.g. `-t -l`).

## References

- [Mbed CE Documentation](https://mbed-ce.dev)
- [STM32 Nucleo-F767ZI Documentation](https://www.st.com/en/evaluation-tools/nucleo-f767zi.html#documentation)
- [STM32F767ZIT6 Documentation](https://www.digikey.com/en/products/detail/stmicroelectronics/STM32F767ZIT6/6004736)
- [MPU-6050 Product Details](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [MPU-6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU-6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
