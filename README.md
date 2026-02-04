# rtos-pedometer

RTOS-based pedometer firmware featuring deterministic execution, interrupt-driven IMU sampling, and real-time step detection on STM32 Nucleo-F767ZI with an MPU-6050.

## Build (Mbed OS Community Edition)

This project uses [Mbed CE](https://mbed-ce.dev/). Add Mbed OS as a submodule, then build:

```bash
git submodule add --depth 1 https://github.com/mbed-ce/mbed-os.git mbed-os
# Configure and build (e.g. with CMake + your toolchain, or via PlatformIO/IDE)
```

See [Creating a Project](https://mbed-ce.dev/getting-started/creating-a-project/) for full setup.
