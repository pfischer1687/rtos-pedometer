# Host-based unit tests

Native (host) unit tests for platform-independent code. No Mbed OS, no target toolchain.

## Requirements

- **CMake** 3.19+
- **Clang** (recommended on Windows with Ninja) or GCC
- **Ninja** (recommended) or another generator

## Build (Windows + Ninja + Clang)

From the **repository root**:

```bat
cmake -S test -B test/build -G Ninja -DCMAKE_CXX_COMPILER=clang++
cmake --build test/build
```

Run tests:

```bat
test\build\runTests.exe
```

Or via CTest:

```bat
cd test\build
ctest -C Debug -V
```

## Build (other generators / compilers)

- **Visual Studio** (clang-cl):  
  `cmake -S test -B test/build -G "Visual Studio 17 2022" -A x64 -T ClangCL`

- **Ninja + GCC**:  
  `cmake -S test -B test/build -G Ninja -DCMAKE_CXX_COMPILER=g++`

## Options

| Option | Default | Description |
|--------|---------|-------------|
| `ENABLE_SANITIZERS` | `OFF` | Build with Address and UndefinedBehavior sanitizers (Clang/GCC). |
| `ENABLE_COVERAGE` | `OFF` | Build with `--coverage` for code coverage. |

Examples:

```bat
cmake -S test -B test/build -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DENABLE_SANITIZERS=ON
cmake -S test -B test/build -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DENABLE_COVERAGE=ON
```

## compile_commands.json

Generated in the build directory (e.g. `test/build/compile_commands.json`) for use with clangd or other tools. Symlink or copy it to the repo root if you want the IDE to use it for the whole tree.

## What is built

- **Sources**: `signal_processing/`, `step_detection/`, `session/` from the project root (platform-independent only), plus **host-only** `test/imu/Mpu6050DriverHostImpl.cpp` instead of `imu/Mpu6050Driver.cpp` so the driver under test links and runs without Mbed.
- **Not built**: `platform/*.cpp` (Mbed-specific); tests use mocks for I2C and tick source.
- **Test executable**: `runTests` (uses a global `main()` that calls into the test runner; GoogleTest is available for a future conversion).

## Converting tests to GoogleTest

The test CMake already fetches GoogleTest and links `runTests` to `GTest::gtest`. When you switch the test file to `TEST()` macros, add `GTest::gtest_main` so the default `main()` is used and remove the custom `main()` from the test file.
