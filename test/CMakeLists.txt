# Host-based unit tests: native build (clang), no Mbed OS.
# Configure from this directory, e.g.:
#   cmake -S test -B test/build -G Ninja -DCMAKE_CXX_COMPILER=clang++
#   cmake --build test/build

cmake_minimum_required(VERSION 3.19)
cmake_policy(SET CMP0135 NEW)  # CMAKE_ROLE for FetchContent

project(rtos-pedometer-tests
  LANGUAGES CXX
  DESCRIPTION "Host unit tests for rtos-pedometer (no Mbed)"
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Generate compile_commands.json for IDE/clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -----------------------------------------------------------------------------
# Options
# -----------------------------------------------------------------------------
option(ENABLE_SANITIZERS "Build with Address and UndefinedBehavior sanitizers" OFF)
option(ENABLE_COVERAGE "Build with code coverage (--coverage)" OFF)

# -----------------------------------------------------------------------------
# Project root (parent of test/) for platform-independent sources and includes
# -----------------------------------------------------------------------------
set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# -----------------------------------------------------------------------------
# GoogleTest via FetchContent (official Quickstart approach)
# -----------------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/v1.14.0.zip
)
# Windows: build GTest with shared CRT to match host executable and avoid link errors
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# Disable GTest install (not needed for FetchContent usage)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# -----------------------------------------------------------------------------
# Platform-independent sources (no Mbed; platform/*.cpp are target-specific).
# Use host-only driver impl so tests link and run without modifying firmware.
# -----------------------------------------------------------------------------
set(PLATFORM_INDEPENDENT_SOURCES
  ${PROJECT_ROOT}/signal_processing/SignalProcessing.cpp
  ${PROJECT_ROOT}/step_detection/StepDetector.cpp
  ${PROJECT_ROOT}/session/SessionManager.cpp
)
set(HOST_DRIVER_SOURCES
  imu/Mpu6050DriverHostImpl.cpp
)

# -----------------------------------------------------------------------------
# Test executable (existing test file still has main(); GTest infra ready for conversion)
# -----------------------------------------------------------------------------
add_executable(runTests
  ${PLATFORM_INDEPENDENT_SOURCES}
  ${HOST_DRIVER_SOURCES}
  imu/Mpu6050DriverTests.cpp
)

target_include_directories(runTests PRIVATE
  ${PROJECT_ROOT}
)

# Link GTest and gtest_main (test file uses TEST() macros and no custom main())
target_link_libraries(runTests PRIVATE GTest::gtest GTest::gtest_main)

# -----------------------------------------------------------------------------
# Sanitizers (Clang/GCC; Windows Clang supports ASan with recent versions)
# -----------------------------------------------------------------------------
if(ENABLE_SANITIZERS)
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(runTests PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(runTests PRIVATE
      -fsanitize=address,undefined
    )
  endif()
endif()

# -----------------------------------------------------------------------------
# Coverage
# -----------------------------------------------------------------------------
if(ENABLE_COVERAGE)
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(runTests PRIVATE --coverage)
    target_link_options(runTests PRIVATE --coverage)
  endif()
endif()

# -----------------------------------------------------------------------------
# CTest
# -----------------------------------------------------------------------------
enable_testing()
add_test(NAME HostTests COMMAND runTests)
