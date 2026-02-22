cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)
cmake_policy(SET CMP0135 NEW)

project(rtos-pedometer-tests LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_COMPILER "C:/Program Files/LLVM/bin/clang.exe")
set(CMAKE_CXX_COMPILER "C:/Program Files/LLVM/bin/clang++.exe")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(COVERAGE_FLAGS "-fprofile-instr-generate -fcoverage-mapping")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COVERAGE_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${COVERAGE_FLAGS}")

set(FIRMWARE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(TEST_MAIN ${CMAKE_CURRENT_SOURCE_DIR}/test_main.cpp)

set(THREADS_PREFER_PTHREAD_FLAG OFF)

set(HOST_SOURCES
  ${FIRMWARE_ROOT}/imu/Mpu6050Driver.cpp
  ${FIRMWARE_ROOT}/led/RecordingLed.cpp
  ${FIRMWARE_ROOT}/session/SessionManager.cpp
  ${FIRMWARE_ROOT}/signal_processing/SignalProcessing.cpp
  ${FIRMWARE_ROOT}/step_detection/StepDetector.cpp
  ${FIRMWARE_ROOT}/usb/UsbInterface.cpp
)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.17.0
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
enable_testing()

add_executable(runUnitTests
  ${HOST_SOURCES}
  ${TEST_MAIN}
  ${TEST_FILES}
)

file(GLOB TEST_FILES "${CMAKE_CURRENT_SOURCE_DIR}/tests/**/*.cpp")
target_sources(runUnitTests PRIVATE ${TEST_FILES})

target_include_directories(runUnitTests PRIVATE
  ${FIRMWARE_ROOT}
  ${FIRMWARE_ROOT}/app
  ${FIRMWARE_ROOT}/imu
  ${FIRMWARE_ROOT}/led
  ${FIRMWARE_ROOT}/platform
  ${FIRMWARE_ROOT}/session
  ${FIRMWARE_ROOT}/signal_processing
  ${FIRMWARE_ROOT}/step_detection
  ${FIRMWARE_ROOT}/usb
)

find_package(Threads REQUIRED)
target_link_libraries(runUnitTests PRIVATE GTest::gtest_main Threads::Threads)

include(GoogleTest)
gtest_discover_tests(runUnitTests)
