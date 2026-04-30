cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)
cmake_policy(SET CMP0135 NEW)

project(rtos-pedometer-tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(ENABLE_COVERAGE "Enable LLVM coverage (fprofile-instr-generate, fcoverage-mapping)" OFF)

if(ENABLE_SANITIZERS AND ENABLE_COVERAGE)
  message(FATAL_ERROR "ENABLE_SANITIZERS and ENABLE_COVERAGE cannot both be ON. Disable one.")
endif()
if(ENABLE_SANITIZERS AND WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
endif()

function(configure_instrumentation target)
  if(ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(${target} PRIVATE
      -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=address -fsanitize=undefined)
  elseif(ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(${target} PRIVATE -fprofile-instr-generate -fcoverage-mapping)
    target_link_options(${target} PRIVATE -fprofile-instr-generate -fcoverage-mapping)
  endif()
endfunction()

function(configure_clang_runtime target)
  if(NOT ENABLE_SANITIZERS OR NOT WIN32 OR NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    return()
  endif()
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} --print-runtime-dir
    OUTPUT_VARIABLE _dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(NOT _dir OR NOT EXISTS "${_dir}")
    return()
  endif()
  set(CLANG_SANITIZER_RUNTIME_DIR "${_dir}" PARENT_SCOPE)
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${_dir}" "$<TARGET_FILE_DIR:${target}>"
    COMMENT "Copying Clang sanitizer runtime DLLs for ${target}"
  )
endfunction()

set(FIRMWARE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
file(GLOB_RECURSE SUPPORT_MOCK_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/support/mocks/*.cpp")
file(GLOB_RECURSE TEST_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
set(HOST_SOURCES
  ${FIRMWARE_ROOT}/imu/Mpu6050Driver.cpp
  ${FIRMWARE_ROOT}/signal_processing/SignalProcessing.cpp
  ${FIRMWARE_ROOT}/step_detection/OscillationTracker.cpp
)

include(FetchContent)
FetchContent_Declare(googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.17.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

if(ENABLE_SANITIZERS)
  configure_instrumentation(gtest)
  configure_instrumentation(gtest_main)
endif()
include(GoogleTest)
enable_testing()

add_executable(runUnitTests
  ${HOST_SOURCES}
  ${SUPPORT_MOCK_SOURCES}
  ${CMAKE_CURRENT_SOURCE_DIR}/test_main.cpp
  ${TEST_FILES}
)

target_include_directories(runUnitTests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/support
  ${CMAKE_CURRENT_SOURCE_DIR}/support/mocks
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

configure_instrumentation(runUnitTests)
configure_clang_runtime(runUnitTests)

if(ENABLE_SANITIZERS)
  add_test(NAME runUnitTests COMMAND runUnitTests)
  if(CLANG_SANITIZER_RUNTIME_DIR)
    set_tests_properties(runUnitTests PROPERTIES ENVIRONMENT "PATH=${CLANG_SANITIZER_RUNTIME_DIR};$ENV{PATH}")
  endif()
else()
  gtest_discover_tests(runUnitTests)
endif()
