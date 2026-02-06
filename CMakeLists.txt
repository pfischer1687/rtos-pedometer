cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

# Created this batch file for VS Code's clangd extension to use the compiler path without spaces.
# ```C:\arm-gcc\arm-none-eabi-gcc.bat
# @echo off
# "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.3 rel1\bin\arm-none-eabi-gcc.exe" %*
# ```
set(CMAKE_C_COMPILER "C:/arm-gcc/arm-none-eabi-gcc.bat")
set(CMAKE_CXX_COMPILER "C:/arm-gcc/arm-none-eabi-gcc.bat")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MBED_APP_JSON_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mbed_app.json5)

include(mbed-os/tools/cmake/mbed_toolchain_setup.cmake)
project(rtos-pedometer
    LANGUAGES C CXX ASM)
include(mbed_project_setup)

add_subdirectory(mbed-os)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(APP_SOURCES
    main.cpp
    app/Application.cpp
    platform/Platform.cpp
    imu/Mpu6050Driver.cpp
    signal_processing/SignalProcessing.cpp
    step_detection/StepDetector.cpp
    session/SessionManager.cpp
    usb/UsbInterface.cpp
    led/LedManager.cpp
)
add_executable(${PROJECT_NAME} ${APP_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/app
    ${CMAKE_CURRENT_SOURCE_DIR}/platform
    ${CMAKE_CURRENT_SOURCE_DIR}/imu
    ${CMAKE_CURRENT_SOURCE_DIR}/signal_processing
    ${CMAKE_CURRENT_SOURCE_DIR}/step_detection
    ${CMAKE_CURRENT_SOURCE_DIR}/session
    ${CMAKE_CURRENT_SOURCE_DIR}/usb
    ${CMAKE_CURRENT_SOURCE_DIR}/led
)

target_link_libraries(${PROJECT_NAME} mbed-os)
mbed_set_post_build(${PROJECT_NAME})
