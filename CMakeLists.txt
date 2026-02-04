cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

set(MBED_APP_JSON_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mbed_app.json5)
include(mbed-os/tools/cmake/mbed_toolchain_setup.cmake)

project(rtos-pedometer
    LANGUAGES C CXX ASM)

include(mbed_project_setup)
add_subdirectory(mbed-os)

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
)

target_link_libraries(${PROJECT_NAME} mbed-os)
mbed_set_post_build(${PROJECT_NAME})
