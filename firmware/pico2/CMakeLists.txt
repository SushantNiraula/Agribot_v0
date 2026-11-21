# TEMPLATE: written from the micro-ROS Pico example layout; not yet build-tested
# on a clean checkout. Verify with the steps in README.md before tagging.
cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED ENV{PICO_SDK_PATH})
  message(FATAL_ERROR "Set PICO_SDK_PATH to your pico-sdk checkout.")
endif()
if(NOT DEFINED ENV{MICROROS_SDK_PATH})
  message(FATAL_ERROR "Set MICROROS_SDK_PATH to your micro_ros_raspberrypi_pico_sdk checkout (with libmicroros built).")
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(agribot_pico2_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

set(MICROROS_LIB_DIR $ENV{MICROROS_SDK_PATH}/libmicroros)
link_directories(${MICROROS_LIB_DIR})

add_executable(agribot_firmware src/main.cpp)

target_include_directories(agribot_firmware PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${MICROROS_LIB_DIR}/include
)

target_link_libraries(agribot_firmware
  pico_stdlib
  hardware_gpio
  hardware_pwm
  hardware_i2c
  hardware_timer
  microros
)

target_compile_options(agribot_firmware PRIVATE -ffunction-sections -fdata-sections)
target_link_options(agribot_firmware PRIVATE -Wl,--gc-sections)

# micro-ROS talks to the agent over USB CDC; UART stdio stays off.
pico_enable_stdio_usb(agribot_firmware 1)
pico_enable_stdio_uart(agribot_firmware 0)

pico_add_extra_outputs(agribot_firmware)
