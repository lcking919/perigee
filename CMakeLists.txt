cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico2_w CACHE STRING "Board type")
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(perigee C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(lib/fatfs-sd/FatFs_SPI)

add_executable(perigee
    app/main.c
    core/src/apogee.c
    core/src/vapogee.c
    core/src/boost.c
    core/src/burnout.c
    core/src/landing.c
    core/src/state.c
    drivers/src/bmp388.c
    drivers/src/mpu6050.c
    drivers/src/adxl375.c
    hal/pico/i2c_pico.c
    hal/pico/log_sd.c
    hal/pico/hw_config.c
)

target_include_directories(perigee PRIVATE
    core/include
    drivers/include
    hal/pico
)

target_link_libraries(perigee
    pico_stdlib
    hardware_i2c
    FatFs_SPI
)

pico_enable_stdio_usb(perigee 1)
pico_enable_stdio_uart(perigee 0)

pico_add_extra_outputs(perigee)