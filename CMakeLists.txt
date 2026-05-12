cmake_minimum_required(VERSION 3.28.3)

include(../pico-sdk/external/pico_sdk_import.cmake)

project(clock_machine)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

add_executable(clock_machine main.c ssd1306.c)
pico_set_binary_type(clock_machine copy_to_ram)

target_link_libraries(clock_machine pico_stdlib
    hardware_gpio
    hardware_spi
    hardware_timer
    hardware_irq
    pico_time
    hardware_i2c
    hardware_adc
    )

pico_add_extra_outputs(clock_machine)

pico_enable_stdio_usb(clock_machine 1)
pico_enable_stdio_uart(clock_machine 0)
