cmake_minimum_required(VERSION 3.18)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(rp2350_efi_uav C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

pico_generate_pio_header(rp2350_efi_uav
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pio/crank_trigger.pio
)

add_executable(rp2350_efi_uav
    src/main.cpp
    src/drivers/crank_sensor.cpp
    src/drivers/adc_sensors.cpp
    src/drivers/uav_sensors.cpp       # [NEW] MS5611 BAP + MAX31855 EGT
    src/fusion/kalman_filter.cpp      # [MOD] 3-state MAP+BAP KF + EGT KF
    src/efi/fuel_calc.cpp             # [MOD] Alpha-N blend + EGT enrichment
)

target_include_directories(rp2350_efi_uav PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(rp2350_efi_uav
    pico_stdlib
    pico_multicore
    hardware_adc
    hardware_dma
    hardware_gpio
    hardware_irq
    hardware_pio
    hardware_pwm
    hardware_spi        # [NEW] for MS5611 + MAX31855
    hardware_timer
    hardware_uart
    hardware_clocks
    m
)

pico_add_extra_outputs(rp2350_efi_uav)

target_compile_options(rp2350_efi_uav PRIVATE
    -Wall -Wextra -O2 -ffast-math -DPICO_RP2350=1
)

pico_enable_stdio_usb (rp2350_efi_uav 0)
pico_enable_stdio_uart(rp2350_efi_uav 1)

message(STATUS "rp2350_efi_uav (UAV variant) configured for RP2350")
