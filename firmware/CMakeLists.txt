cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w)
set(PICO_PLATFORM rp2040)
set(PICO_COMPILER pico_arm_cortex_m0plus_gcc)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(hootswitch)

pico_sdk_init()

add_executable(hootswitch)

pico_generate_pio_header(hootswitch ${CMAKE_CURRENT_LIST_DIR}/bus.pio)

# for FreeRTOS to find the configuration file
target_include_directories(hootswitch PRIVATE ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(drivers)

target_sources(hootswitch PRIVATE
		buzzer.c computer.c debug.c driver.c handler.c host.c host_sync.c led.c main.c util.c
		)

target_link_libraries(hootswitch
		pico_async_context_freertos FreeRTOS-Kernel-Heap4
		pico_stdlib pico_rand pico_cyw43_arch_none
		hardware_dma hardware_pio hardware_pwm
		)

pico_enable_stdio_usb(hootswitch 1)
pico_enable_stdio_uart(hootswitch 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(hootswitch)
