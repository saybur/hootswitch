cmake_minimum_required(VERSION 3.12)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
set(PICO_BOARD pico_w)

project(hootswitch)

pico_sdk_init()

add_executable(hootswitch)

pico_generate_pio_header(hootswitch ${CMAKE_CURRENT_LIST_DIR}/bus.pio)

target_sources(hootswitch PRIVATE
		drivers/keyboard.c drivers/mouse.c
		bus.c buzzer.c computer.c debug.c driver.c handler.c host.c host_sync.c led.c manager.c main.c util.c
		)

target_link_libraries(hootswitch
		pico_stdlib pico_rand pico_cyw43_arch_none hardware_pio hardware_pwm
		)

pico_enable_stdio_usb(hootswitch 1)
pico_enable_stdio_uart(hootswitch 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(hootswitch)
