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

# for FreeRTOS/TinyUSB to find the configuration file
target_include_directories(hootswitch PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(hootswitch PRIVATE ${CMAKE_CURRENT_LIST_DIR}/usb)
target_include_directories(hootswitch PRIVATE ${CMAKE_CURRENT_LIST_DIR}/drivers)
target_include_directories(hootswitch PRIVATE ${CMAKE_CURRENT_LIST_DIR}/wireless)
target_include_directories(hootswitch PRIVATE ${CMAKE_CURRENT_LIST_DIR}/lib)

# generates .su files for stack analysis if needed
# target_compile_options(hootswitch PRIVATE -fstack-usage)

target_compile_definitions(hootswitch PRIVATE
		CYW43_ENABLE_BLUETOOTH=1
		CYW43_LWIP=0
		# prevents an assertion issue with btstack TLV key storage
		# this will need to be evaluated if core1 ever becomes used
		PICO_FLASH_ASSUME_CORE1_SAFE=1
		# grant a large stack to the async_context, bluepad32/btstack run in
		# it; smaller values (~1024) were causing overflows
		CYW43_TASK_STACK_SIZE=2048
        )

add_subdirectory(drivers)
add_subdirectory(wireless)
add_subdirectory(usb)

target_include_directories(hootswitch PRIVATE
		$ENV{BLUEPAD32_ROOT}/src/components/bluepad32/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/wireless)

target_sources(hootswitch PRIVATE
		button.c buzzer.c computer.c config.c control.c debug.c driver.c handler.c host.c host_sync.c led.c main.c util.c
		)

target_link_libraries(hootswitch
		FreeRTOS-Kernel FreeRTOS-Kernel-Heap4
		tinyusb_board tinyusb_device
		pico_stdlib pico_rand
		pico_cyw43_arch_sys_freertos
		pico_btstack_classic pico_btstack_cyw43
		bluepad32
		hardware_dma hardware_pio hardware_pwm
		)

include_directories(${CMAKE_CURRENT_LIST_DIR}) # for FreeRTOSConfig.h below
add_subdirectory($ENV{BLUEPAD32_ROOT}/src/components/bluepad32 libbluepad32)

pico_enable_stdio_usb(hootswitch 1)
pico_enable_stdio_uart(hootswitch 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(hootswitch)
