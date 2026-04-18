cmake_minimum_required(VERSION 3.13)

if (NOT DEFINED PICO_BOARD)
	set(PICO_BOARD pico_w)
endif()

set(PICO_COMPILER pico_arm_cortex_m0plus_gcc)

# if user has installed pico-sdk separately use that instead
if (DEFINED ENV{PICO_SDK_PATH})
	set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
else()
	set(PICO_SDK_PATH ${CMAKE_SOURCE_DIR}/external/pico-sdk)
endif()

set(FREERTOS_PATH ${CMAKE_SOURCE_DIR}/external/FreeRTOS-Kernel)
set(BLUEPAD32_PATH ${CMAKE_SOURCE_DIR}/external/bluepad32/src/components/bluepad32)

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(${FREERTOS_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(hootswitch)

pico_sdk_init()

add_executable(hootswitch)

# generates .su files for stack analysis if needed
# target_compile_options(hootswitch PRIVATE -fstack-usage)

# use higher clock speed supported officially since SDK 2.1.1+
target_compile_definitions(hootswitch PRIVATE SYS_CLK_MHZ=200)

if (${PICO_BOARD} STREQUAL pico_w)
	message("setting build for pico_w")
	set(HOOTSWITCH_WIRELESS 1)
	target_compile_definitions(hootswitch PRIVATE
			HOOTSWITCH_WIRELESS
			CYW43_ENABLE_BLUETOOTH=1
			CYW43_LWIP=0
			# prevents an assertion issue with btstack TLV key storage
			# this will need to be evaluated if core1 ever becomes used
			PICO_FLASH_ASSUME_CORE1_SAFE=1
			# grant a large stack to the async_context, bluepad32/btstack run in
			# it; smaller values (~1024) were causing overflows
			CYW43_TASK_STACK_SIZE=2048
			)
elseif (${PICO_BOARD} STREQUAL pico)
	message("setting build for pico")
else()
	message(FATAL_ERROR "unsupported PICO_BOARD " ${PICO_BOARD})
endif()

if (DEFINED HOOTSWITCH_WIRELESS)
	target_include_directories(hootswitch PRIVATE
			${BLUEPAD32_PATH}/include)
endif()

if (RUNTIME_FREERTOS_REPORTING EQUAL 1)
	message("including runtime FreeRTOS reporting")
	target_compile_definitions(hootswitch PRIVATE RUNTIME_FREERTOS_REPORTING)
endif()

add_subdirectory(firmware)

target_link_libraries(hootswitch
		FreeRTOS-Kernel FreeRTOS-Kernel-Heap4
		tinyusb_board tinyusb_device
		pico_stdlib pico_rand
		hardware_dma hardware_pio hardware_pwm
		)
if (DEFINED HOOTSWITCH_WIRELESS)
	target_link_libraries(hootswitch
			pico_cyw43_arch_sys_freertos
			pico_btstack_classic pico_btstack_cyw43
			bluepad32
			)
	add_subdirectory(${CMAKE_SOURCE_DIR}/external/bluepad32-freertos)
	target_include_directories(bluepad32 PRIVATE
			${CMAKE_SOURCE_DIR}/firmware) # for FreeRTOSConfig.h
	target_include_directories(bluepad32 PRIVATE
			${CMAKE_SOURCE_DIR}/firmware/wireless) # for sdkconfig.h
endif()

pico_enable_stdio_usb(hootswitch 1)
pico_enable_stdio_uart(hootswitch 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(hootswitch)
