cmake_minimum_required(VERSION 3.13)

pico_generate_pio_header(hootswitch ${CMAKE_CURRENT_LIST_DIR}/bus.pio)

# for FreeRTOS/TinyUSB to find the configuration file
target_include_directories(hootswitch PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(hootswitch PRIVATE ${CMAKE_CURRENT_LIST_DIR}/usb)
target_include_directories(hootswitch PRIVATE ${CMAKE_CURRENT_LIST_DIR}/drivers)
target_include_directories(hootswitch PRIVATE ${CMAKE_CURRENT_LIST_DIR}/lib)

add_subdirectory(drivers)
add_subdirectory(usb)

if (DEFINED HOOTSWITCH_WIRELESS)
	target_include_directories(hootswitch PRIVATE ${CMAKE_CURRENT_LIST_DIR}/wireless)
	include_directories(${CMAKE_CURRENT_LIST_DIR}/wireless)
	add_subdirectory(wireless)
endif()

target_sources(hootswitch PRIVATE
		button.c
		buzzer.c
		computer.c
		config.c
		control.c
		debug.c
		driver.c
		handler.c
		host.c
		host_sync.c
		keymap.c
		led.c
		main.c
		util.c
		)
