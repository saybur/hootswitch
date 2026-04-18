cmake_minimum_required(VERSION 3.13)

# Handles building Bluepad32 with FreeRTOS support for the Pico W Hootswitch
# (pico_cyw43_arch_sys_freertos, FreeRTOS-Kernel-Heap4, and no lwip).
#
# This is likely not the most elegant way to handle things, please open a PR
# if there is a better approach that doesn't require maintaining a separate
# CMakeLists.txt file.

add_library(bluepad32)
target_sources(bluepad32 PRIVATE
		${BLUEPAD32_PATH}/arch/uni_console_pico.c
		${BLUEPAD32_PATH}/arch/uni_system_pico.c
		${BLUEPAD32_PATH}/arch/uni_log_pico.c
		${BLUEPAD32_PATH}/arch/uni_property_pico.c
		${BLUEPAD32_PATH}/bt/uni_bt.c
		${BLUEPAD32_PATH}/bt/uni_bt_bredr.c
		${BLUEPAD32_PATH}/bt/uni_bt_allowlist.c
		${BLUEPAD32_PATH}/bt/uni_bt_conn.c
		${BLUEPAD32_PATH}/bt/uni_bt_hci_cmd.c
		${BLUEPAD32_PATH}/bt/uni_bt_le.c
		${BLUEPAD32_PATH}/bt/uni_bt_sdp.c
		${BLUEPAD32_PATH}/bt/uni_bt_service.c
		${BLUEPAD32_PATH}/bt/uni_bt_setup.c
		${BLUEPAD32_PATH}/controller/uni_balance_board.c
		${BLUEPAD32_PATH}/controller/uni_controller.c
		${BLUEPAD32_PATH}/controller/uni_controller_type.c
		${BLUEPAD32_PATH}/controller/uni_gamepad.c
		${BLUEPAD32_PATH}/controller/uni_keyboard.c
		${BLUEPAD32_PATH}/controller/uni_mouse.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_8bitdo.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_android.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_atari.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_ds3.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_ds4.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_ds5.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_generic.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_icade.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_keyboard.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_mouse.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_nimbus.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_ouya.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_psmove.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_smarttvremote.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_stadia.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_steam.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_switch.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_wii.c
		${BLUEPAD32_PATH}/parser/uni_hid_parser_xboxone.c
		${BLUEPAD32_PATH}/platform/uni_platform.c
		${BLUEPAD32_PATH}/uni_circular_buffer.c
		${BLUEPAD32_PATH}/uni_hid_device.c
		${BLUEPAD32_PATH}/uni_init.c
		${BLUEPAD32_PATH}/uni_joystick.c
		${BLUEPAD32_PATH}/uni_log.c
		${BLUEPAD32_PATH}/uni_property.c
		${BLUEPAD32_PATH}/uni_utils.c
		${BLUEPAD32_PATH}/uni_version.c
		${BLUEPAD32_PATH}/uni_virtual_device.c
		)
target_include_directories(bluepad32 PUBLIC
		${BLUEPAD32_PATH}/include)
target_compile_definitions(bluepad32 PRIVATE
		CYW43_LWIP=0)
target_link_libraries(bluepad32
		pico_stdlib
		pico_cyw43_arch_sys_freertos
		pico_btstack_ble
		pico_btstack_classic
		pico_btstack_cyw43
		FreeRTOS-Kernel-Heap4
		)
