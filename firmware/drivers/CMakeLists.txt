cmake_minimum_required(VERSION 3.13)

target_sources(hootswitch PRIVATE
		joystick.c joystick_handler.c
		kensington.c
		keyboard.c keyboard_handler.c
		mouse.c mouse_handler.c
		serial.c
		virtual.c
		)
