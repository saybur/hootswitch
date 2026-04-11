cmake_minimum_required(VERSION 3.13)

if (DEFINED HOOTSWITCH_WIRELESS)
	target_sources(hootswitch PRIVATE
			bt.c
			)
endif()
