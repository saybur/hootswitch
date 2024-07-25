cmake_minimum_required(VERSION 3.12)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(hootswitch)

pico_sdk_init()

add_executable(hootswitch
        relay.c main.c
        )

# pull in common dependencies
target_link_libraries(hootswitch pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(hootswitch)
