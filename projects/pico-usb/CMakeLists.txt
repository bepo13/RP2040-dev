cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico-usb)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# include headers
include_directories(
    .
    "${PICO_SDK_PATH}/lib/tinyusb/hw"
    "${PICO_SDK_PATH}/lib/tinyusb/src"
)

# predefined symbles
add_compile_definitions(
	__PICO_SDK__
)

# rest of your project
add_executable(pico-usb
    main.c
    usb_descriptors.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(pico-usb pico_stdlib tinyusb_device tinyusb_board)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico-usb)