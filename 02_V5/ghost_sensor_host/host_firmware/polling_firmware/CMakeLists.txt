# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(polling_firmware C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(polling_firmware
    polling_firmware.cpp
    src/isl29124.c
    src/open_source_drivers/icm20602/src/icm20602.c
    src/icm20602_pico.c
    src/VL53L4CD/platform.c
    src/VL53L4CD/VL53L4CD_api.c
    src/VL53L4CD/VL53L4CD_calibration.c
    src/open_source_drivers/Library-Sensors/src/bus_i2c.cpp
    src/open_source_drivers/Library-Sensors/src/imu_base.cpp
    src/open_source_drivers/Library-Sensors/src/imu_icm20602.cpp
    src/open_source_drivers/Library-Sensors/src/imu_null.cpp
    src/open_source_drivers/arduino-tca9536/TCA9536.cpp
)

pico_set_program_name(polling_firmware "polling_firmware")
pico_set_program_version(polling_firmware "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(polling_firmware 1)
pico_enable_stdio_usb(polling_firmware 0)

# Add the standard library to the build
target_link_libraries(polling_firmware
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(polling_firmware PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/src/VL53L4CD
        ${CMAKE_CURRENT_LIST_DIR}/inc/VL53L4CD
        ${CMAKE_CURRENT_LIST_DIR}/src/open_source_drivers/Library-Sensors/src
        ${CMAKE_CURRENT_LIST_DIR}/src/open_source_drivers/Library-Sensors/Library-VectorQuaternionsMatrix/src
        ${CMAKE_CURRENT_LIST_DIR}/src/open_source_drivers/arduino-tca9536
)


# Add any user requested libraries
target_link_libraries(polling_firmware 
        pico_stdlib
        hardware_i2c
        )

# Enable USB serial output, disable UART serial
pico_enable_stdio_usb(polling_firmware 1)
pico_enable_stdio_uart(polling_firmware 0)
pico_add_extra_outputs(polling_firmware)
 

