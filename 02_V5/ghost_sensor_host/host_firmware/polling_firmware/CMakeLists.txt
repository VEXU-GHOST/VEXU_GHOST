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

# FreeRTOS-Kernel (RP2040 single-core port)
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/FreeRTOS-Kernel)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Add executable. Default name is the project name, version 0.1

add_executable(polling_firmware
    polling_firmware.cpp
    src/comms.cpp
    src/Device.cpp
    src/isl29124.cpp
    src/VL53L4CD/platform.cpp
    src/VL53L4CD/VL53L4CD_api.cpp
    src/VL53L4CD/VL53L4CD_calibration.cpp
    src/ICM20602/bus_i2c.cpp
    src/ICM20602/imu_base.cpp
    src/ICM20602/imu_icm20602.cpp
    src/ICM20602/imu_null.cpp
    src/TCA9536.cpp
    src/I2CBus/HardwareI2CBus.cpp
    src/I2CBus/SoftwareI2CBus.cpp
)

# PIO I2C program — uncomment once src/i2c.pio is present
pico_generate_pio_header(polling_firmware ${CMAKE_CURRENT_LIST_DIR}/src/I2CBus/i2c.pio)

pico_set_program_name(polling_firmware "polling_firmware")
pico_set_program_version(polling_firmware "0.1")

# Add the standard library to the build
target_link_libraries(polling_firmware
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(polling_firmware PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/inc
        ${CMAKE_CURRENT_LIST_DIR}/src/VL53L4CD
        ${CMAKE_CURRENT_LIST_DIR}/inc/VL53L4CD
        ${CMAKE_CURRENT_LIST_DIR}/src/ICM20602
        ${CMAKE_CURRENT_LIST_DIR}/inc/ICM20602
        ${CMAKE_CURRENT_LIST_DIR}/src/I2CBus
        ${CMAKE_CURRENT_LIST_DIR}/inc/I2CBus
        ${CMAKE_CURRENT_LIST_DIR}/lib/Library-VectorQuaternionsMatrix/src
)


# Add any user requested libraries
target_link_libraries(polling_firmware
        pico_stdlib
        hardware_i2c
        hardware_pio
        hardware_clocks
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
        )

# Enable USB serial output, disable UART serial
pico_enable_stdio_usb(polling_firmware 1)
pico_enable_stdio_uart(polling_firmware 0)
pico_add_extra_outputs(polling_firmware)
 

