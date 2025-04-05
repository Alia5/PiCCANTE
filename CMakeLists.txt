# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(NUM_CAN_BUSSES $ENV{PICCANTE_NUM_CAN_BUSSES})
set(CAN0_RX_PIN $ENV{PICCANTE_CAN0_RX_PIN})
set(CAN0_TX_PIN $ENV{PICCANTE_CAN0_TX_PIN})
set(CAN1_RX_PIN $ENV{PICCANTE_CAN1_RX_PIN})
set(CAN1_TX_PIN $ENV{PICCANTE_CAN1_TX_PIN})
set(CAN2_RX_PIN $ENV{PICCANTE_CAN2_RX_PIN})
set(CAN2_TX_PIN $ENV{PICCANTE_CAN2_TX_PIN})
set(CAN_IDLE_SLEEP_MS $ENV{PICCANTE_CAN_IDLE_SLEEP_MS})
set(CAN_QUEUE_TIMEOUT_MS $ENV{PICCANTE_CAN_QUEUE_TIMEOUT_MS})
set(CAN_QUEUE_SIZE $ENV{PICCANTE_CAN_QUEUE_SIZE})

if(NOT DEFINED NUM_CAN_BUSSES)
    message(STATUS "NUM_CAN_BUSSES not defined, defaulting to 2")
    set(NUM_CAN_BUSSES 2)
endif()

if(NOT DEFINED CAN0_RX_PIN)
    message(STATUS "CAN0_RX_PIN not defined, defaulting to 4")
    set(CAN0_RX_PIN 4)
endif()

if(NOT DEFINED CAN0_TX_PIN)
    message(STATUS "CAN0_TX_PIN not defined, defaulting to 5")
    set(CAN0_TX_PIN 5)
endif()

if(NOT DEFINED CAN1_RX_PIN)
    message(STATUS "CAN1_RX_PIN not defined, defaulting to 6")
    set(CAN1_RX_PIN 6)
endif()

if(NOT DEFINED CAN1_TX_PIN)
    message(STATUS "CAN1_TX_PIN not defined, defaulting to 7")
    set(CAN1_TX_PIN 7)
endif()

if(NOT DEFINED CAN2_RX_PIN)
    message(STATUS "CAN2_RX_PIN not defined, defaulting to 8")
    set(CAN2_RX_PIN 8)
endif()

if(NOT DEFINED CAN2_TX_PIN)
    message(STATUS "CAN2_TX_PIN not defined, defaulting to 9")
    set(CAN2_TX_PIN 9)
endif()

if(NOT DEFINED CAN_IDLE_SLEEP_MS)
    message(STATUS "CAN_IDLE_SLEEP_MS not defined, defaulting to 10")
    set(CAN_IDLE_SLEEP_MS 10)
endif()

if(NOT DEFINED CAN_QUEUE_TIMEOUT_MS)
    message(STATUS "CAN_QUEUE_TIMEOUT_MS not defined, defaulting to 10")
    set(CAN_QUEUE_TIMEOUT_MS 10)
endif()

if(NOT DEFINED CAN_QUEUE_SIZE)
    message(STATUS "CAN_QUEUE_SIZE not defined, defaulting to 32")
    set(CAN_QUEUE_SIZE 32)
endif()

add_compile_definitions(
    piccanteNUM_CAN_BUSSES=${NUM_CAN_BUSSES}
    piccanteCAN0_RX_PIN=${CAN0_RX_PIN}
    piccanteCAN0_TX_PIN=${CAN0_TX_PIN}
    piccanteCAN1_RX_PIN=${CAN1_RX_PIN}
    piccanteCAN1_TX_PIN=${CAN1_TX_PIN}
    piccanteCAN2_RX_PIN=${CAN2_RX_PIN}
    piccanteCAN2_TX_PIN=${CAN2_TX_PIN}
    piccanteCAN_IDLE_SLEEP_MS=${CAN_IDLE_SLEEP_MS}
    piccanteCAN_QUEUE_TIMEOUT_MS=${CAN_QUEUE_TIMEOUT_MS}
    piccanteCAN_QUEUE_SIZE=${CAN_QUEUE_SIZE}

    #
    picacanteCAN_NUM_1=1
    picacanteCAN_NUM_2=2
    picacanteCAN_NUM_3=3
)


set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")
set(PICO_PLATFORM rp2350-arm-s)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PiCCANTE C CXX ASM)

# Enabling Exceptions kills debugging as the SDK is buggy...
set(PICO_CXX_ENABLE_EXCEPTIONS 0)
set(PICO_CXX_ENABLE_RTTI 1)

set(target_name ${PROJECT_NAME})
add_executable(${target_name})

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

file(GLOB_RECURSE srcFiles CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/src/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
)

target_sources(${target_name} PRIVATE
    src/main.cpp
    ${srcFiles}
)

pico_set_program_name(${target_name} "PiCCANTE")
pico_set_program_version(${target_name} "0.1")

target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)

if(DEFINED ENV{PICO_DEBUG} OR CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(PICO_DEOPTIMIZED_DEBUG 1)
    target_compile_options(${target_name} PRIVATE -Wall -Wextra -Og)
ELSE()
    target_compile_options(${target_name} PRIVATE -Wall -Wextra -O3)
ENDIF()

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${target_name} 1)
pico_enable_stdio_usb(${target_name} 0)

# Add the standard library to the build
target_link_libraries(${target_name}
    pico_stdlib)

# Can2040
set(CAN2040_PATH ${CMAKE_CURRENT_LIST_DIR}/ext/can2040)
pico_generate_pio_header(${target_name} ${CAN2040_PATH}/pio/can2040.pio)

# target_compile_options(${target_name} PRIVATE -Wall -Wextra -O3)
target_sources(
    ${target_name} PRIVATE
    ${CAN2040_PATH}/src/can2040.c
)

file(GLOB_RECURSE extFiles CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/ext/**/*"
    "${PICO_SDK_PATH}/**/*"
    "${PICO_SDK_PATH}/**/*"
)

set_source_files_properties(
    ${extFiles}
    PROPERTIES
    COMPILE_FLAGS "-w"
)

# Pull in FreeRTOS
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/ext/FreeRTOS-Kernel)
set(FREERTOS_CONFIG_PATH ${CMAKE_CURRENT_LIST_DIR}/src/config)

if(PICO_PLATFORM STREQUAL "rp2040")
    include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
elseif(PICO_PLATFORM STREQUAL "rp2350-arm-s")
    include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/Community-Supported-Ports/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)
endif()

# Add the standard include files to the build
target_include_directories(${target_name} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${FREERTOS_KERNEL_PATH}/include
    ${CMAKE_CURRENT_LIST_DIR}/src/config

    # Can2040
    ${CAN2040_PATH}/src
)

# Add any user requested libraries
target_link_libraries(${target_name}
    pico_async_context_freertos FreeRTOS-Kernel-Heap4
    hardware_pio
    pico_unique_id tinyusb_device tinyusb_board
    pico_cyw43_arch_none

    -lstdc++
)
# pico_enable_stdio_usb(${target_name} 0)
pico_add_extra_outputs(${target_name})
