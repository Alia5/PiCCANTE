# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

include(version.cmake)
get_version_from_git()
message(STATUS "Building version: ${GIT_DESCRIBE_VERSION}")

set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# ====================================================================================
set(PICO_BOARD $ENV{PICO_BOARD})

if(NOT DEFINED PICO_BOARD)
    message(STATUS "PICO_BOARD not defined, defaulting to pico2_w")
    set(PICO_BOARD pico2_w CACHE STRING "Board type")
endif()

set(PICO_PLATFORM $ENV{PICO_PLATFORM})

if(NOT DEFINED PICO_PLATFORM)
    message(STATUS "PICO_PLATFORM not defined, defaulting to rp2350")
    set(PICO_PLATFORM rp2350-arm-s)
endif()

set(TINYUSB_OPT_OS OPT_OS_FREERTOS)
set(LOG 0) # TINYUSB Logging
set(CFG_TUSB_OS_INC_PATH ${CMAKE_CURRENT_LIST_DIR}/ext/FreeRTOS-Kernel/include/)

set(PICO_TINYUSB_PATH ${CMAKE_CURRENT_LIST_DIR}/ext/tinyusb)
set(TINYUSB_CONFIG_PATH ${CMAKE_CURRENT_LIST_DIR}/src/config/tusb_config.h)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

set(PICO_EXTRAS_PATH ${CMAKE_CURRENT_LIST_DIR}/ext/pico-extras)
include(${PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

project(PiCCANTE C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 0)
set(PICO_CXX_ENABLE_RTTI 1)

# set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
set(target_name ${PROJECT_NAME})
add_executable(${target_name})

add_compile_definitions(
    PICCANTE_VERSION="${GIT_DESCRIBE_VERSION}"
    PICCANTE_VERSION_MAJOR=${VERSION_MAJOR}
    PICCANTE_VERSION_MINOR=${VERSION_MINOR}
    PICCANTE_VERSION_PATCH=${VERSION_PATCH}
)

set(NUM_CAN_BUSSES $ENV{PICCANTE_NUM_CAN_BUSSES})
set(CAN0_RX_PIN $ENV{PICCANTE_CAN0_RX_PIN})
set(CAN0_TX_PIN $ENV{PICCANTE_CAN0_TX_PIN})
set(CAN1_RX_PIN $ENV{PICCANTE_CAN1_RX_PIN})
set(CAN1_TX_PIN $ENV{PICCANTE_CAN1_TX_PIN})
set(CAN2_RX_PIN $ENV{PICCANTE_CAN2_RX_PIN})
set(CAN2_TX_PIN $ENV{PICCANTE_CAN2_TX_PIN})
set(IDLE_SLEEP_MS $ENV{PICCANTE_IDLE_SLEEP_MS})
set(CAN_QUEUE_TIMEOUT_MS $ENV{PICCANTE_CAN_QUEUE_TIMEOUT_MS})
set(CAN_QUEUE_SIZE $ENV{PICCANTE_CAN_QUEUE_SIZE})

if(NOT DEFINED NUM_CAN_BUSSES)
    if(PICO_PLATFORM STREQUAL "rp2350-arm-s")
        message(STATUS "PICO_PLATFORM is rp2350-arm-s, defaulting to 3 CAN buses")
        set(NUM_CAN_BUSSES 3)
    else()
        message(STATUS "PICO_PLATFORM is probably rp2040, defaulting to 2 CAN buses")
        set(NUM_CAN_BUSSES 2)
    endif()
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

if(NOT DEFINED IDLE_SLEEP_MS)
    message(STATUS "IDLE_SLEEP_MS not defined, defaulting to 10")
    set(IDLE_SLEEP_MS 10)
endif()

if(NOT DEFINED CAN_QUEUE_TIMEOUT_MS)
    message(STATUS "CAN_QUEUE_TIMEOUT_MS not defined, defaulting to 10")
    set(CAN_QUEUE_TIMEOUT_MS 10)
endif()

if(NOT DEFINED CAN_QUEUE_SIZE)
    message(STATUS "CAN_QUEUE_SIZE not defined, defaulting to 32")
    set(CAN_QUEUE_SIZE 32)
endif()

if(NOT DEFINED UART_SPEED)
    message(STATUS "UART_SPEED not defined, defaulting to 115200")
    set(UART_SPEED 115200)
endif()

add_compile_definitions(
    piccanteNUM_CAN_BUSSES=${NUM_CAN_BUSSES}
    piccanteCAN0_RX_PIN=${CAN0_RX_PIN}
    piccanteCAN0_TX_PIN=${CAN0_TX_PIN}
    piccanteCAN1_RX_PIN=${CAN1_RX_PIN}
    piccanteCAN1_TX_PIN=${CAN1_TX_PIN}
    piccanteCAN2_RX_PIN=${CAN2_RX_PIN}
    piccanteCAN2_TX_PIN=${CAN2_TX_PIN}
    piccanteIDLE_SLEEP_MS=${IDLE_SLEEP_MS}
    piccanteCAN_QUEUE_TIMEOUT_MS=${CAN_QUEUE_TIMEOUT_MS}
    piccanteCAN_QUEUE_SIZE=${CAN_QUEUE_SIZE}

    #
    piccanteCAN_NUM_1=1
    piccanteCAN_NUM_2=2
    piccanteCAN_NUM_3=3

    #
    piccanteUART_SPEED=${UART_SPEED}

    #
    CFG_TUSB_OS_INC_PATH=${CFG_TUSB_OS_INC_PATH}
)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

pico_set_program_version(${target_name} "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

file(GLOB_RECURSE srcFiles CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/src/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
)

set(IS_WIFI_BOARD 0)

if(PICO_BOARD MATCHES ".*w.*")
    message(STATUS "WiFi-capable board detected: ${PICO_BOARD}")
    set(IS_WIFI_BOARD 1)
    add_compile_definitions(WIFI_ENABLED=1)
endif()

if(NOT IS_WIFI_BOARD)
    foreach(srcFile ${srcFiles})
        if(NOT "${srcFile}" MATCHES ".*/wifi/.*")
            list(APPEND filteredSrcFiles ${srcFile})
        else()
            message(STATUS "Excluding WiFi file from build: ${srcFile}")
        endif()
    endforeach()

    set(srcFiles ${filteredSrcFiles})
endif()

target_sources(${target_name} PRIVATE
    src/main.cpp
    ${srcFiles}
)

pico_set_program_name(${target_name} "PiCCANTE")
pico_set_program_version(${target_name} "0.1")

if(DEFINED ENV{PICO_DEBUG} OR CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(PICO_DEOPTIMIZED_DEBUG 1)
    target_compile_options(${target_name} PRIVATE -Wall -Wextra -Og -DDEBUG)
ELSE()
    target_compile_options(${target_name} PRIVATE -Wall -Wextra -O3)
ENDIF()

target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)

if(DEFINED LOG AND LOG GREATER 0)
    pico_enable_stdio_uart(${target_name} 1)
else()
    pico_enable_stdio_uart(${target_name} 0)
endif()

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
    "${CMAKE_CURRENT_LIST_DIR}/ext/**/*.c"
    "${PICO_SDK_PATH}/**/*.c"
    "${PICO_SDK_PATH}/**/*.h"
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
    ${CMAKE_CURRENT_LIST_DIR}/src/config
)

target_include_directories(${target_name} SYSTEM PRIVATE
    ${FREERTOS_KERNEL_PATH}/include

    # Can2040
    ${CAN2040_PATH}/src

    # littlefs
    ${CMAKE_CURRENT_LIST_DIR}/ext/littlefs
)

# Set up paths
set(FS_FILES_DIR ${CMAKE_CURRENT_LIST_DIR}/fs_files)
set(FS_FILES_LOCAL_DIR ${CMAKE_CURRENT_LIST_DIR}/fs_files.local)
set(BUILD_FS_FILES_DIR ${CMAKE_BINARY_DIR}/fs_files)
set(BUILD_FS_FILES_LOCAL_DIR${CMAKE_BINARY_DIR}/fs_files.local)
set(WEB_DIR ${CMAKE_CURRENT_LIST_DIR}/Web)
set(BUILD_WEB_DIR ${BUILD_FS_FILES_DIR}/web)

if(EXISTS ${FS_FILES_LOCAL_DIR})
    add_custom_target(copy_fs_files ALL
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${FS_FILES_LOCAL_DIR} ${BUILD_FS_FILES_DIR}
        COMMENT "Copying fs_files.local to build directory"
    )
else()
    add_custom_target(copy_fs_files ALL
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${FS_FILES_DIR} ${BUILD_FS_FILES_DIR}
        COMMENT "Copying fs_files to build directory"
    )
endif()

set(LFS_EXTRA_DEPS copy_fs_files)
find_program(NODE_EXECUTABLE node)

if(IS_WIFI_BOARD AND NODE_EXECUTABLE)
    file(GLOB_RECURSE WEB_SOURCE_FILES
        "${WEB_DIR}/src/**/*.ts"
        "${WEB_DIR}/src/**/*.tsx"
        "${WEB_DIR}/src/**/*.js"
        "${WEB_DIR}/src/**/*.jsx"
        "${WEB_DIR}/src/**/*.css"
        "${WEB_DIR}/src/**/*.html"
        "${WEB_DIR}/package.json"
        "${WEB_DIR}/vite.config.ts"
    )

    find_program(PNPM_EXECUTABLE pnpm)

    if(NOT PNPM_EXECUTABLE)
        message(STATUS "pnpm not found, attempting to install via corepack...")
        execute_process(COMMAND corepack enable)
        execute_process(COMMAND corepack prepare pnpm@latest --activate)

        # Try again
        find_program(PNPM_EXECUTABLE pnpm)

        if(NOT PNPM_EXECUTABLE)
            message(FATAL_ERROR "pnpm could not be installed via corepack! Please check your Node.js installation.")
        endif()
    endif()

    add_custom_command(
        OUTPUT ${CMAKE_BINARY_DIR}/web_assets_ready
        COMMAND ${CMAKE_COMMAND} -E make_directory ${BUILD_WEB_DIR}
        COMMAND pnpm install
        COMMAND pnpm run build
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_BINARY_DIR}/web_assets_ready
        WORKING_DIRECTORY ${WEB_DIR}
        DEPENDS ${WEB_SOURCE_FILES}
        COMMENT "Building web assets"
    )

    add_custom_target(web_assets
        DEPENDS ${CMAKE_BINARY_DIR}/web_assets_ready
    )
    add_dependencies(${target_name} web_assets)

    set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
        ${BUILD_FS_FILES_DIR}
        ${BUILD_FS_FILES_LOCAL_DIR}
    )
    list(APPEND LFS_EXTRA_DEPS web_assets)

else()
    message(WARNING "Node.js (node) not found! Web assets will NOT be built or included. Please install Node.js if you want web UI.")
endif()

# Set filesystem size with default of 512KB
if(NOT DEFINED ENV{PICCANTE_FS_SIZE_KB})
    set(LITTLEFS_SIZE_KB 512)
else()
    set(LITTLEFS_SIZE_KB $ENV{PICCANTE_FS_SIZE_KB})
endif()

message(STATUS "Using filesystem size of ${LITTLEFS_SIZE_KB}KB")

math(EXPR LITTLEFS_SIZE_BYTES "${LITTLEFS_SIZE_KB} * 1024")
math(EXPR LFS_BLOCK_COUNT "${LITTLEFS_SIZE_KB} / 4")

configure_file(${CMAKE_CURRENT_LIST_DIR}/flash_region.template.ld ${CMAKE_BINARY_DIR}/pico_flash_region.ld)

set(LITTLEFS_DATA_DIR ${BUILD_FS_FILES_DIR})
file(MAKE_DIRECTORY ${LITTLEFS_DATA_DIR})
include(${CMAKE_CURRENT_LIST_DIR}/generate_littlefs.cmake)
generate_littlefs_image(${target_name} ${LITTLEFS_DATA_DIR} ${LITTLEFS_SIZE_BYTES}
    DEPENDS ${LFS_EXTRA_DEPS}
)
target_compile_definitions(${target_name} PRIVATE
    LFS_BLOCK_SIZE=4096
    LFS_BLOCK_COUNT=${LFS_BLOCK_COUNT}
    LFS_CACHE_SIZE=128
    LFS_LOOKAHEAD_SIZE=32
)

target_sources(${target_name} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/ext/littlefs/lfs.c
    ${CMAKE_CURRENT_LIST_DIR}/ext/littlefs/lfs_util.c
)

# \ littlefs

# Add any user requested libraries
target_link_libraries(${target_name}
    pico_async_context_freertos
    FreeRTOS-Kernel-Heap4
    pico_unique_id
    tinyusb_device
    tinyusb_board
    hardware_pio
    hardware_adc
    hardware_sleep

    -lstdc++
)

if(IS_WIFI_BOARD)
    target_link_libraries(${target_name}
        pico_cyw43_arch_lwip_sys_freertos
        pico_lwip_http
        pico_lwip_mdns
        pico_btstack_classic
        pico_btstack_ble
    )

    # Make sure we include lwIP headers
    target_include_directories(${target_name} PRIVATE
        ${PICO_SDK_PATH}/lib/lwip/src/include
        ${PICO_SDK_PATH}/lib/btstack/src
    )

    add_compile_definitions(CYW43_HOST_NAME="PiCCANTE")
endif()

# pico_enable_stdio_usb(${target_name} 0)
pico_add_extra_outputs(${target_name})
target_link_options(${target_name} PRIVATE "LINKER:--script=${CMAKE_CURRENT_LIST_DIR}/fs_end_flash.ld")
