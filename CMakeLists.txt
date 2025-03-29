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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PicoRet C CXX ASM)

# Enabling Exceptions kills debugging as the SDK is buggy...
set(PICO_CXX_ENABLE_EXCEPTIONS 0)
set(PICO_CXX_ENABLE_RTTI 1)

set(target_name ${PROJECT_NAME})
add_executable(${target_name} )

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


pico_set_program_name(${target_name} "PicoRet")
pico_set_program_version(${target_name} "0.1")

target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)

# Generate PIO header
pico_generate_pio_header(${target_name} ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${target_name} 1)
pico_enable_stdio_usb(${target_name} 1)

# Add the standard library to the build
target_link_libraries(${target_name}
        pico_stdlib)

# Pull in FreeRTOS
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/ext/FreeRTOS-Kernel)
set(FREERTOS_CONFIG_PATH ${CMAKE_CURRENT_LIST_DIR}/src/config)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Add the standard include files to the build
target_include_directories(${target_name} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${FREERTOS_KERNEL_PATH}/include
        ${CMAKE_CURRENT_LIST_DIR}/src/config
)

# Add any user requested libraries
target_link_libraries(${target_name} 
        pico_async_context_freertos FreeRTOS-Kernel-Heap4
        hardware_pio
        pico_cyw43_arch_none
        -lstdc++
    )

pico_add_extra_outputs(${target_name})

