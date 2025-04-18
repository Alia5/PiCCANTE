function(download_mklittlefs)
    # Determine platform-specific download URL
    if(WIN32)
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(MKLITTLEFS_URL "https://github.com/earlephilhower/mklittlefs/releases/download/4.0.2/x86_64-w64-mingw32-mklittlefs-db0513a.zip" PARENT_SCOPE)
            set(MKLITTLEFS_IS_ZIP TRUE PARENT_SCOPE)
        else()
            set(MKLITTLEFS_URL "https://github.com/earlephilhower/mklittlefs/releases/download/4.0.2/i686-w64-mingw32-mklittlefs-db0513a.zip" PARENT_SCOPE)
            set(MKLITTLEFS_IS_ZIP TRUE PARENT_SCOPE)
        endif()

        set(MKLITTLEFS_EXECUTABLE "mklittlefs.exe" PARENT_SCOPE)
    elseif(APPLE)
        execute_process(
            COMMAND uname -m
            OUTPUT_VARIABLE ARCH
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        if(ARCH STREQUAL "arm64")
            set(MKLITTLEFS_URL "https://github.com/earlephilhower/mklittlefs/releases/download/4.0.2/aarch64-apple-darwin-mklittlefs-db0513a.tar.gz" PARENT_SCOPE)
        else()
            set(MKLITTLEFS_URL "https://github.com/earlephilhower/mklittlefs/releases/download/4.0.2/x86_64-apple-darwin-mklittlefs-db0513a.tar.gz" PARENT_SCOPE)
        endif()

        set(MKLITTLEFS_IS_ZIP FALSE PARENT_SCOPE)
        set(MKLITTLEFS_EXECUTABLE "mklittlefs" PARENT_SCOPE)
    else() # Linux
        execute_process(
            COMMAND uname -m
            OUTPUT_VARIABLE ARCH
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        if(ARCH STREQUAL "aarch64")
            set(MKLITTLEFS_URL "https://github.com/earlephilhower/mklittlefs/releases/download/4.0.2/aarch64-linux-gnu-mklittlefs-db0513a.tar.gz" PARENT_SCOPE)
        elseif(ARCH MATCHES "^arm")
            set(MKLITTLEFS_URL "https://github.com/earlephilhower/mklittlefs/releases/download/4.0.2/arm-linux-gnueabihf-mklittlefs-db0513a.tar.gz" PARENT_SCOPE)
        else()
            set(MKLITTLEFS_URL "https://github.com/earlephilhower/mklittlefs/releases/download/4.0.2/x86_64-linux-gnu-mklittlefs-db0513a.tar.gz" PARENT_SCOPE)
        endif()

        set(MKLITTLEFS_IS_ZIP FALSE PARENT_SCOPE)
        set(MKLITTLEFS_EXECUTABLE "mklittlefs" PARENT_SCOPE)
    endif()
endfunction()

function(generate_littlefs_image TARGET FS_DIR FS_SIZE)
    set(FS_IMAGE ${CMAKE_BINARY_DIR}/littlefs.bin)
    set(FS_DATA_C ${CMAKE_BINARY_DIR}/fs_data.c)
    set(CONVERTER_SCRIPT ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/bin2c.cmake)

    # Prefer FS_DIR.local if it exists
    set(FS_DIR_LOCAL "${FS_DIR}.local")
    if(EXISTS "${FS_DIR_LOCAL}")
        message(STATUS "Using local filesystem folder: ${FS_DIR_LOCAL}")
        set(FS_DIR "${FS_DIR_LOCAL}")
    endif()

    download_mklittlefs()

    # Set up paths for downloaded tool
    set(TOOLS_PATH ${CMAKE_BINARY_DIR}/tools)
    set(MKLITTLEFS_PATH ${TOOLS_PATH})
    set(MKLITTLEFS_ARCHIVE ${TOOLS_PATH}/mklittlefs_archive)
    set(MKLITTLEFS_BINARY ${MKLITTLEFS_PATH}/mklittlefs/${MKLITTLEFS_EXECUTABLE})

    # Make dirs
    file(MAKE_DIRECTORY ${TOOLS_PATH})
    file(MAKE_DIRECTORY ${MKLITTLEFS_PATH})

    if(NOT EXISTS ${MKLITTLEFS_BINARY})
        message(STATUS "Downloading mklittlefs from ${MKLITTLEFS_URL}")

        file(DOWNLOAD ${MKLITTLEFS_URL} ${MKLITTLEFS_ARCHIVE}
            SHOW_PROGRESS
            STATUS DOWNLOAD_STATUS)

        list(GET DOWNLOAD_STATUS 0 STATUS_CODE)

        if(NOT STATUS_CODE EQUAL 0)
            list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
            message(FATAL_ERROR "Error downloading mklittlefs: ${ERROR_MESSAGE}")
        endif()

        if(MKLITTLEFS_IS_ZIP)
            execute_process(
                COMMAND ${CMAKE_COMMAND} -E tar xf ${MKLITTLEFS_ARCHIVE}
                WORKING_DIRECTORY ${MKLITTLEFS_PATH}
            )
        else()
            execute_process(
                COMMAND ${CMAKE_COMMAND} -E tar xzf ${MKLITTLEFS_ARCHIVE}
                WORKING_DIRECTORY ${MKLITTLEFS_PATH}
            )
        endif()

        if(NOT WIN32)
            file(CHMOD ${MKLITTLEFS_BINARY}
                PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE)
        endif()

        # Verify the download was successful
        if(NOT EXISTS ${MKLITTLEFS_BINARY})
            message(FATAL_ERROR "Failed to extract mklittlefs binary at ${MKLITTLEFS_BINARY}")
        else()
            message(STATUS "mklittlefs binary ready at ${MKLITTLEFS_BINARY}")
        endif()
    endif()

    add_custom_command(
        OUTPUT ${FS_IMAGE}
        COMMAND ${MKLITTLEFS_BINARY} -c ${FS_DIR} -s ${FS_SIZE} ${FS_IMAGE}
        DEPENDS ${FS_DIR}
        COMMENT "Generating LittleFS image from ${FS_DIR}"
        VERBATIM
    )

    add_custom_command(
        OUTPUT ${FS_DATA_C}
        COMMAND ${CMAKE_COMMAND} -D BIN_FILE=${FS_IMAGE} -D C_FILE=${FS_DATA_C} -D BIN_SIZE=${FS_SIZE} -P ${CONVERTER_SCRIPT}
        DEPENDS ${FS_IMAGE} ${CONVERTER_SCRIPT}
        COMMENT "Converting LittleFS image to C array (cross-platform)"
        VERBATIM
    )

    # Add __attribute__((used)) to prevent optimization
    add_library(${TARGET}_fs_data OBJECT ${FS_DATA_C})
    target_include_directories(${TARGET}_fs_data PRIVATE ${CMAKE_BINARY_DIR})

    # Make sure compile flags don't strip the data
    target_compile_options(${TARGET}_fs_data PRIVATE -fno-lto -fno-builtin)

    target_link_libraries(${TARGET} ${TARGET}_fs_data)
    add_custom_target(${TARGET}_fs DEPENDS ${FS_IMAGE})
    add_dependencies(${TARGET}_fs_data ${TARGET}_fs)
    add_dependencies(${TARGET} ${TARGET}_fs_data)

    add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "Verifying filesystem section in ELF:"
        COMMAND arm-none-eabi-objdump -h ${CMAKE_BINARY_DIR}/${TARGET}.elf | grep littlefs_fs
        COMMAND ${CMAKE_COMMAND} -E copy ${FS_IMAGE} ${CMAKE_CURRENT_BINARY_DIR}/${TARGET}.littlefs.bin
    )
endfunction()