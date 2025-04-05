function(generate_littlefs_image TARGET FS_DIR FS_SIZE)
    set(FS_IMAGE ${CMAKE_BINARY_DIR}/littlefs.bin)
    set(FS_DATA_C ${CMAKE_BINARY_DIR}/fs_data.c)

    add_custom_command(
        OUTPUT ${FS_IMAGE}
        COMMAND mklittlefs -c ${FS_DIR} -s ${FS_SIZE} ${FS_IMAGE}
        DEPENDS ${FS_DIR}
        COMMENT "Generating LittleFS image from ${FS_DIR}"
        VERBATIM
    )

    add_custom_command(
        OUTPUT ${FS_DATA_C}
        COMMAND xxd -i -c 16 ${FS_IMAGE} |
        sed "1s/.*/const unsigned char __attribute__((section(\".littlefs_fs\"))) fs_data[] = {/; \
                     s/unsigned int/const unsigned int/; \
                     s/\\(.*\\)_len/\\1_size/" > ${FS_DATA_C}
        DEPENDS ${FS_IMAGE}
        COMMENT "Converting LittleFS image to C array"
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