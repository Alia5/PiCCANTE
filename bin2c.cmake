file(READ ${BIN_FILE} CONTENTS HEX)

string(LENGTH "${CONTENTS}" CONTENTS_LENGTH)
math(EXPR MAX_POS "${CONTENTS_LENGTH} - 2")
set(LAST_NON_FF_POS -1)

foreach(POS RANGE 0 ${MAX_POS} 2)
    string(SUBSTRING "${CONTENTS}" ${POS} 2 BYTE)

    if(NOT "${BYTE}" STREQUAL "ff")
        set(LAST_NON_FF_POS ${POS})
    endif()
endforeach()

if(${LAST_NON_FF_POS} GREATER -1)
    math(EXPR TRUNC_POS "${LAST_NON_FF_POS} + 2")
    string(SUBSTRING "${CONTENTS}" 0 ${TRUNC_POS} TRUNCATED)

    math(EXPR TRUNCATED_SIZE "${TRUNC_POS} / 2")
else()
    set(TRUNCATED "")
    set(TRUNCATED_SIZE 0)
endif()

string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1, " C_CONTENT ${TRUNCATED})
file(WRITE ${C_FILE} "/* Auto-generated filesystem data */\n")
file(APPEND ${C_FILE} "const unsigned char __attribute__((section(\".littlefs_fs\"))) fs_data[] = {\n  ${C_CONTENT}\n};\n\n")
file(APPEND ${C_FILE} "const unsigned int fs_data_size = ${BIN_SIZE};\n")
file(APPEND ${C_FILE} "const unsigned int fs_data_actual_size = ${TRUNCATED_SIZE};\n")