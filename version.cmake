function(get_version_from_git)
    execute_process(
        COMMAND git describe --tags --match "v[0-9]*.[0-9]*.[0-9]*" --always
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_DESCRIBE_VERSION
        RESULT_VARIABLE GIT_DESCRIBE_RESULT
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    if(NOT GIT_DESCRIBE_RESULT EQUAL 0 OR NOT GIT_DESCRIBE_VERSION MATCHES "^v[0-9]+\\.[0-9]+\\.[0-9]+")
        execute_process(
            COMMAND git rev-list --count HEAD
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            OUTPUT_VARIABLE GIT_COMMIT_COUNT
            RESULT_VARIABLE GIT_COMMIT_COUNT_RESULT
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        execute_process(
            COMMAND git rev-parse --short HEAD
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            OUTPUT_VARIABLE GIT_COMMIT_HASH
            RESULT_VARIABLE GIT_COMMIT_HASH_RESULT
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        if(GIT_COMMIT_COUNT_RESULT EQUAL 0 AND GIT_COMMIT_HASH_RESULT EQUAL 0)
            set(GIT_DESCRIBE_VERSION "v0.0.0-${GIT_COMMIT_COUNT}-${GIT_COMMIT_HASH}" PARENT_SCOPE)
        else()
            set(GIT_DESCRIBE_VERSION "v0.0.0-unknown" PARENT_SCOPE)
        endif()
    else()
        set(GIT_DESCRIBE_VERSION "${GIT_DESCRIBE_VERSION}" PARENT_SCOPE)
    endif()

    string(REGEX MATCH "v([0-9]+)[.]([0-9]+)[.]([0-9]+)" VERSION_MATCH "${GIT_DESCRIBE_VERSION}")

    if(VERSION_MATCH)
        set(VERSION_MAJOR ${CMAKE_MATCH_1} PARENT_SCOPE)
        set(VERSION_MINOR ${CMAKE_MATCH_2} PARENT_SCOPE)
        set(VERSION_PATCH ${CMAKE_MATCH_3} PARENT_SCOPE)
    else()
        set(VERSION_MAJOR 0 PARENT_SCOPE)
        set(VERSION_MINOR 0 PARENT_SCOPE)
        set(VERSION_PATCH 0 PARENT_SCOPE)
    endif()
endfunction()