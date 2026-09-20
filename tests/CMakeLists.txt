file(GLOB PP_TESTS ${CMAKE_CURRENT_SOURCE_DIR}/test_*.h)

set_source_files_properties(${PP_TESTS} LANGUAGE CXX)

set(TEST_GENERATOR_BIN ${PROJECT_SOURCE_DIR}/tools/test_generator.py)
set(TEST_MAIN_FILENAME ${CMAKE_CURRENT_BINARY_DIR}/main.cpp)

add_custom_command(
    OUTPUT ${TEST_MAIN_FILENAME}
    COMMAND ${TEST_GENERATOR_BIN} --output ${TEST_MAIN_FILENAME} ${PP_TESTS}
    DEPENDS ${PP_TESTS} ${TEST_GENERATOR_BIN}
)

set(PP_TEST_SOURCES ${TEST_MAIN_FILENAME} ${PP_TESTS} ${CMAKE_CURRENT_SOURCE_DIR}/picophysics.c)

# `tests` is the suite on plain C maths; `tests_sh4zam` is the same suite with
# the maths going through SH4ZAM. With kos-cmake both come out as Dreamcast
# ELFs, plus run-tests / run-tests-sh4zam targets that hand them to $KOS_LOADER.
add_executable(tests ${PP_TEST_SOURCES})
target_include_directories(tests PRIVATE ${PROJECT_SOURCE_DIR})

if(PICOPHYSICS_HAVE_SH4ZAM)
    add_executable(tests_sh4zam ${PP_TEST_SOURCES})
    target_include_directories(tests_sh4zam PRIVATE ${PROJECT_SOURCE_DIR})
    target_compile_definitions(tests_sh4zam PRIVATE PICOPHYSICS_USE_SH4ZAM)
    target_link_libraries(tests_sh4zam PRIVATE sh4zam)
else()
    message(STATUS "extern/sh4zam not checked out: skipping tests_sh4zam")
endif()

if(PLATFORM_DREAMCAST)
    include(kallistios REQUIRED)
    kos_run_target(tests run-tests)
    if(TARGET tests_sh4zam)
        kos_run_target(tests_sh4zam run-tests-sh4zam)
    endif()
endif()
