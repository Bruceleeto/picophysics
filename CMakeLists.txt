cmake_minimum_required(VERSION 3.22)
project(Picophysics LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# picophysics is a single header: this target only carries the include path
# (and, optionally, SH4ZAM) to whoever links it.
add_library(picophysics INTERFACE)
target_include_directories(picophysics INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# Route the maths through SH4ZAM. Uses the copy in extern/sh4zam:
#   git submodule update --init
option(PICOPHYSICS_USE_SH4ZAM "Route maths through SH4ZAM" OFF)

set(PICOPHYSICS_SH4ZAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/sh4zam)
if(EXISTS ${PICOPHYSICS_SH4ZAM_DIR}/CMakeLists.txt)
    set(PICOPHYSICS_HAVE_SH4ZAM TRUE)
    if(NOT TARGET sh4zam)
        add_subdirectory(${PICOPHYSICS_SH4ZAM_DIR} EXCLUDE_FROM_ALL)
    endif()
endif()

if(PICOPHYSICS_USE_SH4ZAM)
    if(NOT TARGET sh4zam)
        message(FATAL_ERROR "PICOPHYSICS_USE_SH4ZAM is on but extern/sh4zam is empty: git submodule update --init")
    endif()
    target_compile_definitions(picophysics INTERFACE PICOPHYSICS_USE_SH4ZAM)
    target_link_libraries(picophysics INTERFACE sh4zam)
endif()

# On by default when this is the top-level project, off when it has been
# pulled into someone else's build.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(PICOPHYSICS_TESTS_DEFAULT ON)
else()
    set(PICOPHYSICS_TESTS_DEFAULT OFF)
endif()
option(PICOPHYSICS_ENABLE_TESTS "Build the picophysics unit tests" ${PICOPHYSICS_TESTS_DEFAULT})

if(PICOPHYSICS_ENABLE_TESTS)
    add_subdirectory(tests)
endif()
