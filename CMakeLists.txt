cmake_minimum_required(VERSION 3.28)
project(Picophysics)

enable_language(C)
enable_language(CXX)

set(CMAKE_C_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
    ${CMAKE_SOURCE_DIR}
)

add_subdirectory(tests)
