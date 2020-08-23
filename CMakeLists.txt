cmake_minimum_required(VERSION 3.13)
project(MicroDronITESM_GUI)

set(CMAKE_CXX_STANDARD 14)

file(GLOB_RECURSE MicroDronGUI_SRC
        "src/*.h"
        "src/*.cpp"
        "src/**/*.cpp"
        "src/**/*.hpp"
        "src/**/*.h"
        "src/*.hpp"
        "src/*.c"
        "src/**/*.c"
        )

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(SFML 2.4 REQUIRED network audio graphics window system)

link_libraries(${SFML_LIBRARIES} GL pthread atomic)
include_directories(src)
include_directories(libs/MAVLinkV2/common)

add_executable(MicroDronITESM_GUI  ${MicroDronGUI_SRC})