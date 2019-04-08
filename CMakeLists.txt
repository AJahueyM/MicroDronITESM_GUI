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
find_package(SFML REQUIRED network audio graphics window system)


link_libraries(sfml-graphics sfml-window sfml-system pthread GL)

add_executable(MicroDronITESM_GUI ${MicroDronGUI_SRC} src/MicroDron/MicroDronInterface.cpp src/MicroDron/MicroDronInterface.h)