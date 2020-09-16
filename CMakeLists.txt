cmake_minimum_required(VERSION 3.13)
project(MicroDronITESM_GUI)

set(CMAKE_CXX_STANDARD 17)
find_package(Lua REQUIRED)

file(GLOB_RECURSE MicroDronGUI_SRC
        "src/*.h"
        "src/*.cpp"
        "src/**/*.cpp"
        "src/**/*.hpp"
        "src/**/*.h"
        "src/*.hpp"
        "src/*.c"
        "src/**/*.c"
        "libs/*.c"
        "libs/*.h"
        )

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(SFML 2.4 REQUIRED network audio graphics window system)

find_package(fmt)


link_libraries(${SFML_LIBRARIES} GL pthread atomic ${LUA_LIBRARIES} fmt::fmt)
include_directories(src)
include_directories(libs/MAVLinkV2/common)
include_directories(libs/UDPLib/include)
include_directories(${LUA_INCLUDE_DIR})

add_executable(MicroDronITESM_GUI  ${MicroDronGUI_SRC} src/MicroDron/Swarm/SimulatedSwarm.cpp src/MicroDron/Swarm/SimulatedSwarm.h src/MicroDron/Swarm/DrawableSwarm.cpp src/MicroDron/Swarm/DrawableSwarm.h)