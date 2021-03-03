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
        "libs/MAVLinkV2/*.c"
        "libs/MAVLinkV2/*.h"
        "libs/UDPLib/*.h"
        "libs/UDPLib/*.c"
        )

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(SFML 2.5 REQUIRED network audio graphics window system)
set(IMGUI_SFML_FIND_SFML OFF)
set(IMGUI_SFML_BUILD_EXAMPLES OFF)
set(IMGUI_SFML_IMGUI_DEMO ON)
#set(ImGui-SFML_DIR "${CMAKE_CURRENT_LIST_DIR}/libs/imgui-sfml/cmake/")
#message(${ImGui-SFML_DIR})
set(IMGUI_DIR "${CMAKE_CURRENT_LIST_DIR}/libs/imgui" [PARENT_SCOPE])

add_subdirectory(libs/imgui-sfml EXCLUDE_FROM_ALL)

find_package(fmt)


include_directories(src)
include_directories(libs/MAVLinkV2/common)
include_directories(libs/UDPLib/include)
include_directories(${LUA_INCLUDE_DIR})

add_executable(MicroDronITESM_GUI  ${MicroDronGUI_SRC})
#target_link_libraries(MicroDronITESM_GUI )
target_link_libraries(MicroDronITESM_GUI PRIVATE ImGui-SFML::ImGui-SFML ${SFML_LIBRARIES} GL pthread atomic ${LUA_LIBRARIES} fmt::fmt)