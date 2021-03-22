cmake_minimum_required(VERSION 3.13)
project(MicroDronITESM_GUI)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -pedantic)

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
        "libs/implot/*.cpp"
        "libs/implot/*.h"
        )

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(SFML_DIR "${CMAKE_CURRENT_LIST_DIR}/cmake")
message(${CMAKE_MODULE_PATH})

find_package(SFML 2 REQUIRED network audio graphics window system)
set(IMGUI_SFML_FIND_SFML OFF)
set(IMGUI_SFML_BUILD_EXAMPLES OFF)
set(IMGUI_SFML_IMGUI_DEMO ON)
set(IMGUI_DIR "${CMAKE_CURRENT_LIST_DIR}/libs/imgui" [PARENT_SCOPE])

add_subdirectory(libs/imgui-sfml EXCLUDE_FROM_ALL)

find_package(fmt)


include_directories(src)
include_directories(libs/MAVLinkV2/common)
include_directories(libs/UDPLib/include)
include_directories(${LUA_INCLUDE_DIR})
include_directories(libs/implot)

add_executable(MicroDronITESM_GUI  ${MicroDronGUI_SRC})
#target_link_libraries(MicroDronITESM_GUI )
target_link_libraries(MicroDronITESM_GUI PRIVATE ImGui-SFML::ImGui-SFML ${SFML_LIBRARIES} GL pthread atomic ${LUA_LIBRARIES} fmt::fmt)