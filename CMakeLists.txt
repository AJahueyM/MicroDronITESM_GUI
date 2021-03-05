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
        "libs/implot/*.cpp"
        "libs/implot/*.h"
        )

include(FetchContent)
FetchContent_Declare(mahi-gui GIT_REPOSITORY https://github.com/mahilab/mahi-gui.git)
FetchContent_MakeAvailable(mahi-gui)


find_package(fmt)


include_directories(src)
include_directories(libs/MAVLinkV2/common)
include_directories(libs/UDPLib/include)
include_directories(${LUA_INCLUDE_DIR})
include_directories(libs/implot)

add_executable(MicroDronITESM_GUI  ${MicroDronGUI_SRC})
#target_link_libraries(MicroDronITESM_GUI )
target_link_libraries(MicroDronITESM_GUI PRIVATE mahi::gui pthread atomic ${LUA_LIBRARIES} fmt::fmt)