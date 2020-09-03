//
// Created by abiel on 9/2/20.
//

#include "LuaMicroDronInterface.h"
#include <stdexcept>

LuaMicroDronInterface::LuaMicroDronInterface() {
    ;
}

void LuaMicroDronInterface::setMicroDronInterface(const std::shared_ptr<MicroDronInterface> &interfaceIn) {
    interface = interfaceIn;
}

void LuaMicroDronInterface::registerFunctions(lua_State *L) {
    if(!interface){
        throw std::runtime_error("No MicroDronInterface given!");
    }

    lua_register(L, "takeoff", LuaMicroDronInterface::lua_drone_takeoff);
}

int LuaMicroDronInterface::lua_drone_takeoff(lua_State *L) {
    double height = lua_tonumber(L, 1);
    interface->takeoff(height);
    return 0;
}