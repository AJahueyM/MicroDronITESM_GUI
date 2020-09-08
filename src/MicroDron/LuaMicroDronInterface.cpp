//
// Created by abiel on 9/2/20.
//

#include "LuaMicroDronInterface.h"
#include <stdexcept>
#include <thread>

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
    lua_register(L, "land", LuaMicroDronInterface::lua_drone_land);
    lua_register(L, "estop", LuaMicroDronInterface::lua_drone_estop);
    lua_register(L, "setRPY", LuaMicroDronInterface::lua_drone_set_rot);
    lua_register(L, "hover", LuaMicroDronInterface::lua_drone_hover);
    lua_register(L, "getHeight", LuaMicroDronInterface::lua_drone_get_height);
    lua_register(L, "sleepMS", LuaMicroDronInterface::lua_sleep_ms);
}

int LuaMicroDronInterface::lua_sleep_ms(lua_State *L) {
    double ms = lua_tonumber(L, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds((int) ms));
    return 0;
}

int LuaMicroDronInterface::lua_drone_takeoff(lua_State *L) {
    double height = lua_tonumber(L, 1);
    //interface->takeoff(height);
    interface->sendJoystickControl(0,0,800,0);
    return 0;
}

int LuaMicroDronInterface::lua_drone_land(lua_State *L) {
    //interface->land();
    std::cout<<"LAND" <<std::endl;
    interface->sendJoystickControl(0,0,0,0);
    return 0;
}

int LuaMicroDronInterface::lua_drone_estop(lua_State *L) {
    interface->emergencyStop();
    return 0;
}

int LuaMicroDronInterface::lua_drone_set_rot(lua_State *L) {
    double r, p, y;

    r = lua_tonumber(L, 1);
    p = lua_tonumber(L, 2);
    y = lua_tonumber(L, 3);

    //interface->setSetpoints(r, p, y);

    if(lua_gettop(L) == 4){
        double time = lua_tonumber(L, 4);

        //Send commands and wait
        std::this_thread::sleep_for(std::chrono::milliseconds((int) (time * 1000.0)));
    }

    return 0;
}

int LuaMicroDronInterface::lua_drone_hover(lua_State *L) {
    double height = lua_tonumber(L, 1);

    //interface->setHeight(height);
    std::cout << fmt::format("Hovering at: {}", height) << std::endl;

    if(lua_gettop(L) > 1){
        double time = lua_tonumber(L, 2);

        //Send commands and wait
        std::this_thread::sleep_for(std::chrono::milliseconds((int) (time * 1000.0)));
    }

    return 0;
}

int LuaMicroDronInterface::lua_drone_get_height(lua_State *L) {
    lua_pushnumber(L, 1.3); //TODO actually call getHeight
    return 1;
}