//
// Created by abiel on 9/2/20.
//

#ifndef MICRODRONITESM_GUI_LUAMICRODRONINTERFACE_H
#define MICRODRONITESM_GUI_LUAMICRODRONINTERFACE_H

#include <memory>
#include "MicroDronInterface.h"
#include <lua.hpp>

/**
 * Declared as a singleton as lua requires static functions
 */
class LuaMicroDronInterface {
public:
    static LuaMicroDronInterface& getInstance(){
        static LuaMicroDronInterface instance;
        return instance;
    }

    static void setMicroDronInterface(const std::shared_ptr<MicroDronInterface> &interfaceIn);
    static void registerFunctions(lua_State *L);

    LuaMicroDronInterface(const LuaMicroDronInterface&) = delete;
    LuaMicroDronInterface(LuaMicroDronInterface&&) = delete;
    LuaMicroDronInterface& operator=(const LuaMicroDronInterface&) = delete;
    LuaMicroDronInterface& operator=(LuaMicroDronInterface&&) = delete;

protected:
    static int lua_drone_takeoff(lua_State *L);

    static int lua_drone_land(lua_State *L);

    static int lua_drone_estop(lua_State *L);

    static int lua_drone_set_rot(lua_State *L);

    static int lua_drone_hover(lua_State *L);

    static int lua_drone_get_height(lua_State *L);

    static int lua_sleep_ms(lua_State *L);

private:
    LuaMicroDronInterface();

    inline static std::shared_ptr<MicroDronInterface> interface;
};


#endif //MICRODRONITESM_GUI_LUAMICRODRONINTERFACE_H
