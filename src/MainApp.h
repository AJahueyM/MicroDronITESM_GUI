//
// Created by abiel on 3/5/21.
//

#ifndef MICRODRONITESM_GUI_MAINAPP_H
#define MICRODRONITESM_GUI_MAINAPP_H

#include "imgui-SFML.h"
#include "imgui.h"
#include "MicroDron/MicroDronInterfaceUDP.h"
#include "MicroDron/LuaMicroDronInterface.h"
#include "PlotVar.h"
#include "implot.h"
#include "Utils/SFMLController.h"
#include <boost/circular_buffer.hpp>

class MainApp {
public:
    MainApp();

    ~MainApp();

    void update();
private:
    MicroDronInterfaceUDP interface;

    lua_State *luaState;

    std::map<int, bool> paramsToSend;

    bool refreshParams = true;

    SFMLControllerConfig tarranisConfig;
    std::unique_ptr<SFMLController> controller;

    bool lastScriptButton = false;

    void createPIDConfigMenu(const std::string &name, SimplePID &pid, const std::function<void(const SimplePID &pid)> &setFun, int id);

    void createPIDStatus(const std::string &name, const SimplePID &pid);

    void drawParamWindow();

    void drawIMUPlots();

    void showDroneControl();

    void showScriptsWindow();
};


#endif //MICRODRONITESM_GUI_MAINAPP_H
