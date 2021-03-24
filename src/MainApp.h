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

struct ScrollingBuffer {
    size_t MaxSize;
    int Offset;
    std::vector<ImVec2> Data;
    ScrollingBuffer(size_t max_size = 2500) {
        MaxSize = max_size;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        auto l = Data.size();
        if (Data.size() < MaxSize)
            Data.emplace_back(x,y);
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        return;
        if (Data.size() > 0) {
            Data.resize(0);
            Offset  = 0;
        }
    }
};

class MainApp {
public:
    MainApp();

    ~MainApp();

    void update();
private:
    MicroDronInterfaceUDP interface;

    lua_State *luaState;

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
