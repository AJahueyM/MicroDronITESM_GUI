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

    // utility structure for realtime plot
    struct RollingBuffer {
        float Span;
        ImVector<ImVec2> Data;
        RollingBuffer() {
            Span = 100.0f;
            Data.reserve(30000);
        }
        void AddPoint(float x, float y) {
            float xmod = fmodf(x, Span);
            if (!Data.empty() && xmod < Data.back().x)
                Data.shrink(0);
            Data.push_back(ImVec2(xmod, y));
        }
    };

    void createPIDConfigMenu(const std::string &name, SimplePID &pid, const std::function<void(const SimplePID &pid)> &setFun, int id);

    void createPIDStatus(const std::string &name, const SimplePID &pid);

    void drawParamWindow();

    void drawIMUPlots();

    void showDroneControl();

    void showScriptsWindow();
};


#endif //MICRODRONITESM_GUI_MAINAPP_H
