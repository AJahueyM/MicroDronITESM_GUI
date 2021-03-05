//
// Created by abiel on 3/5/21.
//

#include "MainApp.h"

using namespace std::placeholders;

MainApp::MainApp() {
    luaState = luaL_newstate();
    luaL_openlibs(luaState);

    tarranisConfig.thrustAxis = sf::Joystick::X;
    tarranisConfig.rollAxis = sf::Joystick::Y;
    tarranisConfig.pitchAxis = sf::Joystick::Z;
    tarranisConfig.yawAxis = sf::Joystick::U;

    controller = std::make_unique<SFMLController>(tarranisConfig);

    LuaMicroDronInterface::setMicroDronInterface(std::shared_ptr<MicroDronInterface>(&interface));
    LuaMicroDronInterface::registerFunctions(luaState);
}

void MainApp::update() {
    ImGui::ShowDemoWindow();
    drawParamWindow();
    drawIMUPlots();
    showDroneControl();
    showScriptsWindow();

    if (sf::Joystick::isConnected(0)) {
        controller->sendControl(std::bind(&MicroDronInterface::sendJoystickControl, &interface, _1, _2, _3, _4));
    }
}

void MainApp::createPIDConfigMenu(const std::string &name, SimplePID &pid,
                                  const std::function<void(const SimplePID &)> &setFun, int id) {
    ImGui::Text("%s", name.c_str());
    ImGui::PushID(id);
    //ImGui::SameLine();
    ImGui::InputFloat("P", &pid.p, 0.1f, 0.0f, "%.5f");
    //ImGui::SameLine();
    ImGui::InputFloat("I", &pid.i, 0.1f, 0.0f, "%.5f");
    //ImGui::SameLine();
    ImGui::InputFloat("D", &pid.d, 0.1f, 0.0f, "%.5f");
    // ImGui::SameLine();
    if (ImGui::Button("Send")) {
        setFun(pid);
    }
    ImGui::Separator();

    ImGui::PopID();
}

void MainApp::createPIDStatus(const std::string &name, const SimplePID &pid){
    ImGui::Text("%s:   P: %.3f, I: %.3f, D: %.3f", name.c_str(), pid.p, pid.i, pid.d);
}

void MainApp::drawParamWindow() {
    static std::chrono::high_resolution_clock::time_point refreshParamsTime = std::chrono::high_resolution_clock::now();
    auto &params = interface.getParams();

    ImGui::Begin("Parameters", nullptr, 0);
    if(ImGui::Button("Request Parameters")){
        interface.requestParamList();
    }

    if(ImGui::TreeNode("Parameters")){
        ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable;

        if(ImGui::Button("Send Parameters")){
            for(const auto &sendParam : paramsToSend){
                if(sendParam.second){
                    const auto &paramToSend(params.at(sendParam.first));
                    mavlink_param_set_t paramSet;
                    paramSet.param_value = paramToSend.param_value;
                    snprintf(paramSet.param_id, sizeof(paramSet.param_id), "%s", paramToSend.param_id);
                    paramSet.param_type = paramToSend.param_type;

                    std::cout << fmt::format("Sending: {}", paramSet.param_id) << std::endl;

                    interface.setParameter(paramSet);

                    refreshParams = true;
                    refreshParamsTime = std::chrono::high_resolution_clock::now();
                }
            }
        }

        if(refreshParams && (std::chrono::high_resolution_clock::now() - refreshParamsTime) > std::chrono::milliseconds (500)){
            interface.requestParamList();
            refreshParams = false;
        }

        if(ImGui::BeginTable("Parameters", 4, flags)){
            ImGui::TableSetupColumn("Send on click", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Index", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();

            for(auto &param : params){
                ImGui::TableNextRow();
                int col = 0;
                ImGui::TableSetColumnIndex(col++);
                ImGui::Checkbox("",&(paramsToSend[param.first]));
                ImGui::TableSetColumnIndex(col++);
                ImGui::Text("%d", param.second.param_index);
                ImGui::TableSetColumnIndex(col++);
                ImGui::Text("%s", param.second.param_id);
                ImGui::TableSetColumnIndex(col++);

                //Needs to have a label for some reason, otherwise, checkbox does not work
                ImGui::InputFloat(" ", &param.second.param_value, 0.1f, 0.0f, "%.5f");
            }
            ImGui::EndTable();
        }
        ImGui::TreePop();
    }
    ImGui::End();
}

void MainApp::drawIMUPlots() {
    ImGui::Begin("Measurements", nullptr, 0);
    static RollingBuffer rollBuf, pitchBuf, yawBuf;
    static RollingBuffer m0, m1, m2, m3;
    const static std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();

    static ImPlotAxisFlags rt_axis = ImPlotFlags_AntiAliased;
    static float history = 10.0f;
    ImGui::SliderFloat("History",&history,1,30,"%.1f s");

    std::chrono::duration<double> duration = interface.getLastAttUpdateTime() - t;
    double tDelta = duration.count();
    rollBuf.AddPoint(tDelta, interface.getRoll());
    pitchBuf.AddPoint(tDelta, interface.getPitch());
    yawBuf.AddPoint(tDelta, interface.getYaw());

    ImPlot::SetNextPlotLimitsX(tDelta - history,tDelta, ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-180, 180, ImGuiCond_Always);
    if(ImPlot::BeginPlot("IMU (Degrees)", NULL, NULL, ImVec2(-1, 250), 0, rt_axis, rt_axis)){
        ImPlot::PlotLine("Roll", &rollBuf.Data[0].x, &rollBuf.Data[0].y, rollBuf.Data.size(), 0, 2 * sizeof(float));
        ImPlot::PlotLine("Pitch", &pitchBuf.Data[0].x, &pitchBuf.Data[0].y, pitchBuf.Data.size(), 0, 2 * sizeof(float));
        ImPlot::PlotLine("Yaw", &yawBuf.Data[0].x, &yawBuf.Data[0].y, yawBuf.Data.size(), 0, 2 * sizeof(float));
        ImPlot::EndPlot();
    }

    duration = interface.getLastMotorUpdate() - t;
    tDelta = duration.count();
    auto motorValues = interface.getMotorValues();
    m0.AddPoint(tDelta, motorValues.frontLeft);
    m1.AddPoint(tDelta, motorValues.frontRight);
    m2.AddPoint(tDelta, motorValues.backLeft);
    m3.AddPoint(tDelta, motorValues.backRight);

    ImPlot::SetNextPlotLimitsX(tDelta - history,tDelta, ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-10, 1000, ImGuiCond_Always);

    if(ImPlot::BeginPlot("Motor Outputs", NULL, NULL, ImVec2(-1, 250), 0, rt_axis, rt_axis)){
        ImPlot::PlotLine("Front Left", &m0.Data[0].x, &m0.Data[0].y, m0.Data.size(), 0, 2 * sizeof(float));
        ImPlot::PlotLine("Front Right", &m1.Data[0].x, &m1.Data[0].y, m1.Data.size(), 0, 2 * sizeof(float));
        ImPlot::PlotLine("Back Left", &m2.Data[0].x, &m2.Data[0].y, m2.Data.size(), 0, 2 * sizeof(float));
        ImPlot::PlotLine("Back Right", &m3.Data[0].x, &m3.Data[0].y, m3.Data.size(), 0, 2 * sizeof(float));
        ImPlot::EndPlot();
    }

    ImGui::End();
}

void MainApp::showDroneControl() {
    ImGui::Begin("Drone Control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::RadioButton(interface.isConnected() ? "Connected" : "Disconnected", interface.isConnected());
    ImGui::PlotVar("Drone heartbeat", interface.getHeartbeatTime(), 0, 100);
    if(interface.isEmergencyStopped()){
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Emergency stopped!");
    } else {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Running");
    }
    ImGui::End();
}

void MainApp::showScriptsWindow() {
    ImGui::Begin("Scripts", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    bool scriptButton = ImGui::Button("Run Script");
    if(scriptButton and scriptButton != lastScriptButton){
        std::cout << "Here" << std::endl;
        luaL_dofile(luaState, "helloDrone.lua");
    }
    lastScriptButton = scriptButton;
    ImGui::End();
}

MainApp::~MainApp() {
    free(luaState);
}
