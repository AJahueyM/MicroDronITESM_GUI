// Client side C/C++ program to demonstrate Socket programming 
#include <iostream>
#include "MicroDron/MicroDronInterfaceUDP.h"
#include "SFML/Graphics.hpp"
#include "imgui/imgui-SFML.h"
#include "imgui/imgui.h"
#include "imgui/PlotVar.h"
#include <chrono>
#include <fstream>
#include <functional>
#include "MicroDron/LuaMicroDronInterface.h"

void createPIDConfigMenu(const std::string &name, SimplePID &pid, const std::function<void(const SimplePID &pid)> &setFun, int id){
    ImGui::Text("%s", name.c_str());
    ImGui::PushID(id);
    //ImGui::SameLine();
    ImGui::InputFloat("P", &pid.p, 0.1f, 0.0f, "%.5f");
    //ImGui::SameLine();
    ImGui::InputFloat("I", &pid.i, 0.1f, 0.0f, "%.5f");
    //ImGui::SameLine();
    ImGui::InputFloat("D", &pid.d, 0.1f, 0.0f, "%.5f");
    // ImGui::SameLine();
    if(ImGui::Button("Send")){
        setFun(pid);
    }
    ImGui::Separator();

    ImGui::PopID();
}

void createPIDStatus(const std::string &name, const SimplePID &pid){
    ImGui::Text("%s:   P: %.3f, I: %.3f, D: %.3f", name.c_str(), pid.p, pid.i, pid.d);
}

int main(int argc, char const *argv[]){
    ///Create interface to drone

    MicroDronInterfaceUDP interface;
    //MicroDronInterfaceOLD interface("127.0.0.1", 51717);
    sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(1920 * .45),
                                          static_cast<unsigned int>(1080 * .75)), "MicroDron GUI", sf::Style::Close);
    window.setFramerateLimit(60);

    ImGui::CreateContext();
    ImGui::SFML::Init(window);

#ifdef __APPLE__
    /***
     * This need to be done so that the program runs on MacOS
     */
    ImGui::NewFrame();
#endif

    sf::Clock deltaClock;

    float motor1Target = 0.0f, motor2Target = 0.0f, motor3Target = 0.0f, motor4Target = 0.0f, allMotorTarget = 0.0f;
    float yawSetpoint = 0.0f, pitchSetpoint = 0.0f, rollSetpoint = 0.0f, heightSetpoint = 0.0f, k = 0.0f;
    SimplePID rollPid, pitchPid, yawPid, heightPid;

    float pitchOffset = 0.0f;
    float rollOffset = 0.0f;
    bool flying = false;
    bool lastE = false;

    std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();

    ///Retrive PID from file
    std::ifstream pidFile("pidSave.txt");
    std::string inputLine;
    std::getline(pidFile, inputLine);
    std::sscanf(inputLine.c_str(), "%f %f %f %f %f %f %f %f %f",
                &pitchPid.p , &pitchPid.i,  &pitchPid.d, &rollPid.p ,
                &rollPid.i , &rollPid.d,  &yawPid.p ,&yawPid.i , &yawPid.d );
    pidFile.close();


    yawPid.clamped = true;
    yawPid.maxOutput =  0.2;
    yawPid.minOutput = -0.2;
    yawPid.continuous = true;
    yawPid.maxInput =  180;
    yawPid.minInput = -180;

    rollPid.clamped = false;
    rollPid.continuous = true;
    rollPid.maxInput =  180;
    rollPid.minInput = -180;

    pitchPid.clamped = false;
    pitchPid.continuous = true;
    pitchPid.maxInput =  180;
    pitchPid.minInput = -180;

    lua_State *luaState;
    luaState = luaL_newstate();
    luaL_openlibs(luaState);

    LuaMicroDronInterface::setMicroDronInterface(std::shared_ptr<MicroDronInterface>(&interface));
    LuaMicroDronInterface::registerFunctions(luaState);

    window.setFramerateLimit(60);

    bool lastEstop;
    bool lastScriptButton{false};

    while (window.isOpen()) {
        std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(event);

            if (event.type == sf::Event::Closed) {
                window.close();
            } else if (event.type == sf::Event::Resized) {
                window.setView(
                        sf::View(sf::FloatRect(0, 0, event.size.width, event.size.height)));
            }
        }

        if(true or window.hasFocus()){
            ///Update UI and interactions

            ImGui::SFML::Update(window, deltaClock.restart());
            ImGui::Begin("Drone State", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::RadioButton(interface.isConnected() ? "Connected" : "Disconnected", interface.isConnected());
            ImGui::PlotVar("Pitch", interface.getPitch(), -90, 90);
            ImGui::PlotVar("Roll", interface.getRoll(), -90, 90);
            ImGui::PlotVar("Yaw", interface.getYaw(), -90, 90);
            ImGui::PlotVar("Height", interface.getHeight(), -5, 5);
            ImGui::PlotVar("Drone heartbeat", interface.getHeartbeatTime(), 0, 100);

            if(sf::Keyboard::isKeyPressed(sf::Keyboard::U)){
                interface.setK(k);
                interface.setPitchPid(pitchPid);
                interface.setRollPid(rollPid);
                interface.setYawPid(yawPid);
                interface.setHeightPid(heightPid);
            }

            bool eStopKey = sf::Keyboard::isKeyPressed(sf::Keyboard::F);
            if(eStopKey and eStopKey != lastEstop){
                interface.emergencyStop();
            }
            lastEstop = eStopKey;

            ImGui::Text("Current mode %i", interface.getMode());
            ImGui::End();

            ImGui::Begin("Drone Control", nullptr,ImGuiWindowFlags_AlwaysAutoResize);
            if(ImGui::Button("Emergency Stop")){
                interface.emergencyStop();
            }

            ImGui::SameLine();
            if(interface.isEmergencyStopped()){
                ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Emergency stopped!");
            } else {
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Running");
            }

            ImGui::Separator();
            ImGui::Text("Manual Control");
            ImGui::InputFloat("Motor1", &motor1Target);
            ImGui::InputFloat("Motor2", &motor2Target);
            ImGui::InputFloat("Motor3", &motor3Target);
            ImGui::InputFloat("Motor4", &motor4Target);
            if(ImGui::Button("Send Manual")){
                interface.setAllMotorOutput(motor1Target, motor2Target, motor3Target, motor4Target);
            }
//            interface.setAllMotorOutput(motor1Target, motor2Target, motor3Target, motor4Target);

            ImGui::InputFloat("All Motor Target", &allMotorTarget);
            if(ImGui::Button("Send All Manual")){
                interface.setAllMotorOutput(allMotorTarget);
            }

            ImGui::Separator();
            ImGui::Text("Setpoint Control");
            ImGui::RadioButton("Flying", flying);
            ImGui::InputFloat("Yaw", &yawSetpoint);
            ImGui::InputFloat("Pitch", &pitchSetpoint);
            ImGui::InputFloat("Pitch Offset", &pitchOffset);
            ImGui::InputFloat("Roll", &rollSetpoint);
            ImGui::InputFloat("Roll Offset", &rollOffset);
            ImGui::InputFloat("Height", &heightSetpoint);

            if(ImGui::Button("Send Setpoint")) {
                if(flying){
                    interface.setSetpoints(pitchSetpoint + pitchOffset, rollSetpoint + rollOffset, yawSetpoint, heightSetpoint);
                }
            }

            if(lastE != sf::Keyboard::isKeyPressed(sf::Keyboard::E) && sf::Keyboard::isKeyPressed(sf::Keyboard::E)){
                flying = !flying;
            }
            lastE = sf::Keyboard::isKeyPressed(sf::Keyboard::E);

            ImGui::End();

            ImGui::Begin("PID Setup", nullptr,0);
            createPIDStatus("Pitch", interface.getPitchPid());
            createPIDStatus("Roll", interface.getRollPid());
            createPIDStatus("Yaw", interface.getYawPid());
            createPIDStatus("Height", interface.getHeightPid());

            int objId = 0;

            ImGui::Separator();
            ImGui::PushID(objId);
            ImGui::Text("K Value");
            ImGui::InputFloat("K", &k);
            if(ImGui::Button("Send")){
                interface.setK(k);
            }

            ImGui::Separator();
            ImGui::PopID();

            objId++;
            createPIDConfigMenu("Pitch", pitchPid, [&](const SimplePID &pid){interface.setPitchPid(pitchPid);}, objId);

            objId++;
            createPIDConfigMenu("Roll", rollPid, [&](const SimplePID &pid){interface.setRollPid(rollPid);}, objId);

            objId++;
            createPIDConfigMenu("Yaw", yawPid, [&](const SimplePID &pid){interface.setYawPid(yawPid);}, objId);

            objId++;
            createPIDConfigMenu("Height", heightPid, [&](const SimplePID &pid){interface.setHeightPid(heightPid);}, objId);

            if(ImGui::Button("Save PID to file")){
                std::ofstream pidFile("pidSave.txt");
                pidFile << pitchPid.p << " "
                        << pitchPid.i << " "
                        << pitchPid.d << " "

                        << rollPid.p << " "
                        << rollPid.i << " "
                        << rollPid.d << " "

                        << yawPid.p << " "
                        << yawPid.i << " "
                        << yawPid.d;
                pidFile.close();
            }
            ImGui::SameLine();

            if(ImGui::Button("Load PID from file")){
                std::ifstream pidFile("pidSave.txt");
                std::string inputLine;
                std::getline(pidFile, inputLine);
                std::sscanf(inputLine.c_str(), "%f %f %f %f %f %f %f %f %f",
                            &pitchPid.p , &pitchPid.i,  &pitchPid.d, &rollPid.p ,
                            &rollPid.i , &rollPid.d,  &yawPid.p ,&yawPid.i , &yawPid.d );
                pidFile.close();
            }
            ImGui::End();

            ImGui::Begin("Scripts", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
            bool scriptButton = ImGui::Button("Run Script");
            if(scriptButton and scriptButton != lastScriptButton){
                luaL_dofile(luaState, "helloDrone.lua");
            }
            lastScriptButton = scriptButton;
            ImGui::End();
        }

        window.clear(sf::Color(94,94,94));

        ImGui::SFML::Render(window);

        interface.sendHeartBeat();

        if(sf::Joystick::isConnected(0)){
            float x = sf::Joystick::getAxisPosition(0, sf::Joystick::X) * 10.0;
            float y = sf::Joystick::getAxisPosition(0, sf::Joystick::Y) * 10.0;
            float z = -sf::Joystick::getAxisPosition(0, sf::Joystick::V) * 10.0;
            float r = sf::Joystick::getAxisPosition(0, sf::Joystick::U) * 10.0;

            interface.sendJoystickControl(x, y, z, r);
        }

        window.display();
    }

    free(luaState);
    window.close();
    return 0;
}
