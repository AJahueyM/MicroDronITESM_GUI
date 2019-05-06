// Client side C/C++ program to demonstrate Socket programming 
#include <iostream>
#include "MicroDron/MicroDronInterface.h"
#include "SFML/Graphics.hpp"
#include "imgui/imgui-SFML.h"
#include "imgui/imgui.h"
#include "imgui/PlotVar.h"
#include <chrono>
#include <fstream>
#include "ProyectoIntegrador/FinalIntegrador/src/ProyectoIntegrador.h"

int main(int argc, char const *argv[]){

    MicroDronInterface interface("192.168.4.1", 23);
    sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(1200),
                                          static_cast<unsigned int>(650)), "MicroDron GUI", sf::Style::Close);
    window.setFramerateLimit(60);

    ImGui::CreateContext();
    ImGui::SFML::Init(window);
    sf::Clock deltaClock;

    float motor1Target = 0.0f, motor2Target = 0.0f, motor3Target = 0.0f, motor4Target = 0.0f, allMotorTarget = 0.0f;
    float yawSetpoint = 0.0f, pitchSetpoint = 0.0f, rollSetpoint = 0.0f, heightSetpoint = 0.0f, k = 0.0f;
    SimplePID rollPid, pitchPid, yawPid, heightPid;

    std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();

    std::ifstream pidFile("pidSave.txt");
    std::string inputLine;
    std::getline(pidFile, inputLine);
    std::sscanf(inputLine.c_str(), "%f %f %f %f %f %f %f %f %f",
                &pitchPid.p , &pitchPid.i,  &pitchPid.d, &rollPid.p ,
                &rollPid.i , &rollPid.d,  &yawPid.p ,&yawPid.i , &yawPid.d );
    pidFile.close();

    setMicroDronInterface(interface);
    initProyectoIntegrador();
    bool lastHKeyValue = false, showHandMastk = false;

    sf::Image visionOutImage, visionTelemImage;
    sf::Texture visionOutTexture , visionTelemTexture;
    sf::Sprite visionOutSprite , visionTelemSprite;

    while (window.isOpen()) {
        std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
        double  timeSeconds = std::chrono::duration<double>(currentTime - startTime).count();



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

        if(window.hasFocus()){
            ImGui::SFML::Update(window, deltaClock.restart());
            ImGui::Begin("Drone State", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::RadioButton(interface.isConnected() ? "Connected" : "Disconnected", interface.isConnected());
            ImGui::PlotVar("Pitch", interface.getPitch(), -90, 90);
            ImGui::PlotVar("Roll", interface.getRoll(), -90, 90);
            ImGui::PlotVar("Yaw", interface.getYaw(), -90, 90);
            ImGui::PlotVar("Height", interface.getHeight(), -5, 5);
            ImGui::PlotVar("Drone heartbeat", interface.getHeartbeatTime(), 0, 500);

            if(sf::Keyboard::isKeyPressed(sf::Keyboard::U)){
                interface.setK(k);
                interface.setPitchPid(pitchPid);
                interface.setRollPid(rollPid);
                interface.setYawPid(yawPid);
                interface.setHeightPid(heightPid);
            }

            if(sf::Keyboard::isKeyPressed(sf::Keyboard::F)){
                interface.emergencyStop();
            }
            shouldStop(sf::Keyboard::isKeyPressed(sf::Keyboard::F));

            ImGui::Text("Current mode %i", interface.getMode());
            ImGui::End();

            ImGui::Begin("Drone Control", nullptr,ImGuiWindowFlags_AlwaysAutoResize);
            if(ImGui::Button("Emergency stop")){
                interface.emergencyStop();
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

            ImGui::InputFloat("All Motor Target", &allMotorTarget);
            if(ImGui::Button("Send All Manual")){
                interface.setAllMotorOutput(allMotorTarget);


            }

            ImGui::Separator();
            ImGui::Text("Setpoint Control");
            ImGui::InputFloat("Yaw", &yawSetpoint);
            ImGui::InputFloat("Pitch", &pitchSetpoint);
            ImGui::InputFloat("Roll", &rollSetpoint);
            ImGui::InputFloat("Height", &heightSetpoint);

            if(ImGui::Button("Send Setpoint") || sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
                interface.setSetpoints(pitchSetpoint, rollSetpoint, yawSetpoint, heightSetpoint);
            }
            ImGui::End();

            ImGui::Begin("PID Setup", nullptr,ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::Text("Pitch   P:%f I:%f D:%f", interface.getPitchPid().p,interface.getPitchPid().i,interface.getPitchPid().d);
            ImGui::Text("Roll    P:%f I:%f D:%f", interface.getRollPid().p,interface.getRollPid().i,interface.getRollPid().d);
            ImGui::Text("Yaw     P:%f I:%f D:%f", interface.getYawPid().p,interface.getYawPid().i,interface.getYawPid().d);
            ImGui::Text("Height  P:%f I:%f D:%f", interface.getHeightPid().p,interface.getHeightPid().i,interface.getHeightPid().d);
            ImGui::Text("K Value  %f", interface.getK());

            ImGui::Separator();
            ImGui::PushID(4);
            ImGui::Text("K Value");
            ImGui::InputFloat("K", &k);
            ImGui::SameLine();
            if(ImGui::Button("Send")){
                interface.setK(k);
            }
            ImGui::PopID();
            ImGui::Text("Pitch");
            ImGui::PushID(0);
            ImGui::SameLine();
            ImGui::InputFloat("P", &pitchPid.p, 0.0f, 0.0f, "%.5f");
            ImGui::SameLine();
            ImGui::InputFloat("I", &pitchPid.i, 0.0f, 0.0f, "%.5f");
            ImGui::SameLine();
            ImGui::InputFloat("D", &pitchPid.d, 0.0f, 0.0f, "%.5f");
            ImGui::SameLine();
            if(ImGui::Button("Send")){
                interface.setPitchPid(pitchPid);
            }
            ImGui::PopID();

            ImGui::PushID(1);
            ImGui::Text("Roll");
            ImGui::SameLine();
            ImGui::InputFloat("P", &rollPid.p, 0.0f, 0.0f, "%.5f");
            ImGui::SameLine();
            ImGui::InputFloat("I", &rollPid.i, 0.0f, 0.0f, "%.5f");
            ImGui::SameLine();
            ImGui::InputFloat("D", &rollPid.d, 0.0f, 0.0f, "%.5f");
            ImGui::SameLine();
            if(ImGui::Button("Send")){
                interface.setRollPid(rollPid);

            }
            ImGui::PopID();

            ImGui::PushID(2);
            ImGui::Text("Yaw");
            ImGui::SameLine();
            ImGui::InputFloat("P", &yawPid.p, 0.0f, 0.0f, "%.5f");
            ImGui::SameLine();
            ImGui::InputFloat("I", &yawPid.i, 0.0f, 0.0f, "%.5f");
            ImGui::SameLine();
            ImGui::InputFloat("D", &yawPid.d, 0.0f, 0.0f, "%.5f");
            ImGui::SameLine();
            if(ImGui::Button("Send")){
                interface.setYawPid(yawPid);

            }
            ImGui::PopID();

            ImGui::PushID(3);
            ImGui::Text("Height");
            ImGui::SameLine();
            ImGui::InputFloat("P", &heightPid.p, 0.0f, 0.0f, "%.5f");
            ImGui::SameLine();
            ImGui::InputFloat("I", &heightPid.i, 0.0f, 0.0f, "%.5f");
            ImGui::SameLine();
            ImGui::InputFloat("D", &heightPid.d, 0.0f, 0.0f, "%.5f");
            ImGui::SameLine();
            if(ImGui::Button("Send")){
                interface.setHeightPid(heightPid);

            }
            ImGui::PopID();

            if(ImGui::Button("Save PID")){
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

            if(ImGui::Button("Load PID")){
                std::ifstream pidFile("pidSave.txt");
                std::string inputLine;
                std::getline(pidFile, inputLine);
                std::sscanf(inputLine.c_str(), "%f %f %f %f %f %f %f %f %f",
                            &pitchPid.p , &pitchPid.i,  &pitchPid.d, &rollPid.p ,
                            &rollPid.i , &rollPid.d,  &yawPid.p ,&yawPid.i , &yawPid.d );
                pidFile.close();
            }
            ImGui::End();
        }

        window.clear();

        ImGui::SFML::Render(window);

        //interface.sendHeartBeat();

        shouldCalibrateA(sf::Keyboard::isKeyPressed(sf::Keyboard::C));
        shouldCalibrateB(sf::Keyboard::isKeyPressed(sf::Keyboard::B));
        shouldActivate(sf::Keyboard::isKeyPressed(sf::Keyboard::A));

        proyectoIntegrador();
        const Mat& visionOut = getVisionOutput();
        const Mat& visionTelemetry = showHandMastk ? getHandMask() : getContours();

        Mat visionOutRGB, visionTelemRGB;
        cvtColor(visionOut, visionOutRGB, COLOR_BGR2RGBA);
        resize(visionOutRGB, visionOutRGB, cv::Size(640 * 0.7,360*0.7));

        if(!showHandMastk){
            cvtColor(visionTelemetry, visionTelemRGB, COLOR_BGR2RGBA);
        }else{
            cvtColor(visionTelemetry, visionTelemRGB, COLOR_GRAY2RGBA);
        }

        resize(visionTelemRGB, visionTelemRGB, cv::Size(640 * 0.7,360*0.7));

        visionOutImage.create(visionOutRGB.cols, visionOutRGB.rows, visionOutRGB.ptr());
        if (!visionOutTexture.loadFromImage(visionOutImage))
        {
            break;
        }

        visionOutSprite.setTexture(visionOutTexture);

        visionTelemImage.create(visionTelemRGB.cols, visionTelemRGB.rows, visionTelemRGB.ptr());
        if (!visionTelemTexture.loadFromImage(visionTelemImage))
        {
            break;
        }

        visionTelemSprite.setTexture(visionTelemTexture);

        bool currentHKey = sf::Keyboard::isKeyPressed(sf::Keyboard::H);

        if(currentHKey!= lastHKeyValue  && currentHKey){
            showHandMastk = !showHandMastk;
        }

        visionOutSprite.setPosition(727, 61);
        visionTelemSprite.setPosition(783, 377);

        window.draw(visionTelemSprite);
        window.draw(visionOutSprite);

        lastHKeyValue =currentHKey;

        window.display();


    }
    stopProyectoIntegrador();
    window.close();
    return 0;
} 
