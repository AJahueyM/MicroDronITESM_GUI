// Client side C/C++ program to demonstrate Socket programming 
#include <iostream>
#include "MicroDron/MicroDronInterface.h"
#include "SFML/Graphics.hpp"
#include "imgui/imgui-SFML.h"
#include "imgui/imgui.h"
#include "imgui/PlotVar.h"

int main(int argc, char const *argv[])
{
    MicroDronInterface interface("192.168.4.1", 23);

    sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(800),
                                          static_cast<unsigned int>(600)), "MicroDron GUI", sf::Style::Close);
    window.setFramerateLimit(60);

    ImGui::CreateContext();
    ImGui::SFML::Init(window);
    sf::Clock deltaClock;

    float motor1Target = 0.0f, motor2Target = 0.0f, motor3Target = 0.0f, motor4Target = 0.0f, allMotorTarget = 0.0f;
    float yawSetpoint = 0.0f, pitchSetpoint = 0.0f, rollSetpoint = 0.0f, heightSetpoint = 0.0f;


    while (window.isOpen()) {
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::A)){
            if(interface.getMotorOutput1() != 200) {
                interface.setAllMotorOutput(200);
            }
        }else{
            if(interface.getMotorOutput1() != 0) {
                interface.setAllMotorOutput(0);
            }
        }

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

        ImGui::SFML::Update(window, deltaClock.restart());

        ImGui::Begin("Drone State", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::RadioButton("Connected", interface.isConnected());
        ImGui::PlotVar("Pitch", interface.getPitch(), -180, 180);
        ImGui::PlotVar("Roll", interface.getRoll(), -180, 180);
        ImGui::PlotVar("Yaw", interface.getYaw(), -180, 180);
        ImGui::PlotVar("Height", interface.getHeight(), 0, 100);
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

        if(ImGui::Button("Send Setpoint")) {
            interface.setSetpoints(pitchSetpoint, rollSetpoint, yawSetpoint, heightSetpoint);
        }

        ImGui::End();


        window.clear(sf::Color(100,100,100));

        ImGui::SFML::Render(window);
        window.display();

    }

    return 0;
} 
