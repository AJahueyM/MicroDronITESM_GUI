// Client side C/C++ program to demonstrate Socket programming 
#include <iostream>
#include "MicroDron/MicroDronInterfaceUDP.h"
#include "SFML/Graphics.hpp"
#include "imgui-SFML.h"
#include "imgui.h"
#include "PlotVar.h"
#include <chrono>
#include <fstream>
#include <functional>
#include "MicroDron/LuaMicroDronInterface.h"
#include "implot.h"
#include "MainApp.h"

using namespace std::placeholders;

int main(int argc, char const *argv[]){
    sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(1920 * .75),
                                          static_cast<unsigned int>(1080 * .75)), "MicroDron GUI", sf::Style::Close);
    window.setFramerateLimit(60);

    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::SFML::Init(window);

    MainApp app;

#ifdef __APPLE__
    /***
     * This need to be done so that the program runs on MacOS
     */
    ImGui::NewFrame();
#endif

    window.setFramerateLimit(60);

    while (window.isOpen()) {
        sf::Clock deltaClock;

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
            ImGui::SFML::Update(window, deltaClock.restart());
            app.update();
        }

        window.clear(sf::Color(94,94,94));

        ImGui::SFML::Render(window);

        window.display();
    }

    window.close();

    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    return 0;
}
