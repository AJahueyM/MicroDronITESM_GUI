// Client side C/C++ program to demonstrate Socket programming 
#include <iostream>
#include "MicroDron/MicroDronInterfaceUDP.h"
#include <chrono>
#include <fstream>
#include <functional>
#include "Utils/SFMLController.h"
#include "MainApp.h"

int main(int argc, char const *argv[]){
    //TODO Re-implement controller input
//    tarranisConfig.thrustAxis = sf::Joystick::X;
//    tarranisConfig.rollAxis = sf::Joystick::Y;
//    tarranisConfig.pitchAxis = sf::Joystick::Z;
//    tarranisConfig.yawAxis = sf::Joystick::U;

    //TODO Implement support for various controllers
    //controller = std::make_unique<SFMLController>(tarranisConfig);

    MainApp app;
    app.run();

    return 0;
}
