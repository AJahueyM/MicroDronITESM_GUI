//
// Created by abiel on 11/9/20.
//

#include "SFMLController.h"
#include <iostream>
#include <fmt/format.h>

SFMLController::SFMLController(const SFMLControllerConfig &config) {
    this->config = config;
    thrust = JoystickAxisExpo(config.expoValue, config.deadzone);
    roll = JoystickAxisExpo(config.expoValue, config.deadzone);
    pitch = JoystickAxisExpo(config.expoValue, config.deadzone);
    yaw = JoystickAxisExpo(config.expoValue, config.deadzone);
}

float SFMLController::clampOutput(float in) {
    return in;
}

void SFMLController::sendControl(const std::function<void(int16_t, int16_t, int16_t, int16_t)> &sendFun) {
    float thrustOutput = thrust.calculateOutput(sf::Joystick::getAxisPosition(0, config.thrustAxis) / 100.0);
    float rollOutput = roll.calculateOutput((sf::Joystick::getAxisPosition(0, config.rollAxis) / 100.0 - 0.5) * 2.0);
    float pitchOutput = pitch.calculateOutput((sf::Joystick::getAxisPosition(0, config.pitchAxis) / 100.0 - 0.5) * 2.0);
    float yawOutput = yaw.calculateOutput((sf::Joystick::getAxisPosition(0, config.yawAxis) / 100.0 - 0.5) * 2.0);

    //std::cout << fmt::format("Roll: {}, Pitch: {}, Yaw: {}, Thrust: {}", rollOutput, pitchOutput, yawOutput, thrustOutput) << std::endl;

    sendFun(rollOutput * 10000.0f, pitchOutput * 10000.0f, thrustOutput * 10000.0f, yawOutput * 10000.0f);
}