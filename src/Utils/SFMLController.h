//
// Created by abiel on 11/9/20.
//

#ifndef MICRODRONITESM_GUI_SFMLCONTROLLER_H
#define MICRODRONITESM_GUI_SFMLCONTROLLER_H

#include <SFML/Window/Joystick.hpp>
#include <functional>
#include "JoystickAxisExpo.h"

struct SFMLControllerConfig{
    double deadzone = 0.1;
    double expoValue = 0.1;

    sf::Joystick::Axis thrustAxis;
    sf::Joystick::Axis pitchAxis;
    sf::Joystick::Axis rollAxis;
    sf::Joystick::Axis yawAxis;
};

class SFMLController {
public:
    SFMLController(const SFMLControllerConfig &config);

    void sendControl(const std::function<void(int16_t, int16_t, int16_t, int16_t)> &sendFun);
private:
    float clampOutput(float in);

    SFMLControllerConfig config;
    JoystickAxisExpo thrust, pitch, roll, yaw;
};


#endif //MICRODRONITESM_GUI_SFMLCONTROLLER_H
