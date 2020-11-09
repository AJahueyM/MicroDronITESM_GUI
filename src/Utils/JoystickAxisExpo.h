//
// Created by Abiel on 9/11/18.
//

#pragma once

class JoystickAxisExpo {
public:
    JoystickAxisExpo(double expoValue, double deadZone);
    JoystickAxisExpo(){
        ;
    }

    double calculateOutput(double value);
    double applyDeadzone(double value);
private:
    double expoValue = 1.0;
    double deadZone = 0.0;
    double slope = 0.0;
    double offset = 0.0;

    static constexpr double MAX_VALUE = 1.0;
};
