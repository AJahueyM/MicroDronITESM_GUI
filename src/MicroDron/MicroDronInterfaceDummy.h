//
// Created by abiel on 8/19/20.
//

#ifndef MICRODRONITESM_GUI_MICRODRONINTERFACEDUMMY_H
#define MICRODRONITESM_GUI_MICRODRONINTERFACEDUMMY_H

#include "MicroDronInterface.h"

class MicroDronInterfaceDummy : public MicroDronInterface {
public:
    SimplePID getPitchPid() const override;

    SimplePID getRollPid() const override;

    SimplePID getYawPid() const override;

    SimplePID getHeightPid() const override;

    void setPitchPid(SimplePID pitchPid) override;

    void setRollPid(SimplePID rollPid) override;

    void setYawPid(SimplePID yawPid) override;

    void setHeightPid(SimplePID heightPid) override;

    void sendHeartBeat() override;

    float getPitch() const override;

    float getRoll() const override;

    float getYaw() const override;

    float getHeight() const override;

    int getMode() const override;

    float getK() const override;

    float getMotorOutput1() const override;

    float getMotorOutput2() const override;

    float getMotorOutput3() const override;

    float getMotorOutput4() const override;

    void setAllMotorOutput(float output) override;

    void setAllMotorOutput(float output1, float output2, float output3, float output4) override;

    void setSetpoints(float pitch, float roll, float yaw, float height) override;

    void setK(float newK) override;

    void emergencyStop() override;

    bool isConnected() const override;

    float getHeartbeatTime() const override;
};


#endif //MICRODRONITESM_GUI_MICRODRONINTERFACEDUMMY_H
