//
// Created by abiel on 10/6/20.
//

#ifndef MICRODRONITESM_GUI_SWARMUDPINTERFACE_H
#define MICRODRONITESM_GUI_SWARMUDPINTERFACE_H

#include "MicroDronInterface.h"
#include "MicroDronInterfaceUDP.h"
#include <vector>

class SwarmUDPInterface : public MicroDronInterface {
public:
    SwarmUDPInterface(size_t n, uint16_t sendPort = 14551, uint16_t recvPort = 14550);

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

    void setSetpoints(float roll, float pitch, float yaw, float height) override;

    void setK(float newK) override;

    void emergencyStop() override;

    bool isEmergencyStopped() const override;

    bool isConnected() const override;

    float getHeartbeatTime() const override;

    void sendJoystickControl(int16_t x, int16_t y, int16_t z, int16_t r) override;

    void takeoff(double height) override;

private:
    std::vector<std::unique_ptr<MicroDronInterfaceUDP>> drones;
};


#endif //MICRODRONITESM_GUI_SWARMUDPINTERFACE_H
