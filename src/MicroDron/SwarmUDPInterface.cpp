//
// Created by abiel on 10/6/20.
//

#include "SwarmUDPInterface.h"
#include <algorithm>
#include <numeric>

SwarmUDPInterface::SwarmUDPInterface(size_t n, uint16_t sendPort, uint16_t recvPort) {
    for(int i = 0; i < n; ++i){
        drones.emplace_back(std::make_unique<MicroDronInterfaceUDP>(sendPort, recvPort));
        sendPort += 2; recvPort += 2;
    }
}

SimplePID SwarmUDPInterface::getPitchPid() const {
    return SimplePID();
}

SimplePID SwarmUDPInterface::getRollPid() const {
    return SimplePID();
}

SimplePID SwarmUDPInterface::getYawPid() const {
    return SimplePID();
}

SimplePID SwarmUDPInterface::getHeightPid() const {
    return SimplePID();
}

void SwarmUDPInterface::setPitchPid(SimplePID pitchPid) {

}

void SwarmUDPInterface::setRollPid(SimplePID rollPid) {

}

void SwarmUDPInterface::setYawPid(SimplePID yawPid) {

}

void SwarmUDPInterface::setHeightPid(SimplePID heightPid) {

}

void SwarmUDPInterface::sendHeartBeat() {

}

float SwarmUDPInterface::getPitch() const {
    return 0;
}

float SwarmUDPInterface::getRoll() const {
    return 0;
}

float SwarmUDPInterface::getYaw() const {
    return 0;
}

float SwarmUDPInterface::getHeight() const {
    return 0;
}

int SwarmUDPInterface::getMode() const {
    return 0;
}

float SwarmUDPInterface::getK() const {
    return 0;
}

float SwarmUDPInterface::getMotorOutput1() const {
    return 0;
}

float SwarmUDPInterface::getMotorOutput2() const {
    return 0;
}

float SwarmUDPInterface::getMotorOutput3() const {
    return 0;
}

float SwarmUDPInterface::getMotorOutput4() const {
    return 0;
}

void SwarmUDPInterface::setAllMotorOutput(float output) {

}

void SwarmUDPInterface::setAllMotorOutput(float output1, float output2, float output3, float output4) {

}

void SwarmUDPInterface::setSetpoints(float roll, float pitch, float yaw, float height) {
    for(auto &drone : drones){
        drone->setSetpoints(roll, pitch, yaw, height);
    }
}

void SwarmUDPInterface::setK(float newK) {

}

void SwarmUDPInterface::emergencyStop() {

}

bool SwarmUDPInterface::isEmergencyStopped() const {
    return false;
}

bool SwarmUDPInterface::isConnected() const {
    return std::all_of(drones.begin(), drones.end(),
                       [](const std::unique_ptr<MicroDronInterfaceUDP> &d){return d->isConnected();
    });
}

float SwarmUDPInterface::getHeartbeatTime() const {
    float sum = 0;
    for(const auto &drone : drones){
        sum += drone->getHeartbeatTime();
    }

    return sum / drones.size();
}

void SwarmUDPInterface::sendJoystickControl(int16_t x, int16_t y, int16_t z, int16_t r) {
    for(auto &drone : drones){
        drone->sendJoystickControl(x, y, z, r);
    }
}

void SwarmUDPInterface::takeoff(double height) {
    MicroDronInterface::takeoff(height);
}
