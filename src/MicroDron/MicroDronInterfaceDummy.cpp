//
// Created by abiel on 8/19/20.
//

#include "MicroDronInterfaceDummy.h"

void MicroDronInterfaceDummy::setPitchPid(SimplePID pitchPid) {

}

void MicroDronInterfaceDummy::setRollPid(SimplePID rollPid) {

}

void MicroDronInterfaceDummy::setYawPid(SimplePID yawPid) {

}

void MicroDronInterfaceDummy::setHeightPid(SimplePID heightPid) {

}

void MicroDronInterfaceDummy::sendHeartBeat() {

}

float MicroDronInterfaceDummy::getPitch() const {
    return 0;
}

float MicroDronInterfaceDummy::getRoll() const {
    return 0;
}

float MicroDronInterfaceDummy::getYaw() const {
    return 0;
}

float MicroDronInterfaceDummy::getHeight() const {
    return 0;
}

int MicroDronInterfaceDummy::getMode() const {
    return 0;
}

float MicroDronInterfaceDummy::getK() const {
    return 0;
}

float MicroDronInterfaceDummy::getMotorOutput1() const {
    return 0;
}

float MicroDronInterfaceDummy::getMotorOutput2() const {
    return 0;
}

float MicroDronInterfaceDummy::getMotorOutput3() const {
    return 0;
}

float MicroDronInterfaceDummy::getMotorOutput4() const {
    return 0;
}

void MicroDronInterfaceDummy::setAllMotorOutput(float output) {

}

void MicroDronInterfaceDummy::setAllMotorOutput(float output1, float output2, float output3, float output4) {

}

void MicroDronInterfaceDummy::setSetpoints(float pitch, float roll, float yaw, float height) {

}

void MicroDronInterfaceDummy::setK(float newK) {

}

void MicroDronInterfaceDummy::emergencyStop() {

}

bool MicroDronInterfaceDummy::isConnected() const {
    return false;
}

float MicroDronInterfaceDummy::getHeartbeatTime() const {
    return 0;
}

SimplePID MicroDronInterfaceDummy::getPitchPid() const {
    return SimplePID();
}

SimplePID MicroDronInterfaceDummy::getRollPid() const {
    return SimplePID();
}

SimplePID MicroDronInterfaceDummy::getYawPid() const {
    return SimplePID();
}

SimplePID MicroDronInterfaceDummy::getHeightPid() const {
    return SimplePID();
}
