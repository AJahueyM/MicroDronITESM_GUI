//
// Created by abiel on 8/19/20.
//

#include "MicroDronInterfaceDummy.h"
#include <arpa/inet.h>
#include <cmath>
#include <chrono>

MicroDronInterfaceDummy::MicroDronInterfaceDummy() {
    mavlink_attitude_t initialAttitude;
    initialAttitude.yaw = 0;
    initialAttitude.pitch = 0;
    initialAttitude.roll = 0;
    attitude.store(initialAttitude);

    conn = std::make_unique<UDPConnection>("127.0.0.1", 14550);
    conn->startConnection();

//    ret = fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC);
//    if(ret < 0){
//        perror("Nonblocking set failed");
//        close(sock);
//        exit(EXIT_FAILURE);
//    }
    updateThread = std::thread(&MicroDronInterfaceDummy::update, this);
}

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
    return attitude.load().pitch * 180.0 / M_PI;
}

float MicroDronInterfaceDummy::getRoll() const {
    return attitude.load().roll * 180.0 / M_PI;
}

float MicroDronInterfaceDummy::getYaw() const {
    return attitude.load().yaw * 180.0 / M_PI;
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
    emergencyStopped = !emergencyStopped;
}

bool MicroDronInterfaceDummy::isEmergencyStopped() const {
    return emergencyStopped;
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

void MicroDronInterfaceDummy::update(){
    size_t bufLen = MAVLINK_MAX_PACKET_LEN + sizeof(uint64_t);
    uint8_t buf[bufLen];

    while(isRunning){
        memset(buf, 0, bufLen);
        int len = conn->recv(buf, bufLen);

        if (len > 0) {
            mavlink_message_t msg;
            mavlink_status_t status;
            mavlink_attitude_t new_attitude;

            for (int i = 0; i < len; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    if(msg.msgid == MAVLINK_MSG_ID_ATTITUDE){
                        mavlink_msg_attitude_decode(&msg, &new_attitude);
                        attitude = new_attitude;
                    }
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

MicroDronInterfaceDummy::~MicroDronInterfaceDummy() {
    isRunning = false;
    conn->closeConnection();
    updateThread.join();
}
