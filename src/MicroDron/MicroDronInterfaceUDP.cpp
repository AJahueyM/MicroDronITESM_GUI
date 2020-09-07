//
// Created by abiel on 8/19/20.
//

#include "MicroDronInterfaceUDP.h"
#include <arpa/inet.h>
#include <cmath>
#include <chrono>
#include <string>
#include <stdexcept>
#include <iostream>
#include <fmt/format.h>

MicroDronInterfaceUDP::MicroDronInterfaceUDP() {
    mavlink_attitude_t initialAttitude;
    initialAttitude.yaw = 0;
    initialAttitude.pitch = 0;
    initialAttitude.roll = 0;
    attitude.store(initialAttitude);

    std::string addr("127.0.0.1");
    int ret = udp_conn_open_ip(&conn, addr.c_str(), 14551, 14550);

    if(ret < 0){
        perror("UDP conn open failed");
        udp_conn_close(&conn);
        throw std::runtime_error("UDP conn open failed");
    }

//    ret = fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC);
//    if(ret < 0){
//        perror("Nonblocking set failed");
//        close(sock);
//        exit(EXIT_FAILURE);
//    }
    lastHb = std::chrono::high_resolution_clock::now();
    updateThread = std::thread(&MicroDronInterfaceUDP::update, this);
}

void MicroDronInterfaceUDP::setPitchPid(SimplePID pitchPid) {

}

void MicroDronInterfaceUDP::setRollPid(SimplePID rollPid) {

}

void MicroDronInterfaceUDP::setYawPid(SimplePID yawPid) {

}

void MicroDronInterfaceUDP::setHeightPid(SimplePID heightPid) {

}

void MicroDronInterfaceUDP::sendHeartBeat() {

}

float MicroDronInterfaceUDP::getPitch() const {
    return attitude.load().pitch * 180.0 / M_PI;
}

float MicroDronInterfaceUDP::getRoll() const {
    return attitude.load().roll * 180.0 / M_PI;
}

float MicroDronInterfaceUDP::getYaw() const {
    return attitude.load().yaw * 180.0 / M_PI;
}

float MicroDronInterfaceUDP::getHeight() const {
    return distanceSensor.load().current_distance;
}

int MicroDronInterfaceUDP::getMode() const {
    return 0;
}

float MicroDronInterfaceUDP::getK() const {
    return 0;
}

float MicroDronInterfaceUDP::getMotorOutput1() const {
    return 0;
}

float MicroDronInterfaceUDP::getMotorOutput2() const {
    return 0;
}

float MicroDronInterfaceUDP::getMotorOutput3() const {
    return 0;
}

float MicroDronInterfaceUDP::getMotorOutput4() const {
    return 0;
}

void MicroDronInterfaceUDP::setAllMotorOutput(float output) {

}

void MicroDronInterfaceUDP::setAllMotorOutput(float output1, float output2, float output3, float output4) {

}

void MicroDronInterfaceUDP::setSetpoints(float roll, float pitch, float yaw, float height) {
    std::cout << fmt::format("{}, {}, {}, {}", roll, pitch, yaw, height) << std::endl;
}

void MicroDronInterfaceUDP::setK(float newK) {

}

void MicroDronInterfaceUDP::emergencyStop() {
    emergencyStopped = !emergencyStopped;
    if(emergencyStopped){
        std::cout << "Estopped" << std::endl;
    }
}

bool MicroDronInterfaceUDP::isEmergencyStopped() const {
    return emergencyStopped;
}

bool MicroDronInterfaceUDP::isConnected() const {
    return getHeartbeatTime() < 0.5;
}

float MicroDronInterfaceUDP::getHeartbeatTime() const {
    return std::min(1e3, std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - lastHb).count());
}

SimplePID MicroDronInterfaceUDP::getPitchPid() const {
    return SimplePID();
}

SimplePID MicroDronInterfaceUDP::getRollPid() const {
    return SimplePID();
}

SimplePID MicroDronInterfaceUDP::getYawPid() const {
    return SimplePID();
}

SimplePID MicroDronInterfaceUDP::getHeightPid() const {
    return SimplePID();
}

void MicroDronInterfaceUDP::update(){
    size_t bufLen = MAVLINK_MAX_PACKET_LEN + sizeof(uint64_t);
    uint8_t buf[bufLen];

    while(isRunning){
        memset(buf, 0, bufLen);
        int len = udp_conn_recv(&conn, buf, bufLen);

        if (len > 0) {
            mavlink_message_t msg;
            mavlink_status_t status;
            mavlink_attitude_t new_attitude;
            mavlink_distance_sensor_t new_distanceSensor;
            for (int i = 0; i < len; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_ATTITUDE:
                            mavlink_msg_attitude_decode(&msg, &new_attitude);
                            attitude = new_attitude;
                            break;
                        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
                            mavlink_msg_distance_sensor_decode(&msg, &new_distanceSensor);
                            distanceSensor = new_distanceSensor;
                            break;
                        case MAVLINK_MSG_ID_HEARTBEAT:
                            lastHb = std::chrono::high_resolution_clock::now();
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

MicroDronInterfaceUDP::~MicroDronInterfaceUDP() {
    isRunning = false;
    udp_conn_close(&conn);
    updateThread.join();
}
