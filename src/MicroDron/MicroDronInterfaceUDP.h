//
// Created by abiel on 8/19/20.
//

#ifndef MICRODRONITESM_GUI_MICRODRONINTERFACEUDP_H
#define MICRODRONITESM_GUI_MICRODRONINTERFACEUDP_H

#include "MicroDronInterface.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstdlib>
#include <fcntl.h>
#include <arpa/inet.h>
#include <cstring>
#include <thread>
#include "mavlink.h"
#include <atomic>
#include <map>
#include "ESPComms.h"

struct MotorValues{
    float frontLeft, frontRight;
    float backLeft, backRight;
};

extern "C"{
#include <UDP.h>
};

class MicroDronInterfaceUDP : public MicroDronInterface {
public:
    MicroDronInterfaceUDP();

    SimplePID getPitchPid() const override;

    SimplePID getRollPid() const override;

    SimplePID getYawPid() const override;

    SimplePID getHeightPid() const override;

    void setPitchPid(SimplePID pitchPid) override;

    void setRollPid(SimplePID rollPid) override;

    void setYawPid(SimplePID yawPid) override;

    void setHeightPid(SimplePID heightPid) override;

    void sendHeartBeat() override;

    std::chrono::high_resolution_clock::time_point getLastAttUpdateTime() const;

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

    void sendJoystickControl(int16_t x, int16_t y, int16_t z, int16_t r) override;

    void setK(float newK) override;

    void emergencyStop(bool state) override;

    bool isEmergencyStopped() const override;

    bool isConnected() const override;

    float getHeartbeatTime() const override;

    MotorValues getMotorValues() const;

    std::chrono::high_resolution_clock::time_point getLastMotorUpdate() const;

    void requestParamList();

    std::map<int, mavlink_param_value_t>& getParams();

    void setParameter(const mavlink_param_set_t &paramSet);

    ~MicroDronInterfaceUDP();
private:
    void update();
    void hbUpdate();

    void sendMessage(const mavlink_message_t &msg);

    void sendAndCheckParams();

    bool emergencyStopped{false};
    const uint16_t gs_port = 14550;
    struct sockaddr_in gs_server{}, gs_client{}; //Local IP addr
    std::atomic<mavlink_attitude_t> attitude{};
    std::atomic<mavlink_distance_sensor_t> distanceSensor{};

    std::chrono::high_resolution_clock::time_point lastHb;

    std::map<int, mavlink_param_value_t> paramList;

    std::thread updateThread, hbThread, paramsThread;
    std::mutex updateMutex, nextToAccessMutex, lowPriorityMutex;


    std::map<mavlink_param_set_t, mavlink_message_t> pendingParams {};

    bool isRunning = true;

    MotorValues motorValues;
    std::chrono::high_resolution_clock::time_point lastMotorUpdateTime = std::chrono::high_resolution_clock::now();

    std::chrono::high_resolution_clock::time_point lastAttUpdateTime = std::chrono::high_resolution_clock::now();

    ESPComms *comms;
};


#endif //MICRODRONITESM_GUI_MICRODRONINTERFACEUDP_H
