//
// Created by alberto on 5/04/19.
//

#ifndef MICRODRONITESM_GUI_MICRODRONINTERFACE_H
#define MICRODRONITESM_GUI_MICRODRONINTERFACE_H
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <thread>
#include <chrono>


struct SimplePID{
    float p = 0;
    float i = 0;
    float d = 0;
};


class MicroDronInterface {
public:
    MicroDronInterface(const std::string& ipAddress, short int port);
    ~MicroDronInterface();

    const SimplePID &getPitchPid() const;

    const SimplePID &getRollPid() const;

    const SimplePID &getYawPid() const;

    const SimplePID &getHeightPid() const;

    void setPitchPid(SimplePID pitchPid);

    void setRollPid(SimplePID rollPid);

    void setYawPid(SimplePID yawPid);

    void setHeightPid(SimplePID heightPid);

    void sendHeartBeat();

    float getPitch() const;

    float getRoll() const;

    float getYaw() const;

    float getHeight() const;

    int getMode() const;

    float getK() const;

    float getMotorOutput1() const;

    float getMotorOutput2() const;

    float getMotorOutput3() const;

    float getMotorOutput4() const;

    void setAllMotorOutput(float output);

    void setAllMotorOutput(float output1, float output2, float output3, float output4);

    void setSetpoints(float pitch, float roll, float yaw, float height);

    void setK(float newK);

    void emergencyStop();

    bool isConnected() const;

    float getHeartbeatTime() const;

private:
    void updateComms();

    std::thread updateThread;
    bool isRunning = true;

    std::string ipAddress;
    short int port;
    int sock;
    struct sockaddr_in address;
    struct sockaddr_in serv_addr;
    bool connected = false;

    char incomeBuffer[200] = {'\0'};
    const int maxIncomeBufferSize = 200;

    int incomeBufferIndex = 0;
    char startChar = 'Y';
    std::string manualMotorControlTemplate = ",M %f %f %f %f\n";
    std::string setpointControlTemplate = ",S %f %f %f %f\n";
    std::string emergencyStopTemplate = ",E\n";
    std::string pidConfigUpdateTemplate = ",P %c %f %f %f\n";
    std::string kUpdateTemplate = ",K %f\n";
    std::string kHeartbeatTemplate = ",L\n";

    bool validMessage = false;

    float pitch = 0;
    float roll = 0;
    float yaw = 0;
    float height = 0;

    int mode = 0;
    float motorOutput1 = 0, motorOutput2 = 0, motorOutput3 = 0, motorOutput4 = 0;
    float droneTime = 0.0f;

    float k = 0;
    SimplePID pitchPid, rollPid, yawPid, heightPid;

    std::chrono::high_resolution_clock::time_point lastTimeUpdate;
};


#endif //MICRODRONITESM_GUI_MICRODRONINTERFACE_H
