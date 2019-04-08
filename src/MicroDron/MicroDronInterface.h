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

class MicroDronInterface {
public:
    MicroDronInterface(const std::string& ipAddress, short int port);
    ~MicroDronInterface();

    float getPitch() const;

    float getRoll() const;

    float getYaw() const;

    float getHeight() const;

    int getMode() const;

    float getMotorOutput1() const;

    float getMotorOutput2() const;

    float getMotorOutput3() const;

    float getMotorOutput4() const;

    void setAllMotorOutput(float output);

    void setAllMotorOutput(float output1, float output2, float output3, float output4);

    void setSetpoints(float pitch, float roll, float yaw, float height);

    void emergencyStop();

    bool isConnected() const;

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

    char incomeBuffer[100] = {'\0'};
    const int maxIncomeBufferSize = 100;

    int incomeBufferIndex = 0;
    char startChar = 'Y';
    bool validMessage = false;

    float pitch = 0;
    float roll = 0;
    float yaw = 0;
    float height = 0;

    int mode = 0;
    float motorOutput1 = 0, motorOutput2 = 0, motorOutput3 = 0, motorOutput4 = 0;
};


#endif //MICRODRONITESM_GUI_MICRODRONINTERFACE_H
