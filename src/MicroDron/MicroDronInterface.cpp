//
// Created by alberto on 5/04/19.
//

#include "MicroDronInterface.h"


MicroDronInterface::MicroDronInterface(const std::string& ipAddress, short int port) : ipAddress(ipAddress), port(port){

    updateThread = std::thread(&MicroDronInterface::updateComms, this);
}

void MicroDronInterface::updateComms() {
    sock = 0;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        printf("\n Socket creation error \n");
    }

    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, ipAddress.c_str(), &serv_addr.sin_addr)<=0){
        printf("\nInvalid address/ Address not supported \n");
    }

    connected = connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0;
    if (connected){
        printf("\nConnection Failed \n");
    }


    while(isRunning){

        char bufferByte = 0;

        ssize_t valread = read( sock , &bufferByte, 1);

        if(!validMessage)
            validMessage = bufferByte == 'Y';

        if(bufferByte != 'K' && validMessage){
            if(bufferByte != '\r' && incomeBufferIndex + 1 < maxIncomeBufferSize) {
                incomeBuffer[incomeBufferIndex++] = bufferByte;
            }else{
                validMessage = false;
            }

        }else{
            if(validMessage){
                int ret = sscanf(incomeBuffer, "Y:%f M:%d P:%f R:%f H:%f  M:%f %f %f %f", &yaw,&mode, &pitch, &roll, &height, &motorOutput1, &motorOutput2, &motorOutput3, &motorOutput4);
                validMessage = false;
            }
            memset(&incomeBuffer, '\0', sizeof(incomeBuffer));
            incomeBufferIndex = 0;
        }

    }

}

MicroDronInterface::~MicroDronInterface() {
    isRunning = false;
    if(updateThread.joinable())
        updateThread.join();
}

float MicroDronInterface::getPitch() const {
    return pitch;
}

float MicroDronInterface::getRoll() const {
    return roll;
}

float MicroDronInterface::getYaw() const {
    return yaw;
}

float MicroDronInterface::getHeight() const {
    return height;
}

int MicroDronInterface::getMode() const {
    return mode;
}

float MicroDronInterface::getMotorOutput1() const {
    return motorOutput1;
}

float MicroDronInterface::getMotorOutput2() const {
    return motorOutput2;
}

float MicroDronInterface::getMotorOutput3() const {
    return motorOutput3;
}

float MicroDronInterface::getMotorOutput4() const {
    return motorOutput4;
}

void MicroDronInterface::setAllMotorOutput(float output) {
    if(!isConnected()){
        return;
    }

    char msg[50] = {'\0'};
    int ret = sprintf(msg, ",M %f %f %f %f", output, output, output, output);

    send(sock, msg, strlen(msg), 0);
}


void MicroDronInterface::setAllMotorOutput(float output1, float output2, float output3, float output4) {
    if(!isConnected()){
        return;
    }

    char msg[50] = {'\0'};
    int ret = sprintf(msg, ",M %f %f %f %f", output1, output2, output3, output4);

    send(sock, msg, strlen(msg), 0);
}

bool MicroDronInterface::isConnected() const {
    return connected;
}

void MicroDronInterface::setSetpoints(float pitch, float roll, float yaw, float height) {
    if(!isConnected()){
        return;
    }

    char msg[50] = {'\0'};
    int ret = sprintf(msg, ",M %f %f %f %f", yaw, pitch, roll, height);

    send(sock, msg, strlen(msg), 0);
}

void MicroDronInterface::emergencyStop() {
    if(!isConnected()){
        return;
    }

    char msg[50] = {'\0'};
    int ret = sprintf(msg, ",K");

    send(sock, msg, strlen(msg), 0);
}
