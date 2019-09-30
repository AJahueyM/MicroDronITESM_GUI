//
// Created by alberto on 5/04/19.
//

#include <fcntl.h>
#include "MicroDronInterface.h"


MicroDronInterface::MicroDronInterface(const std::string& ipAddress, short int port) : ipAddress(ipAddress), port(port){
    updateThread = std::thread(&MicroDronInterface::updateComms, this);
}

void MicroDronInterface::updateComms() {
    sock = 0;

    if ((sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP)) < 0){
        printf("\n Socket creation error \n");
    }

    /* Save the current flags */
    int flags = fcntl(sock, F_GETFL, 0);

    flags |= O_NONBLOCK;
    fcntl(sock, F_SETFL, flags);


    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, ipAddress.c_str(), &serv_addr.sin_addr)<=0){
        printf("\nInvalid address/ Address not supported \n");
    }

    printf("Connecting...\n");

    int res = connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr));;

    while(res < 0 && isRunning){
        //printf("Connecting...\n");
        res = connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if(res > 0){
        printf("Connected\n");
    }

    while(isRunning){

        std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
        double secondsSinceValidMessage = std::chrono::duration<double>(currentTime - lastTimeUpdate).count();

        connected = secondsSinceValidMessage < 0.5;

        char bufferByte = 0;
        ssize_t valread = read( sock , &bufferByte, 1);

        if(bufferByte > 0){
            if(!validMessage)
                validMessage = bufferByte == 'Y';

            if(bufferByte != 'K' && validMessage){
                if(incomeBufferIndex + 1 < maxIncomeBufferSize) {
                    if(bufferByte != '\r' && bufferByte != '\n')
                        incomeBuffer[incomeBufferIndex++] = bufferByte;
                }else{
                    validMessage = false;
                }

            }else{
                if(validMessage){
                    lastTimeUpdate = std::chrono::high_resolution_clock::now();
                    int ret = sscanf(incomeBuffer, "Y:%f M:%d P:%f R:%f H:%f  M:%f %f %f %f PID: %f Y:%f %f %f P:%f %f %f R:%f %f %f %f",
                                     &yaw,&mode, &pitch, &roll, &height, &motorOutput1, &motorOutput2, &motorOutput3, &motorOutput4,
                                     &k, &yawPid.p, &yawPid.i, &yawPid.d, &pitchPid.p, &pitchPid.i, &pitchPid.d, &rollPid.p,
                                     &rollPid.i, &rollPid.d,  &droneTime);
                    validMessage = false;
                }
                memset(&incomeBuffer, '\0', sizeof(incomeBuffer));
                incomeBufferIndex = 0;
            }


        }

    }

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

float MicroDronInterface::getK() const {
    return k;
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
    int ret = sprintf(msg,  manualMotorControlTemplate.c_str(), output, output, output, output);

    send(sock, msg, strlen(msg), 0);
}


void MicroDronInterface::setAllMotorOutput(float output1, float output2, float output3, float output4) {
    if(!isConnected()){
        return;
    }

    char msg[50] = {'\0'};
    int ret = sprintf(msg, manualMotorControlTemplate.c_str(), output1, output2, output3, output4);

    send(sock, msg, strlen(msg), 0);
}

bool MicroDronInterface::isConnected() const {
    return connected;

    /**
     * The transmision from the drone broke due to wiring, this is a workaround to still be able to control it.
     * Effectively disables connection safety functions, should change to return 'connected' once issue is fixed.
     */
    //return true;
}

void MicroDronInterface::setSetpoints(float pitch, float roll, float yaw, float height) {
    if(!isConnected()){
        return;
    }

    char msg[50] = {'\0'};
    int ret = sprintf(msg,  setpointControlTemplate.c_str(), yaw, pitch, roll, height);

    send(sock, msg, strlen(msg), 0);
}

void MicroDronInterface::setK(float newK) {
    if(!isConnected()){
        return;
    }

    char msg[50] = {'\0'};
    int ret = sprintf(msg,  kUpdateTemplate.c_str(),newK);

    send(sock, msg, strlen(msg), 0);
}

void MicroDronInterface::emergencyStop() {
    if(!isConnected()){
        return;
    }

    send(sock, emergencyStopTemplate.c_str(), strlen(emergencyStopTemplate.c_str()), 0);
}

const SimplePID &MicroDronInterface::getPitchPid() const {
    return pitchPid;
}

const SimplePID &MicroDronInterface::getRollPid() const {
    return rollPid;
}

const SimplePID &MicroDronInterface::getYawPid() const {
    return yawPid;
}

const SimplePID &MicroDronInterface::getHeightPid() const {
    return heightPid;
}

void MicroDronInterface::updatePID(char pidCode, SimplePID pid){
    if(!isConnected()){
        return;
    }

    char msg[150] = {'\0'};
    int ret = sprintf(msg,  pidConfigUpdateTemplate.c_str(),
            pidCode, pid.p, pid.i, pid.d, pid.clamped ? 1.0 : -1.0, pid.maxOutput,
            pid.minOutput, pid.continuous  ? 1.0 : -1.0, pid.maxInput, pid.minInput);
    printf("%s", msg);
    send(sock, msg, strlen(msg), 0);

}
void MicroDronInterface::setPitchPid(SimplePID pitchPid){
    updatePID('P', pitchPid);
}

void MicroDronInterface::setRollPid(SimplePID rollPid){
    updatePID('R', rollPid);

}

void MicroDronInterface::setYawPid(SimplePID yawPid){
    updatePID('Y', yawPid);

}

void MicroDronInterface::setHeightPid(SimplePID heightPid){
    updatePID('H', heightPid);
}

void MicroDronInterface::sendHeartBeat(){
    if(!isConnected()){
        return;
    }

    send(sock, kHeartbeatTemplate.c_str(), strlen(kHeartbeatTemplate.c_str()), 0);
}


float MicroDronInterface::getHeartbeatTime() const {
    return droneTime;
}

MicroDronInterface::~MicroDronInterface() {
    isRunning = false;
    close(sock);
    if(updateThread.joinable())
        updateThread.join();
}