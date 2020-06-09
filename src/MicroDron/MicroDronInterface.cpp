//
// Created by alberto on 5/04/19.
//

#include <fcntl.h>
#include "MicroDronInterface.h"
#include <iostream>
#include <utility>

MicroDronInterface::MicroDronInterface(std::string  ipAddress, int port) : ipAddress(std::move(ipAddress)), port(port){
    updateThread = std::thread(&MicroDronInterface::updateComms, this);
}

void MicroDronInterface::updateComms() {

    connectionState = ConectionState::SettingUp;

    while(isRunning){
        connected = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - lastTimeUpdate).count() < 1.5;

        switch(connectionState){
            case ConectionState::SettingUp:{
                sock = 0;

                if ((sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP)) < 0){
                    std::cerr << "Socket creation error " << errno << " " << strerror(errno) << "\n";
                }

                /* Save the current flags */
                int flags = fcntl(sock, F_GETFL, 0);
                fcntl(sock, F_SETFL, flags);


                bzero((char *) &serv_addr, sizeof(serv_addr));

                serv_addr.sin_family = AF_INET;
                serv_addr.sin_port = htons(port);

                // Convert IPv4 and IPv6 addresses from text to binary form
                if(inet_pton(AF_INET, ipAddress.c_str(), &serv_addr.sin_addr)<=0){
                    std::cerr << "Invalid address/ Address not supported \n";
                }

                std::cout << "Connecting...\n";
                connectionState = ConectionState::Connecting;
                break;
            }
            case ConectionState::Connecting: {
                int res = connect(sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
                if(res < 0){
                    std::cerr << "Could not connect to socket... Error id: " << errno << " " << strerror(errno)<< ". Retrying\n";
                }else{
                    connectionState = ConectionState::Connected;
                    std::cout << "Connected" << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                break;
            }
            case ConectionState::Connected: {
                char buffer[BUFFER_SIZE];
                bzero(buffer, sizeof(buffer) - 1);
                int res = read(sock, buffer, sizeof(buffer) -1);

                if(res == 0 && errno == ECONNREFUSED){
                    /**
                     * If no bytes were read and connection was refused, maybe the server disconnected. Try reconnecting
                     */
                    std::cerr << "Server may have disconnected, trying to reconnect\n";
                    connectionState = ConectionState::SettingUp;
                    close(sock);
                    break;
                }else if(res < 0){
                    std::cerr << "An error occurred while trying to read from the socket... Error id: " << errno << " " << strerror(errno) << "\n";
                }

                update(buffer);
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

void MicroDronInterface::update(const char buffer[BUFFER_SIZE]) {
    std::string received = lastMessage + std::string(buffer);
    int messageStart = received.find('Y');
    int messageEnd = received.find('K');

    if((messageStart != std::string::npos && messageEnd != std::string::npos) && (messageEnd > messageStart)){
        if(messageEnd + 1 < received.length()){
            lastMessage = received.substr(messageEnd + 1);
        }else{
            lastMessage = "";
        }

        std::string message = received.substr(messageStart, messageEnd);

        int ret = sscanf(message.c_str(), "Y:%f M:%d P:%f R:%f H:%f  M:%f %f %f %f PID: %f Y:%f %f %f P:%f %f %f R:%f %f %f %f",
                         &yaw,&mode, &pitch, &roll, &height, &motorOutput1, &motorOutput2, &motorOutput3, &motorOutput4,
                         &k, &yawPid.p, &yawPid.i, &yawPid.d, &pitchPid.p, &pitchPid.i, &pitchPid.d, &rollPid.p,
                         &rollPid.i, &rollPid.d,  &droneTime);

        if(ret == 20){
            lastTimeUpdate = std::chrono::high_resolution_clock::now();
        }
    }
    std::lock_guard<std::mutex> lock(commandMutex);
    if(heartbeatReset){
        int res = write(sock, kHeartbeatTemplate.c_str(), strlen(kHeartbeatTemplate.c_str()));
        if(res < 0){
            std::cerr << "An error occurred while sending a message... Error id: " << errno << " " << strerror(errno) << "\n";
        }
        heartbeatReset = false;
    }
    if(needToSendCommand){
        write(sock, commandToSend.c_str(), strlen(commandToSend.c_str()));
        needToSendCommand = false;
    }
}

float MicroDronInterface::getPitch() const{
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
    std::lock_guard<std::mutex> lock(commandMutex);
    char msg[50] = {'\0'};
    int ret = sprintf(msg,  manualMotorControlTemplate.c_str(), output, output, output, output);

    commandToSend = std::string(msg);
    needToSendCommand = true;
}


void MicroDronInterface::setAllMotorOutput(float output1, float output2, float output3, float output4) {
    std::lock_guard<std::mutex> lock(commandMutex);
    char msg[50] = {'\0'};
    int ret = sprintf(msg, manualMotorControlTemplate.c_str(), output1, output2, output3, output4);

    commandToSend = std::string(msg);
    needToSendCommand = true;
}

bool MicroDronInterface::isConnected() const {
    return connected;
}

void MicroDronInterface::setSetpoints(float pitch, float roll, float yaw, float height) {
    std::lock_guard<std::mutex> lock(commandMutex);

    char msg[50] = {'\0'};
    int ret = sprintf(msg,  setpointControlTemplate.c_str(), yaw, pitch, roll, height);

    commandToSend = std::string(msg);
    needToSendCommand = true;
}

void MicroDronInterface::setK(float newK) {
    std::lock_guard<std::mutex> lock(commandMutex);

    char msg[50] = {'\0'};
    sprintf(msg,  kUpdateTemplate.c_str(),newK);
    commandToSend = std::string(msg);
    needToSendCommand = true;
}

void MicroDronInterface::emergencyStop() {
    std::lock_guard<std::mutex> lock(commandMutex);
    commandToSend = emergencyStopTemplate;
    needToSendCommand = true;
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
    std::lock_guard<std::mutex> lock(commandMutex);

    char msg[150] = {'\0'};
    sprintf(msg,  pidConfigUpdateTemplate.c_str(),
                      pidCode, pid.p, pid.i, pid.d, pid.clamped ? 1.0 : -1.0, pid.maxOutput,
                      pid.minOutput, pid.continuous  ? 1.0 : -1.0, pid.maxInput, pid.minInput);
    commandToSend = std::string(msg);
    needToSendCommand = true;

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
    std::lock_guard<std::mutex> lock(commandMutex);
    heartbeatReset = true;
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

