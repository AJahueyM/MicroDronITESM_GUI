//
// Created by abiel on 8/19/20.
//

#include "UDPConnection.h"
#include <stdexcept>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <libnet.h>

UDPConnection::UDPConnection(const std::string &droneIp, uint16_t sendPort, uint16_t recvPort) {
    this->droneIp = droneIp;
    this->targetSendPort = sendPort;
    this->targetRecvPort = recvPort;
}

void UDPConnection::startConnection() {
    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sock < 0){
        perror("Socket open failed");
        throw std::runtime_error("Socket open failed");
    }

    memset(reinterpret_cast<wchar_t *>(&locAddr), 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(targetSendPort);

    int ret = bind(sock, (struct sockaddr *) &locAddr, sizeof(struct sockaddr));
    if(ret < 0){
        perror("UDP bind failed");
        close(sock);
        throw std::runtime_error("UDP bind failed");
    }

    ret = fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC);
    if(ret < 0){
        perror("Nonblocking set failed");
        close(sock);
        throw std::runtime_error("Nonblocking set failed");
    }

    memset(reinterpret_cast<wchar_t *>(&droneAddr), 0, sizeof(locAddr));
    droneAddr.sin_family = AF_INET;
    droneAddr.sin_addr.s_addr = inet_addr(droneIp.c_str());
    droneAddr.sin_port = htons(targetRecvPort);
    fromLen = sizeof(droneAddr);
}

int UDPConnection::send(const uint8_t *buf, char len) {
    return sendto(sock, buf, len, 0, (struct sockaddr*) &droneAddr, sizeof(struct sockaddr_in));
}

int UDPConnection::recv(uint8_t *buf, char bufLen) {
    return recvfrom(sock, (void*) buf, bufLen, 0, (struct sockaddr *) &droneAddr, &fromLen);
}

void UDPConnection::closeConnection() {
    close(sock);
}

UDPConnection::~UDPConnection() {
    closeConnection();
}