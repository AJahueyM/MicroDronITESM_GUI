//
// Created by abiel on 8/19/20.
//

#include "UDPConnection.h"
#include <stdexcept>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>

UDPConnection::UDPConnection(const std::string &droneIp, uint16_t port) : UDPConnection(droneIp, port, port) {
    ;
}

UDPConnection::UDPConnection(const std::string &droneIp, uint16_t rxPort, uint16_t txPort) {
    this->droneIp = droneIp;
    this->rxPort = rxPort;
    this->txPort = txPort;
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
    locAddr.sin_port = htons(rxPort);

    int ret = bind(sock, (struct sockaddr *) &locAddr, sizeof(struct sockaddr));
    if(ret < 0){
        perror("UDP bind failed");
        close(sock);
        throw std::runtime_error("UDP bind failed");
    }

    ret = fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC);
    if(ret < 0){
        perror("Nonblocking set failed");
        close(sock);
        throw std::runtime_error("Nonblocking set failed");
    }

    memset(reinterpret_cast<wchar_t *>(&droneAddr), 0, sizeof(locAddr));
    droneAddr.sin_family = AF_INET;
    droneAddr.sin_addr.s_addr = inet_addr(droneIp.c_str());
    droneAddr.sin_port = htons(txPort);
}

int UDPConnection::send(const uint8_t *buf, char len) {
    return sendto(sock, buf, len, 0, (struct sockaddr*) &droneAddr, sizeof(struct sockaddr_in));
}

int UDPConnection::recv(uint8_t *buf, char bufLen) {
    return recvfrom(sock, (void*) buf, bufLen, 0, nullptr, nullptr);
}

void UDPConnection::closeConnection() {
    close(sock);
}

UDPConnection::~UDPConnection() {
    closeConnection();
}