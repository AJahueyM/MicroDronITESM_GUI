//
// Created by abiel on 8/19/20.
//

#ifndef MICRODRONITESM_GUI_UDPCONNECTION_H
#define MICRODRONITESM_GUI_UDPCONNECTION_H

#include <cstdint>
#include <netinet/in.h>
#include <string>

class UDPConnection {
public:
    UDPConnection(const std::string &droneIp, uint16_t sendPort, uint16_t recvPort);
    ~UDPConnection();

    void startConnection();

    int send(uint8_t const *buf, char len);
    int recv(uint8_t *buf, char bufLen);

    void closeConnection();

private:
    std::string droneIp;

    uint16_t targetSendPort;
    uint16_t targetRecvPort;

    sockaddr_in droneAddr{};
    sockaddr_in locAddr{};
    socklen_t fromLen{};

    int sock{};

    bool connectionStarted{false};
};


#endif //MICRODRONITESM_GUI_UDPCONNECTION_H
