//
// Created by abiel on 3/18/21.
//

#ifndef MICRODRONITESM_GUI_ESPCOMMS_H
#define MICRODRONITESM_GUI_ESPCOMMS_H

#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <netdb.h>
#include <stdexcept>
#include <cstring>
#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <optional>
#include "mavlink.h"

class ESPComms {
public:
    ESPComms(const std::string &serverIp, int udpListenPort, int udpSendPort, int tcpPort);

    /**
     * Messages with sysid 2 will be sent over TCP
     * @param msg
     */
    void sendMessage(const mavlink_message_t &msg);

    std::optional<mavlink_message_t> getMessage();
private:
    [[noreturn]] void readTask();

    [[noreturn]] void writeTask();

    std::thread readThread, writeThread;

    std::queue<mavlink_message_t> messageInQueue;
    std::mutex messageInQueueMutex;

    std::queue<mavlink_message_t> tcpOutQueue, udpOutQueue;
    std::mutex tcpOutMutex, udpOutMutex;

    int udpSocket, tcpSocket;
    int read_epfd, write_epfd;

    size_t fromLen;
    struct sockaddr_in udpSendAddr, udpRecvAddr;

    static constexpr size_t bufferLen = 4096 * 10;
    uint8_t recvBuffer[bufferLen];
    uint8_t sendBuffer[bufferLen];

    mavlink_message_t udpMsgIn, tcpMsgIn;
    mavlink_status_t udpMsgStatus, tcpMsgStatus;
};


#endif //MICRODRONITESM_GUI_ESPCOMMS_H
