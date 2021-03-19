//
// Created by abiel on 3/18/21.
//

#include "ESPComms.h"
#include <functional>

ESPComms::ESPComms(const std::string &serverIp, int udpListenPort, int udpSendPort, int tcpPort) {
    tcpSocket = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    struct hostent *server = nullptr;
    server = gethostbyname(serverIp.c_str());

    if (server == nullptr) {
        throw std::runtime_error("Invalid host");
    }

    /**
     * TCP Socket creation
     */
    struct sockaddr_in tcpAddr;
    memset(&tcpAddr, 0, sizeof(tcpAddr));
    memcpy(&tcpAddr.sin_addr, server->h_addr_list[0], server->h_length);
    tcpAddr.sin_family = AF_INET;
    tcpAddr.sin_port = htons(tcpPort);

    int ret = connect(tcpSocket, (struct sockaddr *) &tcpAddr, sizeof(tcpAddr));
//    if (ret < 0) {
//        throw std::runtime_error(std::string("TCP Connection failed! " + std::to_string(errno)));
//    }

    /**
     * UDP Socket creation
     */
    memset(&udpSendAddr, 0, sizeof(udpSendAddr));
    udpRecvAddr.sin_family = AF_INET;
    udpRecvAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    udpRecvAddr.sin_port = htons(udpListenPort);
    ret = bind(udpSocket, (struct sockaddr *) &udpRecvAddr, sizeof(struct sockaddr));
    if(ret < 0){
        throw std::runtime_error("UDP Bind failed");
    }

    fromLen = sizeof(udpRecvAddr);

    /**
     * Read epfd
     */
    read_epfd = epoll_create(2);
    struct epoll_event ev;

    if (read_epfd < 0) {
        throw std::runtime_error("Error creating epoll");
    }

    ev.data.fd = udpSocket;
    ev.events = EPOLLIN;

    if (epoll_ctl(read_epfd, EPOLL_CTL_ADD, udpSocket, &ev) == -1) {
        throw std::runtime_error("Error at epoll_ctl()");
    }

//    ev.data.fd = udpSocket;
//
//    if (epoll_ctl(read_epfd, EPOLL_CTL_ADD, udpSocket, &ev) == -1) {
//        throw std::runtime_error("Error at epoll_ctl()");
//    }

    /**
     * Write epfd
     */
    write_epfd = epoll_create(2);

    if (write_epfd < 0) {
        throw std::runtime_error("Error creating epoll");
    }

    ev.data.fd = tcpSocket;
    ev.events = EPOLLOUT;

    if (epoll_ctl(write_epfd, EPOLL_CTL_ADD, tcpSocket, &ev) == -1) {
        throw std::runtime_error("Error at epoll_ctl()");
    }

    ev.data.fd = udpSocket;

    if (epoll_ctl(write_epfd, EPOLL_CTL_ADD, udpSocket, &ev) == -1) {
        throw std::runtime_error("Error at epoll_ctl()");
    }

    readThread = std::thread(&ESPComms::readTask, this);
    writeThread = std::thread(&ESPComms::writeTask, this);
}

void ESPComms::readTask() {
    struct epoll_event read_evlist[5];

    for (;;) {
        int ret = epoll_wait(read_epfd, read_evlist, sizeof(read_evlist), -1);
        if (ret <= 0) continue;
        for (const auto &event : read_evlist) {
            int sock = event.data.fd;

            size_t len = 0;

            if(sock == udpSocket){
                len = recvfrom(udpSocket, (void*) recvBuffer, sizeof(recvBuffer), 0, (struct sockaddr *) &udpRecvAddr,
                               reinterpret_cast<socklen_t *>(&fromLen));
            } //TODO if TCPSocket

            for (size_t i = 0; i < len; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, recvBuffer[i], &udpMsgIn, &udpMsgStatus)) {
                    messageInQueueMutex.lock();
                    messageInQueue.emplace(udpMsgIn);
                    messageInQueueMutex.unlock();
                }

                if (mavlink_parse_char(MAVLINK_COMM_1, recvBuffer[i], &tcpMsgIn, &tcpMsgStatus)) {
                    messageInQueueMutex.lock();
                    messageInQueue.emplace(tcpMsgIn);
                    messageInQueueMutex.unlock();
                }
            }

        }
    }
}

void ESPComms::writeTask() {
    struct epoll_event write_evlist[5];

    for (;;) {
        int ret = epoll_wait(read_epfd, write_evlist, sizeof(write_evlist), -1);
        if(ret <= 0) continue;

        for (const auto &event : write_evlist) {
            int sock = event.data.fd;
            mavlink_message_t msg;

            if(sock == tcpSocket and !tcpOutQueue.empty()){
                //Ready to send to tcp
                tcpOutMutex.lock();
                msg = tcpOutQueue.front();
                tcpOutQueue.pop();
                tcpOutMutex.unlock();
            } else if(sock == udpSocket and !udpOutQueue.empty()){
                //Ready to send to udp
                udpOutMutex.lock();
                msg = udpOutQueue.front();
                udpOutQueue.pop();
                udpOutMutex.unlock();
            } else {
                continue;
            }

            auto len = mavlink_msg_to_send_buffer(sendBuffer, &msg);
            write(sock, sendBuffer, len);
        }
    }
}

void ESPComms::sendMessage(const mavlink_message_t &msg) {
    if(msg.sysid == 2){
        tcpOutMutex.lock();
        tcpOutQueue.emplace(msg);
        tcpOutMutex.unlock();
    } else {
        udpOutMutex.lock();
        udpOutQueue.emplace(msg);
        udpOutMutex.unlock();
    }
}

std::optional<mavlink_message_t> ESPComms::getMessage() {
    std::lock_guard<std::mutex> msgLock(messageInQueueMutex);

    std::optional<mavlink_message_t> ret{};
    if(!messageInQueue.empty()){
        ret = messageInQueue.front();
        messageInQueue.pop();
    }

    return ret;
}