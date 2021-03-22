//
// Created by abiel on 3/18/21.
//

#include "ESPComms.h"
#include <functional>
#include <fcntl.h>
#include <signal.h>
#include <arpa/inet.h>

static bool has_sigpipe = false;

static void sigpipe_handler(int unused){
    std::cout << "sigpipe" << std::endl;
    has_sigpipe = true;
}

ESPComms::ESPComms(const std::string &serverIp, int udpListenPort, int udpSendPort, int tcpPort) {
    udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    sigaction(SIGPIPE, (struct sigaction*)&sigpipe_handler, NULL);

    this->tcpPort = tcpPort;
    this->serverIp = serverIp;
    connectTCP(serverIp, tcpPort);

    /**
     * UDP Socket creation
     */
    memset(&udpRecvAddr, 0, sizeof(udpRecvAddr));
    memset(&udpSendAddr, 0, sizeof(udpSendAddr));

    udpRecvAddr.sin_family = AF_INET;
    udpRecvAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    udpRecvAddr.sin_port = htons(udpListenPort);
    int ret = bind(udpSocket, (struct sockaddr *) &udpRecvAddr, sizeof(struct sockaddr));
    if(ret < 0){
        throw std::runtime_error("UDP Bind failed");
    }

    fromLen = sizeof(udpRecvAddr);

    udpSendAddr.sin_family = AF_INET;
    auto hp = gethostbyname(serverIp.c_str());
    if(!hp){
        throw std::runtime_error("Cannot obtain address of host");
    }

    memcpy(&udpSendAddr.sin_addr, hp->h_addr_list[0], hp->h_length);
    udpSendAddr.sin_port = htons(udpSendPort);

    /**
     * Set sock nonblock
     */
    fcntl(udpSocket, F_SETFL, O_NONBLOCK | O_ASYNC);
    fcntl(tcpSocket, F_SETFL, O_NONBLOCK | O_ASYNC);

    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 10;
    setsockopt(udpSocket, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));
    setsockopt(tcpSocket, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

    /**
     * Read epfd
     */
    read_epfd = epoll_create(2);
    struct epoll_event ev_read, ev_write;

    if (read_epfd < 0) {
        throw std::runtime_error("Error creating epoll");
    }

    ev_read.data.fd = udpSocket;
    ev_read.events = EPOLLIN;

    if (epoll_ctl(read_epfd, EPOLL_CTL_ADD, udpSocket, &ev_read) == -1) {
        throw std::runtime_error("Error at epoll_ctl()");
    }

    ev_read.data.fd = tcpSocket;
    ev_read.events = EPOLLIN;

    if (epoll_ctl(read_epfd, EPOLL_CTL_ADD, tcpSocket, &ev_read) == -1) {
        throw std::runtime_error("Error at epoll_ctl()");
    }

    /**
     * Write epfd
     */
    write_epfd = epoll_create(2);

    if (write_epfd < 0) {
        throw std::runtime_error("Error creating epoll");
    }

    ev_write.data.fd = tcpSocket;
    ev_write.events = EPOLLOUT;

    if (epoll_ctl(write_epfd, EPOLL_CTL_ADD, tcpSocket, &ev_write) == -1) {
        throw std::runtime_error("Error at epoll_ctl()");
    }

    ev_write.data.fd = udpSocket;
    ev_write.events = EPOLLOUT;

    if (epoll_ctl(write_epfd, EPOLL_CTL_ADD, udpSocket, &ev_write) == -1) {
        throw std::runtime_error("Error at epoll_ctl()");
    }

    readThread = std::thread(&ESPComms::readTask, this);
    writeThread = std::thread(&ESPComms::writeTask, this);
}

void ESPComms::connectTCP(const std::string &serverIp, int tcpPort) {
    /**
     * TCP Socket creation
     */
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = 0;
    hints.ai_protocol = IPPROTO_TCP;

    struct addrinfo *result;
    getaddrinfo(serverIp.c_str(), std::to_string(tcpPort).c_str(), &hints, &result);

    for(struct addrinfo *addr = result; addr != nullptr; addr = addr->ai_next){
        tcpSocket = socket(AF_INET, SOCK_STREAM, 0);

        if(connect(tcpSocket, addr->ai_addr, addr->ai_addrlen) == 0)
            break;

        close(tcpSocket);
        tcpSocket = -1;
    }

    if(tcpSocket == -1){
        throw std::runtime_error("Failed to connect to TCP");
    }
}

void ESPComms::readTask() {
    struct epoll_event read_evlist[5];

    for (;;) {
        int ret = epoll_wait(read_epfd, read_evlist, sizeof(read_evlist), -1);
        if (ret <= 0) continue;
        for (const auto &event : read_evlist) {
            int sock = event.data.fd;

            ssize_t len = 0;

            if(sock == udpSocket){
                len = recvfrom(udpSocket, (void*) recvBuffer, sizeof(recvBuffer), 0, (struct sockaddr *) &udpRecvAddr,
                               reinterpret_cast<socklen_t *>(&fromLen));
            } else if(sock == tcpSocket){
                len = read(tcpSocket, recvBuffer, sizeof(recvBuffer));
            }

            for (ssize_t i = 0; i < len; ++i) {
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
        int ret = epoll_wait(write_epfd, write_evlist, sizeof(write_evlist), -1);
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
            if(len > 0){
                if(sock == udpSocket){
                    sendto(sock, sendBuffer, len, 0, (struct sockaddr *) &udpSendAddr, sizeof(struct sockaddr_in));
                } else if(sock == tcpSocket){
                    if(has_sigpipe){
                        connectTCP(serverIp, tcpPort);
                        has_sigpipe = false;
                    }

                    std::string test = "Test";
                    write(sock, test.c_str(), test.length());
                }
            }
        }
    }
}

void ESPComms::sendMessage(const mavlink_message_t &msg) {
    if(msg.sysid == 2 or true){
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