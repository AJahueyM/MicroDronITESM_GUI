//
// Created by abiel on 3/18/21.
//

#ifndef MICRODRONITESM_GUI_ESPCOMMS_H
#define MICRODRONITESM_GUI_ESPCOMMS_H

#include <iostream>
#include <cstring>
#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <optional>
#include "mavlink.h"
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>

using boost::asio::ip::udp;
using boost::asio::ip::address;

class ESPComms {
public:
    ESPComms(const std::string &serverIp, int udpListenPort, int udpSendPort, int tcpPort);

    /**
     * Messages with sysid 2 will be sent over TCP
     * @param msg
     */
    void sendMessage(const mavlink_message_t &msg);

    std::optional<mavlink_message_t> getMessage();

protected:
    void handle_receive(const boost::system::error_code &error, std::size_t bytes);
    void handle_send(const boost::system::error_code &error, std::size_t bytes);

private:
    std::thread io_service_thread;

    std::queue<mavlink_message_t> messageInQueue;
    std::mutex messageInQueueMutex;

    std::queue<mavlink_message_t> tcpOutQueue, udpOutQueue;
    std::mutex tcpOutMutex, udpOutMutex;

    static constexpr size_t bufferLen = 4096 * 10;
    boost::array<uint8_t, bufferLen> recv_buffer;

    udp::endpoint server_endpoint, remote_endpoint;

    uint8_t buf[bufferLen];

    boost::asio::io_service io_service;
    udp::socket udpClientSocket, udpServerSocket;
};


#endif //MICRODRONITESM_GUI_ESPCOMMS_H
