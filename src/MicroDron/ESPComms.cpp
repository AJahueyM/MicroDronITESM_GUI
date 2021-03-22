//
// Created by abiel on 3/18/21.
//

#include "ESPComms.h"
#include <functional>

ESPComms::ESPComms(const std::string &serverIp, int udpListenPort, int udpSendPort, int tcpPort)
    : udpClientSocket(io_service), udpServerSocket(io_service) {
    remote_endpoint = udp::endpoint(address::from_string(serverIp), udpSendPort);
    udpClientSocket.open(udp::v4());

    udpServerSocket.open(udp::v4());
    udpServerSocket.bind(udp::endpoint(udp::v4(), udpListenPort));
    udpServerSocket.async_receive_from(
            boost::asio::buffer(recv_buffer), server_endpoint,
            boost::bind(&ESPComms::handle_receive, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred
            ));

    io_service_thread = std::thread([&]{ io_service.run(); });
}

void ESPComms::handle_receive(const boost::system::error_code &error, std::size_t bytes) {
    mavlink_message_t msg;
    mavlink_status_t status;
    for(size_t i = 0; i < bytes; ++i){
        if(mavlink_parse_char(MAVLINK_COMM_0, recv_buffer[i], &msg, &status)){
            messageInQueueMutex.lock();
            messageInQueue.emplace(msg);
            messageInQueueMutex.unlock();
        }
    }

    udpServerSocket.async_receive_from(
            boost::asio::buffer(recv_buffer), server_endpoint,
            boost::bind(&ESPComms::handle_receive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred
            ));
}

void ESPComms::handle_send(const boost::system::error_code &error, std::size_t bytes) {
    ;
}

void ESPComms::sendMessage(const mavlink_message_t &msg) {
    int len = mavlink_msg_to_send_buffer(buf, &msg);

    if(msg.sysid == 2){
        tcpOutMutex.lock();
        tcpOutQueue.emplace(msg);
        tcpOutMutex.unlock();
    } else {
        udpServerSocket.async_send_to(boost::asio::buffer(buf, len), remote_endpoint,
                                      boost::bind(&ESPComms::handle_send, this,
                                                  boost::asio::placeholders::error,
                                                  boost::asio::placeholders::bytes_transferred));
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