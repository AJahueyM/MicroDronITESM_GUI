//
// Created by abiel on 3/18/21.
//

#include "ESPComms.h"
#include <functional>

ESPComms::ESPComms(const std::string &serverIp, int udpListenPort, int udpSendPort, int tcpPort)
    : udpClientSocket(io_service), udpServerSocket(io_service), tcpAcceptor(io_service, tcp::endpoint(tcp::v4(), tcpPort)) {
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

    droneConn = tcp_connection::create(io_service);
    tcpAcceptor.async_accept(droneConn->socket(),
                             boost::bind(&ESPComms::handle_accept, this, droneConn, boost::asio::placeholders::error));


    for(unsigned int i = 0; i < boost::thread::hardware_concurrency(); ++i){
            tg.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
    }
}

void ESPComms::handle_receive(const boost::system::error_code &error, std::size_t bytes) {
    if(error) return;

    mavlink_message_t msg;
    mavlink_status_t status;
    for(size_t i = 0; i < bytes; ++i){
        if(mavlink_parse_char(MAVLINK_COMM_0, recv_buffer[i], &msg, &status)){
            messageInQueueMutex.lock();
            messageInQueue.emplace(msg);
            messageInQueueMutex.unlock();

            udpServerSocket.async_receive_from(
                    boost::asio::buffer(recv_buffer), server_endpoint,
                    boost::bind(&ESPComms::handle_receive, this,
                                boost::asio::placeholders::error,
                                boost::asio::placeholders::bytes_transferred
                    ));
        }
    }
}

void ESPComms::handle_send(const boost::system::error_code &error, std::size_t bytes) {
    if(error){
        std::cout << "help" << std::endl;
    }
}

void ESPComms::handle_accept(tcp_connection::pointer new_conn, const boost::system::error_code &error) {
    if(!error){
        new_conn->start();
    }
}

void ESPComms::sendMessage(const mavlink_message_t &msg) {
    if(msg.sysid == 2 and droneConn){
        droneConn->queueWrite(msg);
    } else {
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        auto boostBuf = std::make_shared<std::string>((char*) buf, len);
        udpServerSocket.async_send_to(boost::asio::buffer(*boostBuf), remote_endpoint,
                                      boost::bind(&ESPComms::handle_send, this,
                                                  boost::asio::placeholders::error,
                                                  boost::asio::placeholders::bytes_transferred));

    }
}

std::optional<mavlink_message_t> ESPComms::getMessage() {
    std::lock_guard<std::mutex> msgLock(messageInQueueMutex);

    std::optional<mavlink_message_t> ret{};
    if(droneConn and !droneConn->recvQueue.empty()){
        ret = droneConn->recvQueue.front();
        droneConn->recvQueue.pop();
    } else if(!messageInQueue.empty()) {
        ret = messageInQueue.front();
        messageInQueue.pop();
    }

    return ret;
}