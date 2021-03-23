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
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread.hpp>

using boost::asio::ip::udp;
using boost::asio::ip::tcp;
using boost::asio::ip::address;

class tcp_connection : public boost::enable_shared_from_this<tcp_connection> {
public:
    typedef boost::shared_ptr<tcp_connection> pointer;

    static pointer create(boost::asio::io_context &io_context) {
        return pointer(new tcp_connection(io_context));
    }

    tcp::socket &socket() {
        return socket_;
    }

    void start(){
        boost::asio::async_read(socket_, boost::asio::buffer(recvBuf),
                                boost::bind(&tcp_connection::handle_read, shared_from_this(),
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
    }

    void queueWrite(const mavlink_message_t &msg){
        auto len = mavlink_msg_to_send_buffer(buf, &msg);
        boost::asio::async_write(socket_, boost::asio::buffer(buf, len),
                                 boost::bind(&tcp_connection::handle_write, shared_from_this(),
                                             boost::asio::placeholders::error,
                                             boost::asio::placeholders::bytes_transferred()));
    }

    std::queue<mavlink_message_t> recvQueue;

protected:
    void handle_write(const boost::system::error_code& /*error*/,
                      size_t /*bytes_transferred*/){
    }

    void handle_read(const boost::system::error_code& error, size_t len){
        if(error) return;

        mavlink_message_t msg;
        mavlink_status_t status;
        for(size_t i = 0; i < len; ++i){
            if(mavlink_parse_char(MAVLINK_COMM_1, recvBuf[i], &msg, &status)){
                recvQueue.emplace(msg);
                boost::asio::async_read(socket_, boost::asio::buffer(buf, len),
                                         boost::bind(&tcp_connection::handle_read, shared_from_this(),
                                                     boost::asio::placeholders::error,
                                                     boost::asio::placeholders::bytes_transferred()));
            }
        }
    }

private:
    tcp_connection(boost::asio::io_context& io_context)
            : socket_(io_context)
    {
    }

    tcp::socket socket_;

    uint8_t buf[MAVLINK_MAX_PACKET_LEN + 5];
    boost::array<uint8_t, MAVLINK_MAX_PACKET_LEN + 5> recvBuf;
};

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

    void handle_accept(tcp_connection::pointer new_conn, const boost::system::error_code& error);

private:
    boost::thread_group tg;

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

    tcp::acceptor tcpAcceptor;
    tcp_connection::pointer droneConn;
};


#endif //MICRODRONITESM_GUI_ESPCOMMS_H
