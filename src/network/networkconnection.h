#ifndef NETWORKCONNECTION_H
#define NETWORKCONNECTION_H

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/system/error_code.hpp>
//#include <boost/endian/conversion.hpp>
#include <netinet/in.h>
#include <boost/bind.hpp>

#include <iostream>

#include "utils/event.h"

using namespace boost::asio::ip;
using namespace boost::asio;
using namespace boost::system;

using namespace std;



class NetworkConnection
{
public:
    Event<NetworkConnection, char*, unsigned int> messageReceivedEvent;
    Event<NetworkConnection, NetworkConnection*> connectionClosedEvent;


private:    
    tcp::socket* tcpSocket;             //Component
    udp::socket* udpSocket;             //shared (ressource)

    tcp::endpoint* tcpRemoteEndpoint;   //Component
    udp::endpoint* udpRemoteEndpoint;   //Component


public:
    NetworkConnection(tcp::socket* tcpSocket, tcp::endpoint* tcpRemoteEndpoint, udp::socket* udpSocket, udp::endpoint* udpRemoteEndpoint);
    ~NetworkConnection();

    void start();
    void restartUDP();

    void sendMessageTCP(char* messageData, unsigned int messageLength);
    void sendMessageUDP(char* messageData, unsigned int messageLength);


private:
    void asyncReadLengthTCP();
    void asyncReadContentTCP(unsigned int length);

    void asyncReadLengthUDP();
    void asyncReadContentUDP(unsigned int length);

    void on_tcpSocket_lengthReceived(char* receivedLengthData, size_t numberOfReceivedBytes, unsigned int numberOfReceivedBytesAspected, const boost::system::error_code error);
    void on_tcpSocket_contentReceived(char* receivedContentData, size_t numberOfReceivedBytes, unsigned int numberOfReceivedBytesAspected, const boost::system::error_code error);

    void on_udpSocket_lengthReceived(boost::asio::mutable_buffer* receivedLengthData, size_t numberOfReceivedBytes, unsigned int numberOfReceivedBytesAspected, const boost::system::error_code error);
    void on_udpSocket_contentReceived(char* receivedContentData, size_t numberOfReceivedBytes, unsigned int numberOfReceivedBytesAspected, const boost::system::error_code error);

    void on_tcpSocket_dataSent(char* outputBuffer, size_t numberOfSentBytes, unsigned int numberOfSentBytesAspected, const boost::system::error_code error);
    void on_udpSocket_dataSent(char* outputBuffer, size_t numberOfSentBytes, unsigned int numberOfSentBytesAspected, const boost::system::error_code error);
};

#endif // NETWORKCONNECTION_H
