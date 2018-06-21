#ifndef NETWORKSERVER_H
#define NETWORKSERVER_H

#include <boost/asio.hpp>
#include <set>
#include <list>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include "utils/event.h"

#include "network/networkconnection.h"


using namespace boost::asio::ip;
using namespace boost::asio;
using namespace boost::system;

using namespace std;

class NetworkServer
{
public:
    Event<NetworkServer, NetworkConnection*> clientConnectedEvent;

private:
    io_service* ioService;                      //shared

    tcp::endpoint* tcpServerEndpoint;           //Component
    tcp::acceptor* tcpServerAcceptor;           //Component

    udp::endpoint* udpServerEndpoint;           //Component
    udp::socket*   udpServerSocket;             //Component

    list<NetworkConnection*> connectionList;    //Component List



public:
    NetworkServer(io_service* ioService);
    ~NetworkServer();

    void start();



private:
    void asyncWaitForConnection();

    void on_tcpServerAcceptor_newConnectionAccepted(tcp::socket* socket, const boost::system::error_code error);

    void on_networkConnection_connectionClosed(NetworkConnection* closedConnection);


};

#endif // NETWORKSERVER_H
