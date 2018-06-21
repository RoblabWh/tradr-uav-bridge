#include "networkserver.h"



NetworkServer::NetworkServer(io_service *ioService)
{
    this->ioService = ioService;

    this->tcpServerEndpoint = new tcp::endpoint(tcp::v4(), 10000);
    this->tcpServerAcceptor = new tcp::acceptor(*ioService, *tcpServerEndpoint);

    this->udpServerEndpoint = new udp::endpoint(udp::v4(), 10000);
    this->udpServerSocket   = new udp::socket(*ioService, *udpServerEndpoint);


    this->udpServerSocket->set_option(boost::asio::socket_base::receive_buffer_size(8192000));
}

NetworkServer::~NetworkServer()
{
    for (NetworkConnection* connection : this->connectionList)
    {
        delete connection;
    }

    this->connectionList.clear();

    delete this->tcpServerAcceptor;
    delete this->tcpServerEndpoint;

    delete this->udpServerSocket;
    delete this->udpServerEndpoint;
}



void NetworkServer::start()
{
    cout << "TCP Listener started" << endl;

    this->asyncWaitForConnection();
}



void NetworkServer::asyncWaitForConnection()
{
    tcp::socket* socket = new tcp::socket(*ioService);

    this->tcpServerAcceptor->async_accept(*socket, boost::bind(&NetworkServer::on_tcpServerAcceptor_newConnectionAccepted, this, socket, boost::asio::placeholders::error));

    cout << "wait for clients..." << endl;
}



void NetworkServer::on_tcpServerAcceptor_newConnectionAccepted(tcp::socket* socket, const boost::system::error_code error)
{
    cout << "Client " << socket->remote_endpoint().address().to_string() << " tries to connect..." << endl;
    if (error == NULL)
    {
        tcp::socket*   tcpSocket         = socket;
        tcp::endpoint* tcpRemoteEndpoint = new tcp::endpoint(socket->remote_endpoint().address(), socket->remote_endpoint().port());
        udp::socket*   udpSocket         = this->udpServerSocket;
        udp::endpoint* udpRemoteEndpoint = new udp::endpoint(socket->remote_endpoint().address(), socket->remote_endpoint().port());

        NetworkConnection* newConnection = new NetworkConnection(tcpSocket, tcpRemoteEndpoint, udpSocket, udpRemoteEndpoint);

        newConnection->connectionClosedEvent.registerEventHandler(this, &NetworkServer::on_networkConnection_connectionClosed);

        this->connectionList.push_back(newConnection);

        cout << "Client " << socket->remote_endpoint().address().to_string() << " connected" << endl;

        this->clientConnectedEvent.triggerEvent(newConnection);

        this->asyncWaitForConnection();
    }
    else
    {
        cout << "Error while connection: " << error << endl;
    }
}

void NetworkServer::on_networkConnection_connectionClosed(NetworkConnection* closedConnection)
{
    this->connectionList.remove(closedConnection);
    delete closedConnection;

    for (NetworkConnection* connection : this->connectionList)
    {
        connection->restartUDP();
    }
}
