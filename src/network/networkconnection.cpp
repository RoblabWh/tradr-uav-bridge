#include "networkconnection.h"



NetworkConnection::NetworkConnection(tcp::socket* tcpSocket, tcp::endpoint* tcpRemoteEndpoint, udp::socket* udpSocket, udp::endpoint* udpRemoteEndpoint)
{
    this->tcpSocket = tcpSocket;
    this->tcpRemoteEndpoint = tcpRemoteEndpoint;

    this->udpSocket = udpSocket;
    this->udpRemoteEndpoint = udpRemoteEndpoint;
}

NetworkConnection::~NetworkConnection()
{
    delete this->tcpSocket;
    this->udpSocket->cancel();

    delete this->tcpRemoteEndpoint;
    delete this->udpRemoteEndpoint;
}



void NetworkConnection::start()
{
    this->asyncReadLengthTCP();
    this->asyncReadContentUDP(50000);
}

void NetworkConnection::restartUDP()
{
    this->asyncReadContentUDP(50000);
}



void NetworkConnection::sendMessageTCP(char* messageData, unsigned int messageLength)
{
    char* outputBuffer = new char[messageLength + 4]();

    uint32_t lengthData = (uint32_t) messageLength;
    //boost::endian::native_to_big_inplace(lengthData);
    lengthData = htonl(lengthData);

    memcpy(outputBuffer, &lengthData, (size_t) 4);
    memcpy(outputBuffer + 4, messageData, (size_t) messageLength);

    this->tcpSocket->async_send(buffer(outputBuffer, messageLength + 4), boost::bind(&NetworkConnection::on_tcpSocket_dataSent, this, outputBuffer, boost::asio::placeholders::bytes_transferred, messageLength + 4, boost::asio::placeholders::error));

    //cout << "Connection " << this->tcpRemoteEndpoint->address().to_string() << " TCP  -  " << "try to send message (" << messageLength << " Bytes)..." << endl;
}



void NetworkConnection::sendMessageUDP(char* messageData, unsigned int messageLength)
{
    char* outputBuffer = new char[messageLength + 4]();

    uint32_t lengthData = (uint32_t) messageLength;
    //boost::endian::native_to_big_inplace(lengthData);
    lengthData = htonl(lengthData);

    memcpy(outputBuffer, &lengthData, 4);
    memcpy(outputBuffer + 4, messageData, messageLength);

    udpSocket->async_send_to(buffer(outputBuffer, messageLength + 4), *udpRemoteEndpoint, boost::bind(&NetworkConnection::on_udpSocket_dataSent, this, outputBuffer, boost::asio::placeholders::bytes_transferred, messageLength + 4, boost::asio::placeholders::error));

    //cout << "Connection " << this->udpRemoteEndpoint->address().to_string() << " UDP  -  " << "try to send message (" << messageLength << " Bytes)..." << endl;
}



void NetworkConnection::asyncReadLengthTCP()
{
    char* lengthBuffer = new char[4]();

    boost::asio::async_read(*(this->tcpSocket), buffer(lengthBuffer, 4), boost::asio::transfer_exactly(4), boost::bind(&NetworkConnection::on_tcpSocket_lengthReceived, this, lengthBuffer, boost::asio::placeholders::bytes_transferred, 4, boost::asio::placeholders::error));

    //cout << "Connection " << this->tcpRemoteEndpoint->address().to_string() << " TCP  -  " << "waiting for length data..." << endl;
}



void NetworkConnection::asyncReadContentTCP(unsigned int length)
{
    char* contentBuffer = new char[length]();

    boost::asio::async_read(*(this->tcpSocket), buffer(contentBuffer, length), boost::asio::transfer_exactly(length), boost::bind(&NetworkConnection::on_tcpSocket_contentReceived, this, contentBuffer, boost::asio::placeholders::bytes_transferred, length, boost::asio::placeholders::error));

    //cout << "Connection " << this->tcpRemoteEndpoint->address().to_string() << " TCP  -  " << "waiting for content data..." << endl;
}



void NetworkConnection::asyncReadLengthUDP()
{
    char* lengthBuffer = new char[4]();

    //this->udpSocket->async_receive_from(buffer(lengthBuffer, 4), *udpRemoteEndpoint, boost::bind(&NetworkConnection::on_udpSocket_lengthReceived, this, lengthBuffer, boost::asio::placeholders::bytes_transferred, 4, boost::asio::placeholders::error));

    //cout << "Connection " << this->tcpRemoteEndpoint->address().to_string() << " UDP  -  " << "waiting for length data..." << endl;

    boost::asio::mutable_buffer* nullBuffer = new boost::asio::mutable_buffer();
    this->udpSocket->async_receive_from(boost::asio::buffer(*nullBuffer), *udpRemoteEndpoint, boost::bind(&NetworkConnection::on_udpSocket_lengthReceived, this, nullBuffer, boost::asio::placeholders::bytes_transferred, 4, boost::asio::placeholders::error));
}



void NetworkConnection::asyncReadContentUDP(unsigned int length)
{
    char* contentBuffer = new char[length]();

    this->udpSocket->async_receive_from(buffer(contentBuffer, length), *udpRemoteEndpoint, boost::bind(&NetworkConnection::on_udpSocket_contentReceived, this, contentBuffer, boost::asio::placeholders::bytes_transferred, length, boost::asio::placeholders::error));

    //cout << "Connection " << this->tcpRemoteEndpoint->address().to_string() << " UDP  -  " << "waiting for content data..." << endl;
}



void NetworkConnection::on_tcpSocket_lengthReceived(char* receivedLengthData, size_t numberOfReceivedBytes, unsigned int numberOfReceivedBytesAspected, const boost::system::error_code error)
{
    if (error == NULL && numberOfReceivedBytes == numberOfReceivedBytesAspected)
    {
        unsigned int length = 0;
        memcpy(&length, receivedLengthData, 4);
        //boost::endian::big_to_native_inplace(length);
        length = ntohl(length);

        //cout << "Connection " << this->tcpRemoteEndpoint->address().to_string() << " TCP  -  " << "length received: " << length << endl;

        this->asyncReadContentTCP(length);
    }
    else
    {
        if (error == NULL)
        {

        }
        else if (error == boost::asio::error::eof)
        {
            cout << "Connection " << this->tcpRemoteEndpoint->address().to_string() << " TCP  -  " << "Connecton closed" << endl;
            this->connectionClosedEvent.triggerEvent(this);
        }
        else
        {
            cout << "Connection " << this->tcpRemoteEndpoint->address().to_string() << " TCP  -  " << "length receiving error: " << error.message() << "(" << error << ")" << "  " << numberOfReceivedBytes << endl;
        }

    }

    delete receivedLengthData;
}



void NetworkConnection::on_tcpSocket_contentReceived(char* receivedContentData, size_t numberOfReceivedBytes, unsigned int numberOfReceivedBytesAspected, const boost::system::error_code error)
{
    if (error == NULL && numberOfReceivedBytes == numberOfReceivedBytesAspected)
    {
        //cout << "Connection " << this->tcpRemoteEndpoint->address().to_string() << " TCP  -  " << "content received " << endl;

        this->messageReceivedEvent.triggerEvent(receivedContentData, numberOfReceivedBytes);

        this->asyncReadLengthTCP();
    }
    else
    {
        cout << "Connection " << this->tcpRemoteEndpoint->address().to_string() << " TCP  -  " << "content receiving error: " << error.message() << "(" << error << ")" << "  " << numberOfReceivedBytes << " of " << numberOfReceivedBytesAspected << endl;
    }

    delete receivedContentData;
}



void NetworkConnection::on_udpSocket_lengthReceived(boost::asio::mutable_buffer* receivedLengthData, size_t numberOfReceivedBytes, unsigned int numberOfReceivedBytesAspected, const boost::system::error_code error)
{
    if (error == NULL && numberOfReceivedBytes == numberOfReceivedBytesAspected)
    {
        unsigned int length;
        memcpy(&length, receivedLengthData, 4);
        //boost::endian::big_to_native_inplace(length);
        length = ntohl(length);

        //cout << "Connection " << this->udpRemoteEndpoint->address().to_string() << " UDP  -  " << "length received: " << length << endl;

        this->asyncReadContentUDP(udpSocket->available());
    }
    else
    {

        //cout << "Connection " << this->udpRemoteEndpoint->address().to_string() << " UDP  -  " << "length receiving error: " << error.message() << "(" << error << ")" << endl;
        this->asyncReadContentUDP(udpSocket->available());
    }

    delete receivedLengthData;
}



void NetworkConnection::on_udpSocket_contentReceived(char* receivedContentData, size_t numberOfReceivedBytes, unsigned int numberOfReceivedBytesAspected, const boost::system::error_code error)
{
    if (error == NULL) // && numberOfReceivedBytes == numberOfReceivedBytesAspected)
    {
        //cout << "Connection " << this->udpRemoteEndpoint->address().to_string() << " UDP  -  " << "content received " << endl;

        this->messageReceivedEvent.triggerEvent(receivedContentData, numberOfReceivedBytes);

        this->asyncReadContentUDP(50000);
    }
    else
    {
        cout << "Connection " << this->udpRemoteEndpoint->address().to_string() << " UDP  -  " << "content receiving error: " << error << endl;
    }

    delete receivedContentData;
}



void NetworkConnection::on_tcpSocket_dataSent(char* outputBuffer, size_t numberOfSentBytes, unsigned int numberOfSentBytesAspected, const boost::system::error_code error)
{
    if (error == NULL && numberOfSentBytes == numberOfSentBytesAspected)
    {
        cout << "Connection " << this->tcpRemoteEndpoint->address().to_string() << " TCP  -  " << "message sent successfully" << endl;
    }
    else
    {
        cout << "Connection " << this->tcpRemoteEndpoint->address().to_string() << " TCP  -  " << "message could not be sent: " << error << endl;
    }

    delete outputBuffer;
}



void NetworkConnection::on_udpSocket_dataSent(char *outputBuffer, size_t numberOfSentBytes, unsigned int numberOfSentBytesAspected, const boost::system::error_code error)
{
    if (error == NULL && numberOfSentBytes == numberOfSentBytesAspected)
    {
        cout << "Connection " << this->udpRemoteEndpoint->address().to_string() << " UDP  -  " << "message sent successfully" << endl;
    }
    else
    {
        cout << "Connection " << this->udpRemoteEndpoint->address().to_string() << " UDP  -  " << "message could not be sent: " << error << endl;
    }

    delete outputBuffer;
}
