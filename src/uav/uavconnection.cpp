#include "uavconnection.h"

UavConnection::UavConnection(unsigned int id, OCUConnection* ocuConnection, NetworkConnection* networkConnection)
{
    this->id = id;

    this->ocuConnection = ocuConnection;
    this->networkConnection = networkConnection;

    this->networkConnection->connectionClosedEvent.registerEventHandler(this, &UavConnection::on_networkConnection_connectionClosed);

    this->uav = new Uav(id);
    this->ocuCommunicationInterface = new OCUCommunicationInterface(ocuConnection);
    this->uavCommunicationInterface = new UavCommunicationInterface(networkConnection);

    this->mainInteractor = new MainInteractor(this->ocuCommunicationInterface, this->uavCommunicationInterface, this->uav);

    this->mainInteractor->start();

    this->networkConnection->start();
}

UavConnection::~UavConnection()
{
    delete this->mainInteractor;
    delete this->uav;
    delete this->ocuCommunicationInterface;
    delete this->uavCommunicationInterface;

    cout << "UAV Connection destroied" << endl;
}

Uav* UavConnection::getUav()
{
    return this->uav;
}

void UavConnection::on_networkConnection_connectionClosed(NetworkConnection* closedConnection)
{
    this->ocuCommunicationInterface->ocu->closeConnection();
    this->connectionClosedEvent.triggerEvent(this);
}

