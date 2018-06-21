#include "uavserver.h"


UavServer::UavServer(OCUServer* ocuServer, NetworkServer* networkServer, TerminalGUI* terminalGUI)
{
    this->nextConnectionID = 0;

    this->networkServer = networkServer;
    this->ocuServer = ocuServer;
    this->terminalGUI = terminalGUI;

    this->networkServer->clientConnectedEvent.registerEventHandler(this, &UavServer::on_networkServer_clientConnected);

    //this->ocuServer->uavListRequestedEvent.registerEventHandler(this, &UavServer::on_tradr_getUavList);
}

UavServer::~UavServer()
{
    for (UavConnection* connection : this->connectionList)
    {
        delete connection;
    }

    this->connectionList.clear();
}


void UavServer::start()
{
    this->networkServer->start();
    this->ocuServer->start();
}

void UavServer::on_networkServer_clientConnected(NetworkConnection* networkConnection)
{
    OCUConnection* ocuConnection = this->ocuServer->createOCUConnection(this->nextConnectionID);

    UavConnection* uavConnection = new UavConnection(this->nextConnectionID, ocuConnection, networkConnection);

    uavConnection->connectionClosedEvent.registerEventHandler(this, &UavServer::on_uavConnectionList_connectionClosed);

    this->terminalGUI->addUavEntry(uavConnection->getUav());

    this->connectionList.push_back(uavConnection);
    this->nextConnectionID++;
}



void UavServer::on_uavConnectionList_connectionClosed(UavConnection* uavConnection)
{
    this->terminalGUI->removeUavEntry(uavConnection->getUav());

    this->connectionList.remove(uavConnection);
    delete uavConnection;
}

/*
bridge_path_interface_msgs::UavList* UavServer::on_tradr_getUavList()
{
    bridge_path_interface_msgs::UavList* uavList = new bridge_path_interface_msgs::UavList;
    int i = 0;
    for (UavConnection* connection : this->connectionList)
    {
        if (connection->isRegistered())
        {
            uavList->uavs.push_back(connection->getUavRegistration());
            i++;
        }
    }
    uavList->numberOfUavs = i;

    return uavList;
}
*/





