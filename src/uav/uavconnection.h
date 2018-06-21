#ifndef UAVCONNECTION_H
#define UAVCONNECTION_H

#include <boost/asio.hpp>
#include <set>

#include "network/networkconnection.h"

#include "uav/uavcommunicationinterface.h"
#include "uav/ocucommunicationinterface.h"

#include "uav/uav.h"

#include "uav/maininteractor.h"



using namespace boost::asio::ip;
using namespace boost::asio;
using namespace boost::system;

using namespace std;

class UavConnection
{
public:
    Event<UavConnection, UavConnection*> connectionClosedEvent;

private:
    int id;

    NetworkConnection* networkConnection;                   // shared
    OCUConnection* ocuConnection;                           // shared

    MainInteractor* mainInteractor;                         // Component

    Uav* uav;                                               // Component

    OCUCommunicationInterface* ocuCommunicationInterface;   // Component
    UavCommunicationInterface* uavCommunicationInterface;   // Component


public:
    UavConnection(unsigned int id, OCUConnection* ocuConnection, NetworkConnection* networkConnection);
    ~UavConnection();

    Uav* getUav();

    //bool isRegistered();
    //bridge_path_interface_msgs::Uav getUavRegistration();


private:
    //void on_mainInteractor_uavRegistered(bridge_path_interface_msgs::Uav& uav);
    void on_networkConnection_connectionClosed(NetworkConnection* closedConnection);

};

#endif // UAVCONNECTION_H
