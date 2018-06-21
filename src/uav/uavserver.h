#ifndef UAVSERVER_H
#define UAVSERVER_H

#include <boost/asio.hpp>
#include <set>
#include <list>

#include "network/networkserver.h"
#include "network/networkconnection.h"

#include "tradr/ocuserver.h"
#include "tradr/ocuconnection.h"

#include "tgui/terminalgui.h"

#include "uav/uavconnection.h"

#include "bridge_path_interface_msgs/RegistrationList.h"


using namespace boost::system;

using namespace std;

class UavServer
{

private:
    OCUServer* ocuServer;                   //shared
    NetworkServer* networkServer;           //shared
    TerminalGUI* terminalGUI;               //shared

    list<UavConnection*> connectionList;    //Component List

    unsigned int nextConnectionID;


public:
    UavServer(OCUServer* ocuServer, NetworkServer* networkServer, TerminalGUI* terminalGUI);
    ~UavServer();
    void start();


private:
    void on_networkServer_clientConnected(NetworkConnection* networkConnection);

    void on_uavConnectionList_connectionClosed(UavConnection* uavConnection);

    //bridge_path_interface_msgs::UavList* on_tradr_getUavList();
};

#endif // UAVSERVER_H
