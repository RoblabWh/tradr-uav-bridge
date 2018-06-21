#ifndef OCUSERVER_H
#define OCUSERVER_H

#include <ros/ros.h>

#include <list>

#include "tradr/tradrdb.h"
#include "tradr/ocuconnection.h"

#include <bridge_path_interface_msgs/GetRegistrationList.h>
#include <bridge_path_interface_msgs/RegistrationNotification.h>
#include <std_msgs/Empty.h>

using namespace std;

class OCUServer
{
private:
    ros::NodeHandle* nodeHandle;                //Component

    list<TradrDB*> tradrDBList;                 //Component List
    list<OCUConnection*> ocuConnectionList;     //Component List

    ros::Publisher uavRegistrationNotificationPublisher;

    ros::ServiceServer getRegistrationListService;

public:

    OCUServer();
    ~OCUServer();

    void start();

    TradrDB* getTradrDB();
    OCUConnection* createOCUConnection(unsigned int uavID);

private:
    void on_ocuConnectionList_connectionClosed(OCUConnection* ocuConnection);
    void on_ocuConnectionList_connectionRegistered(OCUConnection* ocuConnection);

    void sendUavRegistrationNotification(bridge_path_interface_msgs::RegistrationNotification& msg);
    void sendResetNotification();

    bool on_getRegistrationListService_serviceCalled(bridge_path_interface_msgs::GetRegistrationListRequest& request, bridge_path_interface_msgs::GetRegistrationListResponse& response);

};

#endif // OCUSERVER_H
