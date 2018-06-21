#include "ocuserver.h"

OCUServer::OCUServer()
{
    this->nodeHandle = new ros::NodeHandle("");
}

OCUServer::~OCUServer()
{
    for (OCUConnection* connection : this->ocuConnectionList)
    {
        delete connection;
    }
    this->ocuConnectionList.clear();

    for (TradrDB* dbConnection : this->tradrDBList)
    {
        delete dbConnection;
    }
    this->tradrDBList.clear();

    delete this->nodeHandle;
}

void OCUServer::start()
{
    this->uavRegistrationNotificationPublisher = nodeHandle->advertise<bridge_path_interface_msgs::RegistrationNotification>("uav_registration_notification", 1, true);

    this->getRegistrationListService = nodeHandle->advertiseService("getRegistrationList", &OCUServer::on_getRegistrationListService_serviceCalled, this);

    this->sendResetNotification();
}


TradrDB* OCUServer::getTradrDB()
{
    TradrDB* tradrDB = new TradrDB(this->nodeHandle);
    this->tradrDBList.push_back(tradrDB);
    return tradrDB;
}

OCUConnection* OCUServer::createOCUConnection(unsigned int uavID)
{
    OCUConnection* ocuConnection = new OCUConnection(uavID, this->nodeHandle);
    ocuConnection->connectionClosedEvent.registerEventHandler(this, &OCUServer::on_ocuConnectionList_connectionClosed);
    ocuConnection->connectionRegisteredEvent.registerEventHandler(this, &OCUServer::on_ocuConnectionList_connectionRegistered);
    this->ocuConnectionList.push_back(ocuConnection);

    return ocuConnection;
}

void OCUServer::on_ocuConnectionList_connectionClosed(OCUConnection* ocuConnection)
{
    bridge_path_interface_msgs::RegistrationNotification notification;
    notification.type = notification.TYPE_UNREGISTRATION;
    notification.id = ocuConnection->getID();

    this->sendUavRegistrationNotification(notification);

    this->ocuConnectionList.remove(ocuConnection);

    delete ocuConnection;
}

void OCUServer::on_ocuConnectionList_connectionRegistered(OCUConnection* ocuConnection)
{
    bridge_path_interface_msgs::RegistrationNotification notification;
    notification.type = notification.TYPE_REGISTRATION;
    notification.id = ocuConnection->getID();

    this->sendUavRegistrationNotification(notification);
}

bool OCUServer::on_getRegistrationListService_serviceCalled(bridge_path_interface_msgs::GetRegistrationListRequest& request, bridge_path_interface_msgs::GetRegistrationListResponse& response)
{
    for (OCUConnection* ocuConnection : this->ocuConnectionList) {
        if (ocuConnection->isInitialized())
        {
            response.registrationList.id.push_back(ocuConnection->getID());
        }
    }

    return true;
}

void OCUServer::sendUavRegistrationNotification(bridge_path_interface_msgs::RegistrationNotification& msg)
{
    this->uavRegistrationNotificationPublisher.publish(msg);
}

void OCUServer::sendResetNotification()
{
    bridge_path_interface_msgs::RegistrationNotification notification;
    notification.type = notification.TYPE_RESET;
    notification.id = 0;

    this->sendUavRegistrationNotification(notification);
}



