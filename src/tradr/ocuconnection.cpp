#include "ocuconnection.h"

OCUConnection::OCUConnection(unsigned int uavID, ros::NodeHandle* nodeHandle)
{
    this->uavID = uavID;
    this->uavName = "uav_" + std::to_string(uavID);

    this->initialized = false;

    this->nodeHandle = nodeHandle;

    this->info        = new Info(this);
    this->status      = new Status(this);
    this->task        = new Task(this);
    this->videostream = new Videostream(this);

}

OCUConnection::~OCUConnection()
{
    delete this->info;
    delete this->status;
    delete this->task;
    delete this->videostream;
}

void OCUConnection::init()
{
    this->info->init();
    this->status->init();
    this->videostream->init();
    this->task->init();

    this->initialized = true;

    this->connectionRegisteredEvent.triggerEvent(this);
}

void OCUConnection::closeConnection()
{
    this->connectionClosedEvent.triggerEvent(this);
}

unsigned int OCUConnection::getID()
{
    return this->uavID;
}

bool OCUConnection::isInitialized()
{
    return this->initialized;
}

OCUConnection::Info::Info(OCUConnection* ocuConnection)
{
    this->ocuConnection = ocuConnection;
}

void OCUConnection::Info::updateState(bridge_path_interface_msgs::UavState& msg)
{
    if (this->ocuConnection->isInitialized())
    {
        this->stateChangedNotificationPublisher.publish(msg);
    }
}

void OCUConnection::Info::init()
{
    ros::NodeHandle* nodeHandle = this->ocuConnection->nodeHandle;
    std::string uavName = this->ocuConnection->uavName;

    this->getUavIdentService = nodeHandle->advertiseService(uavName + "/getUavIdent", &OCUConnection::Info::on_getUavIdentService_serviceCalled, this);
    this->getUavStateService = nodeHandle->advertiseService(uavName + "/getUavState", &OCUConnection::Info::on_getUavStateService_serviceCalled, this);

    this->stateChangedNotificationPublisher = nodeHandle->advertise<bridge_path_interface_msgs::UavState>(uavName + "/stateChangedNotification", 1);
}

bool OCUConnection::Info::on_getUavIdentService_serviceCalled(bridge_path_interface_msgs::GetUavIdentRequest& request, bridge_path_interface_msgs::GetUavIdentResponse& response)
{
    response.uavIdent = (this->uavIdentRequestedEvent.triggerEvent().front());
    return true;
}

bool OCUConnection::Info::on_getUavStateService_serviceCalled(bridge_path_interface_msgs::GetUavStateRequest& request, bridge_path_interface_msgs::GetUavStateResponse& response)
{
    response.uavState = (this->uavStateRequestedEvent.triggerEvent().front());
    return true;
}


OCUConnection::Status::Status(OCUConnection* ocuConnection)
{
    this->ocuConnection = ocuConnection;
}

void OCUConnection::Status::init()
{
    ros::NodeHandle* nodeHandle = this->ocuConnection->nodeHandle;
    std::string uavName = this->ocuConnection->uavName;

    this->statusPublisher = nodeHandle->advertise<bridge_path_interface_msgs::Status>(uavName + "/status", 1);
}

void OCUConnection::Status::update(bridge_path_interface_msgs::Status& msg)
{
    if (this->ocuConnection->isInitialized())
    {
        this->statusPublisher.publish(msg);
    }
}


OCUConnection::Videostream::Videostream(OCUConnection* ocuConnection)
{
    this->ocuConnection = ocuConnection;
}

void OCUConnection::Videostream::init()
{
    ros::NodeHandle* nodeHandle = this->ocuConnection->nodeHandle;
    std::string uavName = this->ocuConnection->uavName;

    this->videostreamStartReqSubscriber = nodeHandle->subscribe<uav_videostream_msgs::VideostreamStartReq>(uavName + "/videostream/startReq", 1, &Videostream::on_videostreamStartReqSubscriber_messageReceived, this);
    this->videostreamStopReqSubscriber = nodeHandle->subscribe<uav_videostream_msgs::VideostreamStopReq>(uavName + "/videostream/stopReq", 1, &Videostream::on_videostreamStopReqSubscriber_messageReceived, this);
    this->videostreamStartRspPublisher = nodeHandle->advertise<uav_videostream_msgs::VideostreamStartRsp>(uavName + "/videostream/startRsp", 1);

    this->videostreamDataPublisher = nodeHandle->advertise<uav_videostream_msgs::VideostreamData>(uavName + "/videostream/data", 100);
}

void OCUConnection::Videostream::sendStartResponse(uav_videostream_msgs::VideostreamStartRsp &msg)
{
    if (this->ocuConnection->isInitialized())
    {
        this->videostreamStartRspPublisher.publish(msg);
    }
}

void OCUConnection::Videostream::updateFrame(uav_videostream_msgs::VideostreamData& msg)
{
    if (this->ocuConnection->isInitialized())
    {
        this->videostreamDataPublisher.publish(msg);
    }
}

void OCUConnection::Videostream::on_videostreamStartReqSubscriber_messageReceived(const uav_videostream_msgs::VideostreamStartReqConstPtr& msg)
{
    this->videostreamStartRequestedEvent.triggerEvent(msg);
}

void OCUConnection::Videostream::on_videostreamStopReqSubscriber_messageReceived(const uav_videostream_msgs::VideostreamStopReqConstPtr& msg)
{
    this->videostreamStopRequestedEvent.triggerEvent(msg);
}

OCUConnection::Task::Task(OCUConnection* ocuConnection)
{
    this->ocuConnection = ocuConnection;
}

void OCUConnection::Task::init()
{
    ros::NodeHandle* nodeHandle = this->ocuConnection->nodeHandle;
    std::string uavName = this->ocuConnection->uavName;

    this->taskReqSubscriber = nodeHandle->subscribe<bridge_path_interface_msgs::TaskReq>(uavName + "/task/req", 1, &Task::on_taskReqSubscriber_messageReceived, this);
    this->taskControlSubscriber = nodeHandle->subscribe<bridge_path_interface_msgs::TaskControl>(uavName + "/task/control", 1, &Task::on_taskControlSubscriber_messageReceived, this);

    this->taskAckPublisher = nodeHandle->advertise<bridge_path_interface_msgs::TaskAck>(uavName + "/task/ack", 1);
    this->taskStatusPublisher = nodeHandle->advertise<bridge_path_interface_msgs::TaskFeedback>(uavName + "/task/status", 1);
    this->taskFinPublisher = nodeHandle->advertise<bridge_path_interface_msgs::TaskFin>(uavName + "/task/fin", 1);
}

void OCUConnection::Task::updateStatus(bridge_path_interface_msgs::TaskFeedback& msg)
{
    if (this->ocuConnection->isInitialized())
    {
        this->taskStatusPublisher.publish(msg);
    }
}

void OCUConnection::Task::accept(bridge_path_interface_msgs::TaskAck& msg)
{
    if (this->ocuConnection->isInitialized())
    {
        this->taskAckPublisher.publish(msg);
    }
}

void OCUConnection::Task::on_taskReqSubscriber_messageReceived(const bridge_path_interface_msgs::TaskReqConstPtr& msg)
{
    if (this->ocuConnection->isInitialized())
    {
        this->taskRequestedEvent.triggerEvent(msg);
    }
}

void OCUConnection::Task::on_taskControlSubscriber_messageReceived(const bridge_path_interface_msgs::TaskControlConstPtr& msg)
{
    if (this->ocuConnection->isInitialized())
    {
        this->taskControlledEvent.triggerEvent(msg);
    }
}
