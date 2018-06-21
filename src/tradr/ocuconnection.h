#ifndef TRADROCU_H
#define TRADROCU_H

#include <ros/ros.h>

#include <bridge_path_interface_msgs/GetUavIdent.h>
#include <bridge_path_interface_msgs/UavIdent.h>
#include <bridge_path_interface_msgs/GetUavState.h>
#include <bridge_path_interface_msgs/UavState.h>

#include <bridge_path_interface_msgs/Status.h>

#include <uav_videostream_msgs/VideostreamStartReq.h>
#include <uav_videostream_msgs/VideostreamStartRsp.h>
#include <uav_videostream_msgs/VideostreamStopReq.h>
#include <uav_videostream_msgs/VideostreamData.h>

#include <bridge_path_interface_msgs/TaskReq.h>
#include <bridge_path_interface_msgs/TaskAck.h>
#include <bridge_path_interface_msgs/TaskFeedback.h>
#include <bridge_path_interface_msgs/TaskControl.h>
#include <bridge_path_interface_msgs/TaskFin.h>

#include <string>

#include "utils/event.h"

class OCUConnection
{
    class Info
    {
    private:
        OCUConnection* ocuConnection;   //shared

        ros::ServiceServer getUavIdentService;
        ros::ServiceServer getUavStateService;

        ros::Publisher stateChangedNotificationPublisher;

    public:
        EventWithReturnValue<Info, bridge_path_interface_msgs::UavIdent> uavIdentRequestedEvent;
        EventWithReturnValue<Info, bridge_path_interface_msgs::UavState> uavStateRequestedEvent;

    public:
        Info(OCUConnection* ocuConnection);

        void updateState(bridge_path_interface_msgs::UavState& msg);

    public:
        void init();

    private:
        bool on_getUavIdentService_serviceCalled(bridge_path_interface_msgs::GetUavIdentRequest& request, bridge_path_interface_msgs::GetUavIdentResponse& response);
        bool on_getUavStateService_serviceCalled(bridge_path_interface_msgs::GetUavStateRequest& request, bridge_path_interface_msgs::GetUavStateResponse& response);
    };


    class Status
    {
    private:
        OCUConnection* ocuConnection;   //shared

        ros::Publisher statusPublisher;

    public:
        Status(OCUConnection* ocuConnection);

    public:
        void init();

    public:
        void update(bridge_path_interface_msgs::Status& msg);
    };


    class Videostream
    {
    private:
        OCUConnection* ocuConnection;   //shared

        ros::Subscriber videostreamStartReqSubscriber;
        ros::Publisher  videostreamStartRspPublisher;
        ros::Subscriber videostreamStopReqSubscriber;
        ros::Publisher  videostreamDataPublisher;

    public:
        Videostream(OCUConnection* ocuConnection);

    public:
        Event<Videostream, const uav_videostream_msgs::VideostreamStartReqConstPtr&> videostreamStartRequestedEvent;
        Event<Videostream, const uav_videostream_msgs::VideostreamStopReqConstPtr&> videostreamStopRequestedEvent;

    public:
        void init();

        void sendStartResponse(uav_videostream_msgs::VideostreamStartRsp& msg);
        void updateFrame(uav_videostream_msgs::VideostreamData& msg);

    private:
        void on_videostreamStartReqSubscriber_messageReceived(const uav_videostream_msgs::VideostreamStartReqConstPtr& msg);
        void on_videostreamStopReqSubscriber_messageReceived(const uav_videostream_msgs::VideostreamStopReqConstPtr& msg);
    };


    class Task
    {
    private:
        OCUConnection* ocuConnection;   //shared

        ros::Subscriber taskReqSubscriber;
        ros::Subscriber taskControlSubscriber;
        ros::Publisher  taskAckPublisher;
        ros::Publisher  taskStatusPublisher;
        ros::Publisher  taskFinPublisher;

    public:
        Task(OCUConnection* ocuConnection);

    public:
        Event<Task, const bridge_path_interface_msgs::TaskReqConstPtr&> taskRequestedEvent;
        Event<Task, const bridge_path_interface_msgs::TaskControlConstPtr&> taskControlledEvent;

    public:
        void init();

        void updateStatus(bridge_path_interface_msgs::TaskFeedback& msg);
        void finsh(bridge_path_interface_msgs::TaskFin& msg);
        void accept(bridge_path_interface_msgs::TaskAck& msg);
        void reject();

    private:
        void on_taskReqSubscriber_messageReceived(const bridge_path_interface_msgs::TaskReqConstPtr& msg);
        void on_taskControlSubscriber_messageReceived(const bridge_path_interface_msgs::TaskControlConstPtr& msg);
    };

public:
    Event<OCUConnection, OCUConnection*> connectionRegisteredEvent;
    Event<OCUConnection, OCUConnection*> connectionClosedEvent;

public:
    Info*         info;         // Component
    Status*       status;       // Component
    Videostream*  videostream;  // Component
    Task*         task;         // Component

private:
    int uavID;
    std::string uavName;
    bool initialized;
    ros::NodeHandle* nodeHandle; // shared

public:
    OCUConnection(unsigned int uavID, ros::NodeHandle* nodeHandle);
    ~OCUConnection();
    void init();
    void closeConnection();
    unsigned int getID();
    bool isInitialized();
};

#endif // TRADROCU_H
