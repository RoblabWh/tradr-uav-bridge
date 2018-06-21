#ifndef TASKINTERACTOR_H
#define TASKINTERACTOR_H

#include "tradr_asset_message.pb.h"

#include "uav/uavcommunicationinterface.h"
#include "uav/ocucommunicationinterface.h"
#include "uav/uav.h"

#include "bridge_path_interface_msgs/TaskFeedback.h"



class TaskInteractor
{

public:
    Event<TaskInteractor> stateChangedEvent;


private:
    Uav::TaskState state;

    Uav* uav;   //shared

    UavCommunicationInterface* uavInterface;    //shared
    OCUCommunicationInterface* ocuInterface;    //shared


public:
    TaskInteractor(OCUCommunicationInterface* ocuCommInterface, UavCommunicationInterface* uavCommInterface, Uav* uav);

    void start();

    Uav::TaskState getState();

private:
    void on_uavInterface_taskAckMsgReceived(const tradr::TaskRespMsg& msg);
    void on_uavInterface_taskFeedbackMsgReceived(const tradr::TaskFeedbackMsg& msg);
    void on_ocuInterface_taskRequested(const bridge_path_interface_msgs::TaskReqConstPtr& msg);
    void on_ocuInterface_taskControlled(const bridge_path_interface_msgs::TaskControlConstPtr& msg);

};

#endif // TASKINTERACTOR_H
