#include "taskinteractor.h"

TaskInteractor::TaskInteractor(OCUCommunicationInterface *ocuCommInterface, UavCommunicationInterface *uavCommInterface, Uav* uav)
{
    this->uav = uav;

    this->uavInterface = uavCommInterface;
    this->ocuInterface = ocuCommInterface;

    this->ocuInterface->ocu->task->taskRequestedEvent.registerEventHandler(this, &TaskInteractor::on_ocuInterface_taskRequested);
    this->ocuInterface->ocu->task->taskControlledEvent.registerEventHandler(this, &TaskInteractor::on_ocuInterface_taskControlled);

    this->uavInterface->taskAckMsgReceivedFromUavEvent.registerEventHandler(this, &TaskInteractor::on_uavInterface_taskAckMsgReceived);
    this->uavInterface->taskFeedbackMsgReceivedFromUavEvent.registerEventHandler(this, &TaskInteractor::on_uavInterface_taskFeedbackMsgReceived);

    this->state = Uav::TaskState::TASKSTATE_Init;
    this->uav->changeTaskState(Uav::TaskState::TASKSTATE_Init);
}

void TaskInteractor::start()
{
    this->state = Uav::TaskState::TASKSTATE_WaitForTask;
    this->uav->changeTaskState(Uav::TaskState::TASKSTATE_WaitForTask);
    this->stateChangedEvent.triggerEvent();
}

Uav::TaskState TaskInteractor::getState()
{
    return this->state;
}

void TaskInteractor::on_uavInterface_taskAckMsgReceived(const tradr::TaskRespMsg& msg)
{
    bridge_path_interface_msgs::TaskAck taskAck;
    taskAck.accepted = msg.taskack();

    this->ocuInterface->ocu->task->accept(taskAck);

    this->state = Uav::TaskState::TASKSTATE_Execution;
    this->uav->changeTaskState(Uav::TaskState::TASKSTATE_Execution);
    this->stateChangedEvent.triggerEvent();
}

void TaskInteractor::on_uavInterface_taskFeedbackMsgReceived(const tradr::TaskFeedbackMsg& msg)
{
    if (msg.has_taskstatus())
    {
        bridge_path_interface_msgs::TaskStatus taskStatus;
        taskStatus.taskID = msg.taskstatus().taskid();
        switch (msg.taskstatus().status())
        {
        case tradr::TaskStatusMsg::WAIT_FOR_EXECUTION:
            taskStatus.status = bridge_path_interface_msgs::TaskStatus::STATUS_WAIT_FOR_EXECUTION;
            break;
        case tradr::TaskStatusMsg::EXECUTION:
            taskStatus.status = bridge_path_interface_msgs::TaskStatus::STATUS_EXECUTION;
            break;
        case tradr::TaskStatusMsg::FINISHED:
            taskStatus.status = bridge_path_interface_msgs::TaskStatus::STATUS_FINISHED;
            break;
        case tradr::TaskStatusMsg::CANCELED:
            taskStatus.status = bridge_path_interface_msgs::TaskStatus::STATUS_CACELED;
            break;
        }

        bridge_path_interface_msgs::TaskFeedback feedback;
        feedback.taskStatus = taskStatus;

        this->ocuInterface->ocu->task->updateStatus(feedback);
    }
    else if (msg.has_waypointstatus())
    {
        bridge_path_interface_msgs::WaypointStatus waypointStatus;
        waypointStatus.taskID = msg.waypointstatus().taskid();
        waypointStatus.waypointID = msg.waypointstatus().waypointid();
        switch (msg.waypointstatus().status())
        {
        case tradr::WaypointStatusMsg::WAIT_FOR_EXECUTION:
            waypointStatus.status = bridge_path_interface_msgs::WaypointStatus::STATUS_WAIT_FOR_EXECUTION;
            break;
        case tradr::WaypointStatusMsg::EXECUTION_FLYING_TO_WAYPOINT:
            waypointStatus.status = bridge_path_interface_msgs::WaypointStatus::STATUS_EXECUTION_FLYING_TO_WAYPOINT;
            break;
        case tradr::WaypointStatusMsg::EXECUTION_DOING_ACTIONS:
            waypointStatus.status = bridge_path_interface_msgs::WaypointStatus::STATUS_EXECUTION_DOING_ACTIONS;
            break;
        case tradr::WaypointStatusMsg::FINISHED:
            waypointStatus.status = bridge_path_interface_msgs::WaypointStatus::STATUS_FINISHED;
            break;
        case tradr::WaypointStatusMsg::CANCELED:
            waypointStatus.status = bridge_path_interface_msgs::WaypointStatus::STATUS_CACELED;
            break;
        }

        bridge_path_interface_msgs::TaskFeedback feedback;
        feedback.waypointStatus = waypointStatus;

        this->ocuInterface->ocu->task->updateStatus(feedback);
    }
    else if (msg.has_actionstatus())
    {
        bridge_path_interface_msgs::ActionStatus actionStatus;
        actionStatus.taskID = msg.actionstatus().taskid();
        actionStatus.waypointID = msg.actionstatus().waypointid();
        actionStatus.actionID = msg.actionstatus().actionid();
        switch (msg.actionstatus().status())
        {
        case tradr::ActionStatusMsg::WAIT_FOR_EXECUTION:
            actionStatus.status = bridge_path_interface_msgs::ActionStatus::STATUS_WAIT_FOR_EXECUTION;
            break;
        case tradr::ActionStatusMsg::EXECUTION:
            actionStatus.status = bridge_path_interface_msgs::ActionStatus::STATUS_EXECUTION;
            break;
        case tradr::ActionStatusMsg::FINISHED:
            actionStatus.status = bridge_path_interface_msgs::ActionStatus::STATUS_FINISHED;
            break;
        case tradr::ActionStatusMsg::CANCELED:
            actionStatus.status = bridge_path_interface_msgs::ActionStatus::STATUS_CACELED;
            break;
        }

        bridge_path_interface_msgs::TaskFeedback feedback;
        feedback.actionStatus = actionStatus;

        this->ocuInterface->ocu->task->updateStatus(feedback);
    }

}

void TaskInteractor::on_ocuInterface_taskRequested(const bridge_path_interface_msgs::TaskReqConstPtr& msg)
{
    tradr::TaskMsg* task = new tradr::TaskMsg();

    task->set_taskid(msg->task.taskID);
    task->set_assetid(this->uav->getID());

    for (bridge_path_interface_msgs::Waypoint wp : msg->task.waypoints)
    {
        tradr::WaypointMsg* waypoint = task->add_waypoints();

        waypoint->set_velocity(wp.velocity);
        waypoint->set_waypointid(wp.waypointID);

        tradr::PoseMsg* uavPose = new tradr::PoseMsg();
        uavPose->set_altitude(wp.uavPose.altitude);
        uavPose->set_latitude(wp.uavPose.latitude);
        uavPose->set_longitude(wp.uavPose.longitude);
        uavPose->set_pitch(wp.uavPose.pitch);
        uavPose->set_roll(wp.uavPose.roll);
        uavPose->set_yaw(wp.uavPose.yaw);

        waypoint->set_allocated_uavpose(uavPose);

        for (bridge_path_interface_msgs::FotoAction fa : wp.fotoActions)
        {
            tradr::PoseMsg* cameraPose = new tradr::PoseMsg();
            cameraPose->set_altitude(fa.camPose.altitude);
            cameraPose->set_latitude(fa.camPose.latitude);
            cameraPose->set_longitude(fa.camPose.longitude);
            cameraPose->set_pitch(fa.camPose.pitch);
            cameraPose->set_roll(fa.camPose.roll);
            cameraPose->set_yaw(fa.camPose.yaw);

            /*
            tradr::CameraSettingsMsg* cameraSettings = new tradr::CameraSettingsMsg();
            cameraSettings->set_aperture("");
            cameraSettings->set_iso(std::string("300"));
            cameraSettings->set_ratio("3/4");
            cameraSettings->set_shutterspeed("");
            */

            tradr::ImageRequestMsg* action = waypoint->add_imagerequests();
            action->set_allocated_campose(cameraPose);
            //action->set_allocated_camsettings(cameraSettings);
        }

    }

    this->uavInterface->sendTaskReqMsgToUav(task);

    this->state = Uav::TaskState::TASKSTATE_WaitForResponse;
    this->uav->changeTaskState(Uav::TaskState::TASKSTATE_WaitForResponse);
    this->stateChangedEvent.triggerEvent();

}

void TaskInteractor::on_ocuInterface_taskControlled(const bridge_path_interface_msgs::TaskControlConstPtr& msg)
{

}



