#include "statusinteractor.h"

StatusInteractor::StatusInteractor(OCUCommunicationInterface* ocuCommInterface, UavCommunicationInterface* uavCommInterface, Uav* uav)
{
    this->uav = uav;

    this->uavInterface = uavCommInterface;
    this->ocuInterface = ocuCommInterface;

    this->uavInterface->statusMsgReceivedFromUavEvent.registerEventHandler(this, &StatusInteractor::on_uavInterface_statusMsgReceived);

    this->state = Uav::StatusState::STATUSSTATE_Init;
    this->uav->changeStatusState(Uav::StatusState::STATUSSTATE_Init);
}

void StatusInteractor::start()
{
    this->state = Uav::StatusState::STATUSSTATE_UavMonitoring;
    this->uav->changeStatusState(Uav::StatusState::STATUSSTATE_UavMonitoring);
    this->stateChangedEvent.triggerEvent();
}

Uav::StatusState StatusInteractor::getState()
{
    return this->state;
}

void StatusInteractor::on_uavInterface_statusMsgReceived(const tradr::AssetStatusMsg& msg)
{
    switch (state) {
    case Uav::StatusState::STATUSSTATE_Init:
        //cout << "UAV : status : wrong state" << endl;
        break;
    case Uav::StatusState::STATUSSTATE_UavMonitoring:
        this->sendStatusToOCU(msg);
        break;
    default:
        break;
    }
}

void StatusInteractor::sendStatusToOCU(const tradr::AssetStatusMsg& msg)
{
    bridge_path_interface_msgs::Status statusMsg;

    if (msg.assetinformation_size() < 1)
    {
        return;
    }

    statusMsg.batteryStatus = msg.assetinformation(0).batterystatus();

    statusMsg.pose.altitude = msg.assetinformation(0).assetpose().altitude();
    statusMsg.pose.latitude = msg.assetinformation(0).assetpose().latitude();
    statusMsg.pose.longitude = msg.assetinformation(0).assetpose().longitude();
    statusMsg.pose.pitch = msg.assetinformation(0).assetpose().pitch();
    statusMsg.pose.roll = msg.assetinformation(0).assetpose().roll();
    statusMsg.pose.yaw = msg.assetinformation(0).assetpose().yaw();

    statusMsg.velocity.velX = msg.assetinformation(0).assetvelocity().velx();
    statusMsg.velocity.velY = msg.assetinformation(0).assetvelocity().vely();
    statusMsg.velocity.velY = msg.assetinformation(0).assetvelocity().velz();
    statusMsg.velocity.velPitch = msg.assetinformation(0).assetvelocity().velpitch();
    statusMsg.velocity.velRoll = msg.assetinformation(0).assetvelocity().velroll();
    statusMsg.velocity.velYaw = msg.assetinformation(0).assetvelocity().velyaw();

    statusMsg.currentWork.busy = (msg.assetinformation(0).sortiestatus().currenttaskid() != 0);
    statusMsg.currentWork.currentTaskID = msg.assetinformation(0).sortiestatus().currenttaskid();
    statusMsg.currentWork.currentWaypointID = msg.assetinformation(0).sortiestatus().currentwaypointid();
    statusMsg.currentWork.currentActionID = 0;

    this->ocuInterface->ocu->status->update(statusMsg);
}




