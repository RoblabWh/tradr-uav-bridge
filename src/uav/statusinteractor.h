#ifndef STATUSINTERACTOR_H
#define STATUSINTERACTOR_H

#include "tradr_asset_message.pb.h"

#include "uav/uavcommunicationinterface.h"
#include "uav/ocucommunicationinterface.h"
#include "uav/uav.h"



class StatusInteractor
{

public:
    Event<StatusInteractor> stateChangedEvent;

private:
    Uav::StatusState state;

    Uav* uav;       //shared

    UavCommunicationInterface* uavInterface;    //shared
    OCUCommunicationInterface* ocuInterface;    //shared


public:
    StatusInteractor(OCUCommunicationInterface* ocuCommInterface, UavCommunicationInterface* uavCommInterface, Uav* uav);

    void start();

    Uav::StatusState getState();

private:
    void on_uavInterface_statusMsgReceived(const tradr::AssetStatusMsg& msg);

    void sendStatusToOCU(const tradr::AssetStatusMsg& msg);

};

#endif // STATUSINTERACTOR_H
