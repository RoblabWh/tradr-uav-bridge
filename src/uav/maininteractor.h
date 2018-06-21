#ifndef MAININTERACTOR_H
#define MAININTERACTOR_H

#include "tradr_asset_message.pb.h"
#include "tradr_bridge_message.pb.h"

#include "tradr_asset_reg_messages.pb.h"

#include "uav/uavcommunicationinterface.h"
#include "uav/ocucommunicationinterface.h"

#include "uav/statusinteractor.h"
#include "uav/streaminteractor.h"
#include "uav/taskinteractor.h"
#include "uav/uav.h"

#include "bridge_path_interface_msgs/UavIdent.h"
#include "bridge_path_interface_msgs/RegistrationNotification.h"


class MainInteractor
{

public:
    Event<MainInteractor, bridge_path_interface_msgs::UavIdent&> uavRegisteredEvent;

    Event<MainInteractor> stateChangedEvent;


private:
    Uav::MainState state;

    Uav* uav;                                   // shared

    UavCommunicationInterface* uavInterface;    // shared
    OCUCommunicationInterface* ocuInterface;    // shared

    StatusInteractor* statusInteractor;         // component
    StreamInteractor* streamInteractor;         // component
    TaskInteractor* taskInteractor;             // component


public:
    MainInteractor(OCUCommunicationInterface* ocuCommInterface, UavCommunicationInterface* uavCommInterface, Uav* uav);
    ~MainInteractor();

public:
    void start();

private:
    void on_uavInterface_registerReqMsgReceived(const tradr::AssetRegReqMsg& msg);

    bridge_path_interface_msgs::UavIdent on_ocuInterface_uavIdentRequested();
    bridge_path_interface_msgs::UavState on_ocuInterface_uavStateRequested();

    void on_stateChanged();

    bridge_path_interface_msgs::UavState getStateMsg();

    void registerUav(const tradr::AssetRegReqMsg& msg);

};

#endif // MAININTERACTOR_H
