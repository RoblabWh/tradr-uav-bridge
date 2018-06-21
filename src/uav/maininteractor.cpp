#include "maininteractor.h"

MainInteractor::MainInteractor(OCUCommunicationInterface* ocuCommInterface, UavCommunicationInterface* uavCommInterface, Uav* uav)
{
    this->uav = uav;

    this->uavInterface = uavCommInterface;
    this->ocuInterface = ocuCommInterface;

    this->statusInteractor = new StatusInteractor(ocuInterface, uavInterface, uav);
    this->streamInteractor = new StreamInteractor(ocuInterface, uavInterface, uav);
    this->taskInteractor   = new TaskInteractor(ocuInterface, uavInterface, uav);

    this->uavInterface->registerReqMsgReceivedFromUavEvent.registerEventHandler(this, &MainInteractor::on_uavInterface_registerReqMsgReceived);
    this->ocuInterface->ocu->info->uavIdentRequestedEvent.registerEventHandler(this, &MainInteractor::on_ocuInterface_uavIdentRequested);
    this->ocuInterface->ocu->info->uavStateRequestedEvent.registerEventHandler(this, &MainInteractor::on_ocuInterface_uavStateRequested);

    this->stateChangedEvent.registerEventHandler(this, &MainInteractor::on_stateChanged);
    this->statusInteractor->stateChangedEvent.registerEventHandler(this, &MainInteractor::on_stateChanged);
    this->streamInteractor->stateChangedEvent.registerEventHandler(this, &MainInteractor::on_stateChanged);
    this->taskInteractor->stateChangedEvent.registerEventHandler(this, &MainInteractor::on_stateChanged);

    this->state = Uav::MainState::MAINSTATE_Init;
    this->uav->changeMainState(Uav::MainState::MAINSTATE_Init);
}

MainInteractor::~MainInteractor()
{
    delete this->statusInteractor;
    delete this->streamInteractor;
    delete this->taskInteractor;
}

void MainInteractor::start()
{
    this->state = Uav::MainState::MAINSTATE_WaitForRequest;
    this->uav->changeMainState(Uav::MainState::MAINSTATE_WaitForRequest);
    this->stateChangedEvent.triggerEvent();
}



void MainInteractor::registerUav(const tradr::AssetRegReqMsg& msg)
{
    cout << "Registration request:   " << msg.assetname() << "   " << msg.assetguid() << "   " << msg.assetmodel() << "   " << msg.apiversion() << endl;

    this->uav->registerUAV(msg.assetname(), msg.assetmodel(), msg.assetguid());

    tradr::AssetRegACKMsg* ackMsg = new tradr::AssetRegACKMsg;
    ackMsg->set_assetid(this->uav->getID());

    this->uavInterface->sendRegisterAckMsgToUav(ackMsg);

    this->ocuInterface->ocu->init();



    this->statusInteractor->start();
    this->streamInteractor->start();
    this->taskInteractor->start();

    this->state = Uav::MainState::MAINSTATE_Registered;
    this->uav->changeMainState(Uav::MainState::MAINSTATE_Registered);
    this->stateChangedEvent.triggerEvent();


}


void MainInteractor::on_uavInterface_registerReqMsgReceived(const tradr::AssetRegReqMsg& msg)
{
    switch (state) {
    case Uav::MainState::MAINSTATE_Init:
        cout << "UAV : register req : wrong state" << endl;
        break;
    case Uav::MainState::MAINSTATE_WaitForRequest:
        this->registerUav(msg);
        break;
    case Uav::MainState::MAINSTATE_Registered:
        cout << "UAV : register req : wrong state" << endl;
        break;
    default:
        break;
    }
}

bridge_path_interface_msgs::UavIdent MainInteractor::on_ocuInterface_uavIdentRequested()
{
    bridge_path_interface_msgs::UavIdent uavIdent;

    uavIdent.id = this->uav->getID();
    uavIdent.name = this->uav->getName();
    uavIdent.model = this->uav->getModel();
    uavIdent.guid = this->uav->getGUID();

    return uavIdent;
}

bridge_path_interface_msgs::UavState MainInteractor::on_ocuInterface_uavStateRequested()
{
    return this->getStateMsg();
}

void MainInteractor::on_stateChanged()
{
    bridge_path_interface_msgs::UavState uavState = this->getStateMsg();
    this->ocuInterface->ocu->info->updateState(uavState);
}

bridge_path_interface_msgs::UavState MainInteractor::getStateMsg()
{
    bridge_path_interface_msgs::UavState uavState;

    uavState.mainState = (uint8_t) this->state;
    uavState.statusState = (uint8_t) this->statusInteractor->getState();
    uavState.streamState = (uint8_t) this->streamInteractor->getState();
    uavState.taskState = (uint8_t) this->taskInteractor->getState();

    return uavState;
}
