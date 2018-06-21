#ifndef UAVCOMMUNICATIONINTERFACE_H
#define UAVCOMMUNICATIONINTERFACE_H

#include "utils/event.h"

#include "network/networkconnection.h"

#include "tradr_asset_message.pb.h"
#include "tradr_asset_reg_messages.pb.h"
#include "tradr_asset_status_messages.pb.h"
#include "tradr_bridge_message.pb.h"
#include "tradr_camera_messages.pb.h"
#include "tradr_common_messages.pb.h"
#include "tradr_task_messages.pb.h"

#include <iostream>

class UavCommunicationInterface
{
private:
    enum TransportProtocol
    {
        TCP,
        UDP
    };

public:
    Event<UavCommunicationInterface, const tradr::AssetRegReqMsg&> registerReqMsgReceivedFromUavEvent;
    Event<UavCommunicationInterface, const tradr::AssetStatusMsg&> statusMsgReceivedFromUavEvent;
    Event<UavCommunicationInterface, const tradr::VideostreamStartAckMsg&> startVideostreamAckMsgReceivedFromUavEvent;
    Event<UavCommunicationInterface, const tradr::VideostreamDataMsg&> videostreamDataMsgReceivedFromUavEvent;
    Event<UavCommunicationInterface, const tradr::TaskRespMsg&> taskAckMsgReceivedFromUavEvent;
    Event<UavCommunicationInterface, const tradr::TaskFeedbackMsg&> taskFeedbackMsgReceivedFromUavEvent;

private:
    NetworkConnection* networkConnection;   //shared

    std::string tag;

public:
    UavCommunicationInterface(NetworkConnection* networkConnection);

    void sendRegisterAckMsgToUav(tradr::AssetRegACKMsg* msg);
    void sendStartVideotreamReqMsgToUav(tradr::VideostreamStartReqMsg* msg);
    void sendStopVideotreamReqMsgToUav(tradr::VideostreamStopReqMsg* msg);
    void sendTaskReqMsgToUav(tradr::TaskMsg* msg);

private:
    void on_networkConnection_messageReceived(char* messageData, unsigned int messageLength);

    void send(tradr::BRIDGEMsg* msg, TransportProtocol transportProtocol);

};

#endif // UAVCOMMUNICATIONINTERFACE_H
