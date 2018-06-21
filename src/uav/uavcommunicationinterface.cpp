#include "uavcommunicationinterface.h"


UavCommunicationInterface::UavCommunicationInterface(NetworkConnection* networkConnection)
{
    this->networkConnection = networkConnection;

    this->networkConnection->messageReceivedEvent.registerEventHandler(this, &UavCommunicationInterface::on_networkConnection_messageReceived);
}

void UavCommunicationInterface::sendRegisterAckMsgToUav(tradr::AssetRegACKMsg* msg)
{
    tradr::BRIDGEMsg envelope;
    envelope.set_allocated_regresponse(msg);
    envelope.set_tag(this->tag.c_str(), this->tag.length());

    this->send(&envelope, TCP);
}

void UavCommunicationInterface::sendStopVideotreamReqMsgToUav(tradr::VideostreamStopReqMsg* msg)
{
    tradr::BRIDGEMsg envelope;
    envelope.set_allocated_stopvideostreamrequest(msg);
    envelope.set_tag(this->tag.c_str(), this->tag.length());

    this->send(&envelope, TCP);
}

void UavCommunicationInterface::sendStartVideotreamReqMsgToUav(tradr::VideostreamStartReqMsg* msg)
{
    tradr::BRIDGEMsg envelope;
    envelope.set_allocated_startvideostreamrequest(msg);
    envelope.set_tag(this->tag.c_str(), this->tag.length());

    this->send(&envelope, TCP);
}

void UavCommunicationInterface::sendTaskReqMsgToUav(tradr::TaskMsg* msg)
{
    tradr::BRIDGEMsg envelope;
    envelope.set_allocated_task(msg);
    envelope.set_tag(this->tag.c_str(), this->tag.length());
    this->send(&envelope, TCP);
}

void UavCommunicationInterface::on_networkConnection_messageReceived(char* messageData, unsigned int messageLength)
{
    tradr::AssetMsg msg;

    if (msg.ParseFromArray(messageData, messageLength))
    {
        this->tag = msg.tag();

        switch (msg.AssetMsgOneof_case()) {
        case tradr::AssetMsg::kRegRequest:
            this->registerReqMsgReceivedFromUavEvent.triggerEvent(msg.regrequest());
            break;
        case tradr::AssetMsg::kAssetStatus:
            this->statusMsgReceivedFromUavEvent.triggerEvent(msg.assetstatus());
            break;
        case tradr::AssetMsg::kStartVideostreamAck:
            this->startVideostreamAckMsgReceivedFromUavEvent.triggerEvent(msg.startvideostreamack());
            break;
        case tradr::AssetMsg::kVideostreamData:
            this->videostreamDataMsgReceivedFromUavEvent.triggerEvent(msg.videostreamdata());
            break;
        case tradr::AssetMsg::kTaskResponse:
            this->taskAckMsgReceivedFromUavEvent.triggerEvent(msg.taskresponse());
            break;
        case tradr::AssetMsg::kTaskFeedback:
            this->taskFeedbackMsgReceivedFromUavEvent.triggerEvent(msg.taskfeedback());
            break;
        default:
            break;
        }
    }
    else
    {
        cout << "message could not be parsed (" << messageLength << ")" << endl;
    }

}

void UavCommunicationInterface::send(tradr::BRIDGEMsg* msg, UavCommunicationInterface::TransportProtocol transportProtocol)
{
    unsigned int size = msg->ByteSize();
    char* data = new char[msg->ByteSize()];

    if (msg->SerializeToArray(data, size))
    {
        switch (transportProtocol) {
        case TCP:
            this->networkConnection->sendMessageTCP(data, size);
            break;
        case UDP:
            this->networkConnection->sendMessageUDP(data, size);
            break;
        default:
            break;
        }

    }
    else
    {
        cout << "Error ocurred on serialization" << endl;
    }

    delete data;
}
