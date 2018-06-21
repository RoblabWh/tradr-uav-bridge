#include "streaminteractor.h"

StreamInteractor::StreamInteractor(OCUCommunicationInterface* ocuCommInterface, UavCommunicationInterface* uavCommInterface, Uav* uav)
{
    this->uav = uav;

    this->uavInterface = uavCommInterface;
    this->ocuInterface = ocuCommInterface;

    this->uavInterface->startVideostreamAckMsgReceivedFromUavEvent.registerEventHandler(this, &StreamInteractor::on_uavInterface_startVideostreamAckMsgReceived);
    this->uavInterface->videostreamDataMsgReceivedFromUavEvent.registerEventHandler(this, &StreamInteractor::on_uavInterface_videostreamDataMsgReceived);

    this->ocuInterface->ocu->videostream->videostreamStartRequestedEvent.registerEventHandler(this, &StreamInteractor::on_tradrInterface_videostreamStartRequested);
    this->ocuInterface->ocu->videostream->videostreamStopRequestedEvent.registerEventHandler(this, &StreamInteractor::on_tradrInterface_videostreamStopRequested);

    this->state = Uav::StreamState::STREAMSTATE_Init;
    this->uav->changeStreamState(Uav::StreamState::STREAMSTATE_Init);
}

void StreamInteractor::start()
{
    this->state = Uav::StreamState::STREAMSTATE_ReceivingVideoDataOff;
    this->uav->changeStreamState(Uav::StreamState::STREAMSTATE_ReceivingVideoDataOff);
    this->stateChangedEvent.triggerEvent();
}

Uav::StreamState StreamInteractor::getState()
{
    return this->state;
}

void StreamInteractor::on_uavInterface_startVideostreamAckMsgReceived(const tradr::VideostreamStartAckMsg& msg)
{
    switch (state) {
    case Uav::StreamState::STREAMSTATE_Init:
        cout << "UAV : videostream start ack : wrong state" << endl;
        break;
    case Uav::StreamState::STREAMSTATE_ReceivingVideoDataOff:
        cout << "UAV : videostream start ack : wrong state" << endl;
        break;
    case Uav::StreamState::STREAMSTATE_WaitForResponse:
        this->switchVideoReceivingOn(msg);
        break;
    case Uav::StreamState::STREAMSTATE_ReceivingVideoDataOn:
        cout << "UAV : videostream start ack : wrong state" << endl;
        break;
    default:
        break;
    }
}

void StreamInteractor::on_uavInterface_videostreamDataMsgReceived(const tradr::VideostreamDataMsg& msg)
{
    switch (state) {
    case Uav::StreamState::STREAMSTATE_Init:
        cout << "UAV : data : wrong state" << endl;
        break;
    case Uav::StreamState::STREAMSTATE_ReceivingVideoDataOff:
        cout << "UAV : data : wrong state" << endl;
        break;
    case Uav::StreamState::STREAMSTATE_WaitForResponse:
        cout << "UAV : data : wrong state" << endl;
        break;
    case Uav::StreamState::STREAMSTATE_ReceivingVideoDataOn:
        this->processVideoData(msg);
        break;
    default:
        break;
    }
}

void StreamInteractor::on_tradrInterface_videostreamStartRequested(const uav_videostream_msgs::VideostreamStartReqConstPtr &msg)
{
    switch (state) {
    case Uav::StreamState::STREAMSTATE_Init:
        cout << "ROS : videostream start req : wrong state" << endl;
        break;
    case Uav::StreamState::STREAMSTATE_ReceivingVideoDataOff:
        this->sendStartRequestToUav(msg);
        break;
    case Uav::StreamState::STREAMSTATE_WaitForResponse:
        cout << "ROS : videostream start req : wrong state" << endl;
        break;
    case Uav::StreamState::STREAMSTATE_ReceivingVideoDataOn:
        cout << "ROS : videostream start req : wrong state" << endl;
        break;
    default:
        break;
    }
}

void StreamInteractor::on_tradrInterface_videostreamStopRequested(const uav_videostream_msgs::VideostreamStopReqConstPtr& msg)
{
    switch (state) {
    case Uav::StreamState::STREAMSTATE_Init:
        cout << "ROS : videostream stop req : wrong state" << endl;
        break;
    case Uav::StreamState::STREAMSTATE_ReceivingVideoDataOff:
        cout << "ROS : videostream stop req : wrong state" << endl;
        break;
    case Uav::StreamState::STREAMSTATE_WaitForResponse:
        cout << "ROS : videostream stop req : wrong state" << endl;
        break;
    case Uav::StreamState::STREAMSTATE_ReceivingVideoDataOn:
        this->sendStopRequestToUav(msg);
        break;
    default:
        break;
    }
}

void StreamInteractor::sendStartRequestToUav(const uav_videostream_msgs::VideostreamStartReqConstPtr& msg)
{
    tradr::ResolutionMsg* videoResolution = new tradr::ResolutionMsg();
    videoResolution->set_height(msg->videoFormat.height);
    videoResolution->set_width(msg->videoFormat.width);

    tradr::VideoOutputFormatMsg* videoFormat = new tradr::VideoOutputFormatMsg();
    videoFormat->set_videotype(tradr::VIDEO_TYPE_H264);
    videoFormat->set_fps(msg->videoFormat.fps);
    videoFormat->set_allocated_resolution(videoResolution);

    tradr::VideostreamStartReqMsg* reqMsg = new tradr::VideostreamStartReqMsg();
    reqMsg->set_assetid(this->uav->getID());
    reqMsg->set_streamrecipientaddress("192.168.29.7");
    reqMsg->set_streamrecipientport("10000");
    reqMsg->set_allocated_streamoutputformat(videoFormat);

    this->uavInterface->sendStartVideotreamReqMsgToUav(reqMsg);

    this->state = Uav::StreamState::STREAMSTATE_WaitForResponse;
    this->uav->changeStreamState(Uav::StreamState::STREAMSTATE_WaitForResponse);
    this->stateChangedEvent.triggerEvent();
}

void StreamInteractor::sendStopRequestToUav(const uav_videostream_msgs::VideostreamStopReqConstPtr& msg)
{
    tradr::VideostreamStopReqMsg* stopMsg = new tradr::VideostreamStopReqMsg();
    stopMsg->set_stopid(0);

    this->uavInterface->sendStopVideotreamReqMsgToUav(stopMsg);

    this->state = Uav::StreamState::STREAMSTATE_ReceivingVideoDataOff;
    this->uav->changeStreamState(Uav::StreamState::STREAMSTATE_ReceivingVideoDataOff);
    this->stateChangedEvent.triggerEvent();
}

void StreamInteractor::switchVideoReceivingOn(const tradr::VideostreamStartAckMsg& msg)
{
    /*
    if (msg.iframeinjection())
    {
        this->iframe.assign(msg.iframedata().begin(), msg.iframedata().end());
        this->iframeNecessary = true;
    }
    else
    {
        this->iframeNecessary = false;
    }
    this->state = STATE_ReceivingVideoDataOn;
    */

    uav_videostream_msgs::VideostreamStartRsp rspMsg;

    if (msg.streamoutputformat().videotype() == tradr::VideoType::VIDEO_TYPE_H264)
    {
        rspMsg.isTransportStream = false;

        if (msg.iframeinjection())
        {
            rspMsg.iframe.assign(msg.iframedata().begin(), msg.iframedata().end());
            rspMsg.iframeNecessary = true;
        }
        else
        {
            rspMsg.iframeNecessary = false;
        }
    }
    else
    {
        rspMsg.isTransportStream = true;

        rspMsg.iframeNecessary = false;
    }


    rspMsg.videoFormat.fps = msg.streamoutputformat().fps();
    rspMsg.videoFormat.height = msg.streamoutputformat().resolution().height();
    rspMsg.videoFormat.width = msg.streamoutputformat().resolution().width();

    this->ocuInterface->ocu->videostream->sendStartResponse(rspMsg);

    this->state = Uav::StreamState::STREAMSTATE_ReceivingVideoDataOn;
    this->uav->changeStreamState(Uav::StreamState::STREAMSTATE_ReceivingVideoDataOn);
    this->stateChangedEvent.triggerEvent();
}

void StreamInteractor::processVideoData(const tradr::VideostreamDataMsg& msg)
{
    static unsigned long i = 0;
    //static unsigned int n = 0;

    uav_videostream_msgs::VideostreamData dataMsg;

    /*
    if (n > 1000)
    {
        n = 0;
    }
    else if (n == 0)
    {
        dataMsg.iframe.assign(this->iframe.begin(), this->iframe.end());
    }
    n++;
    */

    //dataMsg.iframeNecessary = this->iframeNecessary;
    //dataMsg.data = std::vector(msg.data().data(), msg.data().data() + msg.size());
    //std::copy(msg.data().begin(), msg.data().end(), std::back_inserter(dataMsg.data));
    dataMsg.data.assign(msg.data().begin(), msg.data().end());
    dataMsg.size = msg.size();
    dataMsg.number = i;
    this->ocuInterface->ocu->videostream->updateFrame(dataMsg);

    i++;

}


