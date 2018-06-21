#ifndef STREAMINTERACTOR_H
#define STREAMINTERACTOR_H

#include "tradr_asset_message.pb.h"

#include "uav/uavcommunicationinterface.h"
#include "uav/ocucommunicationinterface.h"
#include "uav/uav.h"



class StreamInteractor
{

public:
    Event<StreamInteractor> stateChangedEvent;


private:
    Uav::StreamState state;

    Uav* uav;   //shared

    UavCommunicationInterface* uavInterface;    //shared
    OCUCommunicationInterface* ocuInterface;    //shared

    bool iframeNecessary;
    vector<char> iframe;




public:
    StreamInteractor(OCUCommunicationInterface* ocuCommInterface, UavCommunicationInterface* uavCommInterface, Uav* uav);

    void start();

    Uav::StreamState getState();

private:
    void on_uavInterface_startVideostreamAckMsgReceived(const tradr::VideostreamStartAckMsg& msg);
    void on_uavInterface_videostreamDataMsgReceived(const tradr::VideostreamDataMsg& msg);

    void on_tradrInterface_videostreamStartRequested(const uav_videostream_msgs::VideostreamStartReqConstPtr& msg);
    void on_tradrInterface_videostreamStopRequested(const uav_videostream_msgs::VideostreamStopReqConstPtr& msg);

    void sendStartRequestToUav(const uav_videostream_msgs::VideostreamStartReqConstPtr& msg);
    void sendStopRequestToUav(const uav_videostream_msgs::VideostreamStopReqConstPtr& msg);
    void switchVideoReceivingOn(const tradr::VideostreamStartAckMsg& msg);
    void processVideoData(const tradr::VideostreamDataMsg& msg);
};

#endif // STREAMINTERACTOR_H
