#ifndef UAV_H
#define UAV_H

#include <string>

#include <iostream>

#include "utils/event.h"

using namespace std;

class Uav
{
public:
    enum MainState
    {
        MAINSTATE_Init,
        MAINSTATE_WaitForRequest,
        MAINSTATE_Registered,
        //TRANSITION_Idle_TO_WaitForRequest,
        //TRANSITION_WaitForRequest_TO_Registered
    };

    enum StatusState
    {
        STATUSSTATE_Init,
        STATUSSTATE_UavMonitoring
    };

    enum StreamState
    {
        STREAMSTATE_Init,
        STREAMSTATE_ReceivingVideoDataOff,
        STREAMSTATE_WaitForResponse,
        STREAMSTATE_ReceivingVideoDataOn,
        //TRANSITION_Idle_TO_WaitForRequest,
        //TRANSITION_WaitForRequest_TO_Registered
    };

    enum TaskState
    {
        TASKSTATE_Init,
        TASKSTATE_WaitForTask,
        TASKSTATE_WaitForResponse,
        TASKSTATE_Execution
    };


public:
    Event<Uav, Uav*> changedEvent;

private:
    unsigned int id;
    string name;
    string model;
    string guid;

    MainState mainState;
    StatusState statusState;
    StreamState streamState;
    TaskState taskState;

public:
    Uav(unsigned int id);

    void registerUAV(string name, string model, string guid);

    unsigned int getID();
    string getName();
    string getModel();
    string getGUID();

    void changeMainState(MainState newState);
    void changeStatusState(StatusState newState);
    void changeStreamState(StreamState newState);
    void changeTaskState(TaskState newState);

    MainState getMainState();
    StatusState getStatusState();
    StreamState getStreamState();
    TaskState getTaskState();
};

#endif // UAV_H
