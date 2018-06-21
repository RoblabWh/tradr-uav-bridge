#include "uav.h"

Uav::Uav(unsigned int id)
{
    this->id = id;

    this->mainState = MainState::MAINSTATE_Init;
    this->statusState = StatusState::STATUSSTATE_Init;
    this->streamState = StreamState::STREAMSTATE_Init;
    this->taskState = TaskState::TASKSTATE_Init;

    this->changedEvent.triggerEvent(this);
}

void Uav::registerUAV(string name, string model, string guid)
{
    this->name = name;
    this->model = model;
    this->guid = guid;

    this->changedEvent.triggerEvent(this);
}

unsigned int Uav::getID()
{
    return this->id;
}

string Uav::getName()
{
    return this->name;
}

string Uav::getModel()
{
    return this->model;
}

string Uav::getGUID()
{
    return this->guid;
}

void Uav::changeMainState(Uav::MainState newState)
{
    this->mainState = newState;
    this->changedEvent.triggerEvent(this);
}

void Uav::changeStatusState(Uav::StatusState newState)
{
    this->statusState = newState;
    this->changedEvent.triggerEvent(this);
}

void Uav::changeStreamState(Uav::StreamState newState)
{
    this->streamState = newState;
    this->changedEvent.triggerEvent(this);
}

void Uav::changeTaskState(Uav::TaskState newState)
{
    this->taskState = newState;
    this->changedEvent.triggerEvent(this);
}

Uav::MainState Uav::getMainState()
{
    return this->mainState;
}

Uav::StatusState Uav::getStatusState()
{
    return this->statusState;
}

Uav::StreamState Uav::getStreamState()
{
    return this->streamState;
}

Uav::TaskState Uav::getTaskState()
{
    return this->taskState;
}



