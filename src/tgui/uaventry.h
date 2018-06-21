#ifndef UAVENTRY_H
#define UAVENTRY_H

#include <iostream>
#include <stdio.h>

#include "uav/uav.h"

class UavEntry
{
private:
    FILE* display;          //shared

    Uav* uav;               //shared
    unsigned int position;

public:
    UavEntry(FILE* display, Uav* uav, unsigned int position);

    void setPosition(unsigned int position);
    unsigned int getPosition();

    Uav* getUav();

    void print();
    void erase();

private:
    void on_uav_changed(Uav* uav);
    string getMainStateDescription(Uav::MainState mainState);
    string getStatusStateDescription(Uav::StatusState statusState);
    string getStreamStateDescription(Uav::StreamState streamState);
    string getTaskStateDescription(Uav::TaskState taskState);


};

#endif // UAVENTRY_H
