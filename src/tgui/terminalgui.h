#ifndef TERMINALGUI_H
#define TERMINALGUI_H

#include "uaventry.h"
#include "uav/uav.h"
#include <list>
#include <vector>

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace std;

class TerminalGUI
{
private:
    int displayFD;
    FILE* display;              //Component (ressource)

    vector<UavEntry*> uavList;  //Component List


public:
    TerminalGUI();
    ~TerminalGUI();

    void addUavEntry(Uav* uav);

    void removeUavEntry(Uav* uav);

private:
    void refresh();

};

#endif // TERMINALGUI_H
