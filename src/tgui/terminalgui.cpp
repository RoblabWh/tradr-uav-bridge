#include "terminalgui.h"


/*
The codes for foreground and background colours are:

         foreground background
black        30         40
red          31         41
green        32         42
yellow       33         43
blue         34         44
magenta      35         45
cyan         36         46
white        37         47

Additionally, you can use these:

reset             0  (everything back to normal)
bold/bright       1  (often a brighter shade of the same colour)
underline         4
inverse           7  (swap foreground and background colours)
bold/bright off  21
underline off    24
inverse off      27

*/


TerminalGUI::TerminalGUI()
{
    if ((mkfifo("/tmp/uav_bridge_display.pipe", S_IRUSR | S_IWUSR)) == -1)
    {
        if(errno == EEXIST)
        {
            printf("/tmp/uav_bridge_display.pipe already exists...\n");
            fflush(stdout);
        }
        else
        {
            perror("/tmp/uav_bridge_display.pipe could not be created...\n");
            fflush(stderr);
            exit(EXIT_FAILURE);
        }
    }

    this->display = fopen("/tmp/uav_bridge_display.pipe", "w");
    if (this->display == NULL)
    {
        perror("/tmp/uav_bridge_display.pipe could not be opened...\n");
        fflush(stderr);
        exit(EXIT_FAILURE);
    }

    this->refresh();

    fprintf(this->display, "\033[2J");
    fprintf(this->display, "\033[1;1H");
    fprintf(this->display, "\033[1;47;1;31m");
    fprintf(this->display, "                                                                     \n");
    fprintf(this->display, "   T R A D R  -  U A V   G r o u n d  C o n t r o l  S t a t i o n   \n");
    fprintf(this->display, "                                                                     \n");
    fprintf(this->display, "\033[0m");
    fflush(this->display);

    this->refresh();
}

TerminalGUI::~TerminalGUI()
{
    fclose(this->display);

    for (UavEntry* entry : this->uavList)
    {
        delete entry;
    }
    this->uavList.clear();
}

void TerminalGUI::removeUavEntry(Uav* uav)
{
    int index = -1;
    for (int i = 0; i < this->uavList.size(); i++)
    {
        if (this->uavList[i]->getUav() == uav) {
            index = i;
            break;
        }
    }

    if (index >= 0)
    {
        unsigned int position = this->uavList[index]->getPosition();
        this->uavList[index]->erase();
        this->uavList.erase(this->uavList.begin() + index);
        for (int i = index; i < this->uavList.size(); i++)
        {
            unsigned int tmp = this->uavList[i]->getPosition();
            this->uavList[i]->erase();
            this->uavList[i]->setPosition(position);
            this->uavList[i]->print();
            position = tmp;
        }
    }
}

void TerminalGUI::refresh()
{
    fprintf(this->display, "\033[10;1H");
    fprintf(this->display, "\n");
    fflush(this->display);
}

void TerminalGUI::addUavEntry(Uav* uav)
{
    unsigned int position = 5 + this->uavList.size() * 3;
    this->uavList.push_back(new UavEntry(this->display, uav, position));
}

