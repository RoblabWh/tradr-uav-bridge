#include "uaventry.h"

UavEntry::UavEntry(FILE* display, Uav* uav, unsigned int position)
{
    this->display = display;

    this->uav = uav;
    this->position = position;

    this->uav->changedEvent.registerEventHandler(this, &UavEntry::on_uav_changed);
}

void UavEntry::setPosition(unsigned int position)
{
    this->position = position;
}

unsigned int UavEntry::getPosition()
{
    return this->position;
}

Uav* UavEntry::getUav()
{
    return this->uav;
}

void UavEntry::print()
{
    fprintf(this->display, "\033[%d;1H", this->position);
    fprintf(this->display, "\033[21;41;37m");
    fprintf(this->display, "                                                                     \n");
    fprintf(this->display, "                                                                     \n");

    fprintf(this->display, "\033[%d;1H", this->position);
    fprintf(this->display, "\033[21;41;37m");
    fprintf(this->display, "%d\t%s\t%s\t%s\n", this->uav->getID(), this->uav->getName().c_str(), this->uav->getModel().c_str(), this->uav->getGUID().c_str());
    fprintf(this->display, "%s\t%s\t%s\t%s",
            getMainStateDescription(uav->getMainState()).c_str(),
            getStatusStateDescription(uav->getStatusState()).c_str(),
            getStreamStateDescription(uav->getStreamState()).c_str(),
            getTaskStateDescription(uav->getTaskState()).c_str());
    fprintf(this->display, "\033[0m");

    fprintf(this->display, "\033[10;1H");
    fprintf(this->display, "\n");
    fflush(this->display);
}

void UavEntry::erase()
{
    fprintf(this->display, "\033[%d;1H", this->position);
    fprintf(this->display, "\033[0m");
    fprintf(this->display, "                                                                     \n");
    fprintf(this->display, "                                                                     \n");
    fprintf(this->display, "\033[0m");

    fprintf(this->display, "\033[10;1H");
    fprintf(this->display, "\n");
    fflush(this->display);
}

void UavEntry::on_uav_changed(Uav *uav)
{
    this->print();
}

string UavEntry::getMainStateDescription(Uav::MainState mainState)
{
    return std::to_string((int) mainState);
}

string UavEntry::getStatusStateDescription(Uav::StatusState statusState)
{
    return std::to_string((int) statusState);
}

string UavEntry::getStreamStateDescription(Uav::StreamState streamState)
{
    return std::to_string((int) streamState);
}

string UavEntry::getTaskStateDescription(Uav::TaskState taskState)
{
    return std::to_string((int) taskState);
}
