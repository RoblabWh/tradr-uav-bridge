#ifndef OCUCOMMUNICATIONINTERFACE_H
#define OCUCOMMUNICATIONINTERFACE_H

#include "tradr/ocuconnection.h"

class OCUCommunicationInterface
{
public:
    OCUConnection* ocu;     //shared
public:
    OCUCommunicationInterface(OCUConnection* ocuConnection);
};

#endif // OCUCOMMUNICATIONINTERFACE_H
