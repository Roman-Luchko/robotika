#include "Drone.h"
#include "PositionLoader.h"

Define_Module(Drone);

void Drone::initialize()
{
    pos = PositionLoader::getInstance()->getDronePosition();
}

void Drone::handleMessage(cMessage *msg)
{
    EV << "Drone received data at position (" << pos.first << ", " << pos.second << ")\n";
    delete msg;
}
