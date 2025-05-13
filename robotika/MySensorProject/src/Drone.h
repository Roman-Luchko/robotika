#ifndef DRONE_H
#define DRONE_H

#include <omnetpp.h>
#include <utility>

using namespace omnetpp;

class Drone : public cSimpleModule
{
  private:
    std::pair<double, double> pos;

  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
};

#endif
