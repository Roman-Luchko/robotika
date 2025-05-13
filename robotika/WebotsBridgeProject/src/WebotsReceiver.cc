#include <omnetpp.h>
#include "D:/omnetpp-6.1/inet4.5/src/inet/applications/base/ApplicationBase.h"
#include "D:/omnetpp-6.1/inet4.5/src/inet/common/packet/Packet.h"
#include "D:/omnetpp-6.1/inet4.5/src/inet/common/INETUtils.h"
#include "D:/omnetpp-6.1/inet4.5/src/inet/common/socket/SocketMap.h"
#include "D:/omnetpp-6.1/inet4.5/src/inet/transportlayer/contract/udp/UdpSocket.h"
#include "D:/omnetpp-6.1/inet4.5/src/inet/common/precompiled_debug.h"


#include "D:/omnetpp-6.1/inet4.5/src/inet/common/INETDefs.h"

using namespace omnetpp;
using namespace inet;

class WebotsReceiver : public ApplicationBase, public UdpSocket::ICallback
{
  protected:
    UdpSocket socket;
    int localPort = -1;

  protected:
    virtual void initialize(int stage) override {
        ApplicationBase::initialize(stage);
        if (stage == INITSTAGE_LOCAL) {
            localPort = par("localPort");
        }
        else if (stage == INITSTAGE_APPLICATION_LAYER) {
            socket.setOutputGate(gate("socketOut"));
            socket.bind(localPort);
            socket.setCallback(this);
        }
    }

    virtual void handleMessageWhenUp(cMessage *msg) override {
        socket.processMessage(msg);
    }

    virtual void socketDataArrived(UdpSocket *socket, Packet *packet) override {
        EV << "Received packet from Webots: " << packet->getByteLength() << " bytes\n";
        delete packet;
    }

    virtual void socketClosed(UdpSocket *socket) override {}
    virtual void socketErrorArrived(UdpSocket *socket, Indication *indication) override {}
    virtual void socketAvailable(UdpSocket *socket, Indication *indication) override {}

    virtual void finish() override {}
    virtual void handleStartOperation(LifecycleOperation *operation) override {}
    virtual void handleStopOperation(LifecycleOperation *operation) override {}
    virtual void handleCrashOperation(LifecycleOperation *operation) override {}
};

Define_Module(WebotsReceiver);
