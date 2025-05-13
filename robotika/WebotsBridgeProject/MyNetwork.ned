package WebotsBridgeProject;

import inet.node.inet.StandardHost;

network MyNetwork
{
    submodules:
        host1: StandardHost;
        host2: StandardHost;
    connections allowunconnected:
        host1.pppg++ <--> pointToPointLink <--> host2.pppg++;
}
