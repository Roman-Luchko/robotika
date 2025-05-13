import WebotsBridgeProject.WebotsReceiver;

network MyNetwork
{
    submodules:
        host1: StandardHost;
        host2: StandardHost;
        webotsReceiver: WebotsReceiver {
            parameters:
                localPort = 6000;
        }
    connections allowunconnected:
        host1.pppg++ <--> pointToPointLink <--> host2.pppg++;
}
