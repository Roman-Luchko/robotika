package mysensorproject;

network SensorDroneNetwork
{
    submodules:
        sensor[5]: Sensor;
        drone: Drone {
            gates:
                in[5];  // ← Размер входного шлюза = количеству сенсоров
        }

    connections allowunconnected:
        for i=0..4 {
            sensor[i].out --> drone.in[i];
        }
}
