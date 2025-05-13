package mysensorproject;

simple Sensor
{
    parameters:
        double sendInterval = default(1);  // Параметр интервала отправки (1 секунда)
    gates:
        output out;
}
