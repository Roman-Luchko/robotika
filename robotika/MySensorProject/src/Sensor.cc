#include "Sensor.h"
#include "PositionLoader.h"
#include <cmath>
#include <fstream>

Define_Module(Sensor);

std::ofstream Sensor::outFile;

void Sensor::initialize()
{
    int index = getIndex();
    pos = PositionLoader::getInstance()->getSensorPosition(index);
    sendInterval = par("sendInterval");

    // Позиция DCC из параметров
    dccPos.first = par("dccX").doubleValue();
    dccPos.second = par("dccY").doubleValue();

    EV << "Sensor " << index << " initialized at (" << pos.first << ", " << pos.second << ").\n";

    if (!outFile.is_open()) {
        outFile.open("transmission_times.txt", std::ios::out | std::ios::app);
        if (outFile.is_open()) {
            outFile << "Sensor Transmission Log\n";
            outFile << "Sensor Index, Distance (m), Delay (s)\n";
        } else {
            EV << "Error: Unable to open the file.\n";
        }
    }

    scheduleAt(0, new omnetpp::cMessage("sendData"));
}

void Sensor::handleMessage(omnetpp::cMessage *msg)
{
    if (strcmp(msg->getName(), "sendData") == 0) {
        omnetpp::simtime_t now = omnetpp::simTime();
        EV << "Sensor " << getIndex() << " preparing data.\n";

        // Создаем сообщение
        omnetpp::cMessage *data = new omnetpp::cMessage("sensorData");

        // Вычисляем расстояние
        double dx = pos.first - dccPos.first;
        double dy = pos.second - dccPos.second;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Скорость света
        double c = 3e8;
        omnetpp::simtime_t delay = distance / c;

        EV << "Distance to DCC: " << distance << " m, delay: " << delay << " s\n";

        // Отправляем с задержкой
        sendDelayed(data, delay, "out");

        // Логируем в файл
        if (outFile.is_open()) {
            outFile << getIndex() << ", " << distance << ", " << delay << "\n";
        }

        // Планируем следующую отправку
        scheduleAt(now + sendInterval, msg);
    } else {
        delete msg;
    }
}

void Sensor::finish()
{
    if (outFile.is_open()) {
        outFile.close();
    }
}
