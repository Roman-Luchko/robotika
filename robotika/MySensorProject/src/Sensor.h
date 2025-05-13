#ifndef SENSOR_H
#define SENSOR_H

#include <omnetpp.h>
#include <utility>
#include <fstream>

class Sensor : public omnetpp::cSimpleModule
{
  private:
    std::pair<double, double> pos;         // Позиция сенсора
    std::pair<double, double> dccPos;      // Позиция DCC
    double sendInterval;                   // Интервал отправки данных
    static std::ofstream outFile;          // Файл для записи задержек
  protected:
    virtual void initialize() override;
    virtual void handleMessage(omnetpp::cMessage *msg) override;
    virtual void finish() override;
};

#endif
