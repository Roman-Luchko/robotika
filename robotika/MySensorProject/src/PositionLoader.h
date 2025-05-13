#ifndef POSITION_LOADER_H
#define POSITION_LOADER_H

#include <string>
#include <map>
#include <utility>

class PositionLoader
{
  private:
    std::map<int, std::pair<double, double>> sensorPositions;
    std::pair<double, double> dronePosition;
    static PositionLoader* instance;

    PositionLoader();  // private constructor

  public:
    static PositionLoader* getInstance();
    std::pair<double, double> getSensorPosition(int index);
    std::pair<double, double> getDronePosition();
};

#endif
