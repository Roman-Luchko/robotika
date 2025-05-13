#include "PositionLoader.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <filesystem>  // Для работы с файловой системой

PositionLoader* PositionLoader::instance = nullptr;

PositionLoader::PositionLoader()
{
    // Выводим текущую рабочую директорию
    std::cout << "Current directory: " << std::filesystem::current_path() << std::endl;

    std::ifstream file("positions.txt");

    if (!file.is_open()) {
        std::cerr << "Error: Failed to open positions.txt\n";
        throw std::runtime_error("Failed to open positions.txt");
    }

    std::string line;
    int lineNumber = 0;
    while (std::getline(file, line)) {
        lineNumber++;
        std::cout << "Reading line " << lineNumber << ": " << line << std::endl;

        if (line.empty() || line[0] == '#') {
            std::cout << "Skipping empty or comment line.\n";
            continue;
        }

        std::istringstream iss(line);
        std::string id;
        double x, y;

        if (!(iss >> id >> x >> y)) {
            std::cout << "Error parsing line " << lineNumber << ": " << line << std::endl;
            continue;  // Пропускаем строку, если не удается распарсить
        }

        std::cout << "Parsed line " << lineNumber << ": id=" << id << ", x=" << x << ", y=" << y << std::endl;

        if (id == "drone") {
            dronePosition = {x, y};
            std::cout << "Loaded drone at position (" << x << ", " << y << ")\n";  // Выводим в консоль
        } else {
            int sensorIndex = std::stoi(id);  // Преобразуем индекс в целое число
            sensorPositions[sensorIndex] = {x, y};
            std::cout << "Loaded sensor " << sensorIndex << " at position (" << x << ", " << y << ")\n";  // Выводим в консоль
        }
    }

    // Вывод всех загруженных сенсоров для отладки
    std::cout << "Loaded sensor positions:\n";
    for (const auto& sensor : sensorPositions) {
        std::cout << "Sensor " << sensor.first << " at position (" << sensor.second.first << ", " << sensor.second.second << ")\n";
    }
}

PositionLoader* PositionLoader::getInstance()
{
    if (!instance)
        instance = new PositionLoader();
    return instance;
}

std::pair<double, double> PositionLoader::getSensorPosition(int index)
{
    std::cout << "Looking for sensor with index: " << index << "\n"; // Отладочный вывод
    auto it = sensorPositions.find(index);
    if (it != sensorPositions.end()) {
        std::cout << "Found sensor " << index << " at position (" << it->second.first << ", " << it->second.second << ")\n";
        return it->second;
    } else {
        std::cout << "Error: Sensor index " << index << " not found\n";  // Отладочный вывод
        throw std::out_of_range("Sensor index out of range");
    }
}

std::pair<double, double> PositionLoader::getDronePosition()
{
    return dronePosition;
}
