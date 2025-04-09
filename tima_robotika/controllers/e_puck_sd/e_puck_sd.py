from controller import Robot, Emitter, GPS
import struct

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Активируем GPS для определения позиции
gps = robot.getDevice("gps")
gps.enable(timestep)

# Настраиваем передатчик
emitter = robot.getDevice("emitter")
emitter.setChannel(1)  # Общий канал для связи с дроном

while robot.step(timestep) != -1:
    # Получаем текущие координаты e-puck
    x, y, z = gps.getValues()
    
    # Упаковываем данные в байты (для надёжной передачи)
    data = struct.pack('fff', x, y, z)  # 3 числа float (x, y, z)
    
    # Отправляем данные дрону
    emitter.send(data)
