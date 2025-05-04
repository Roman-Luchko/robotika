from controller import Robot, GPS, Emitter

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Получаем устройство GPS
gps = robot.getDevice("gps")
gps.enable(timestep)

# Получаем устройство эмиттера
emitter = robot.getDevice("emitter")
emitter.setChannel(1)  # Канал передачи

# Ожидаем первого шага симуляции
robot.step(timestep)

# Получаем позицию один раз
gps_values = gps.getValues()
x, y, z = gps_values[0], gps_values[1], gps_values[2]

# Формируем сообщение с координатами
name = robot.getName()  # Имя робота (например, "sensor1")
message = f"{name}:{x},{y},{z}"

print(message)
# Отправляем сообщение один раз
emitter.send(message.encode("utf-8"))

# Больше ничего не делаем в цикле
