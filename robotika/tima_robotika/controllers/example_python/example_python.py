import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from scipy.spatial.distance import cdist

import numpy as np
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

class UAVPathPlanning:
    def __init__(self, area_size, uav_altitude, coverage_radius, dcc_position, uav_speed, 
                 hover_power, fly_power, battery_capacity):
        self.area_size = area_size
        self.uav_altitude = uav_altitude
        self.coverage_radius = coverage_radius
        self.dcc_position = np.array(dcc_position)
        self.uav_speed = uav_speed
        self.hover_power = hover_power
        self.fly_power = fly_power
        self.battery_capacity = battery_capacity

    def find_hover_positions(self, sensor_positions):
        uncovered_sensors = np.array(sensor_positions)
        hover_positions = []

        while len(uncovered_sensors) > 0:
            max_covered = 0
            best_position = None
            best_covered_indices = []

            for i, pos in enumerate(uncovered_sensors):
                distances = cdist([pos], uncovered_sensors)[0]
                covered_indices = np.where(distances <= self.coverage_radius)[0]

                if len(covered_indices) > max_covered:
                    max_covered = len(covered_indices)
                    best_position = pos
                    best_covered_indices = covered_indices

            if best_position is None:
                break

            covered_sensors = uncovered_sensors[best_covered_indices]
            centroid = np.mean(covered_sensors, axis=0)

            hover_positions.append(centroid)
            uncovered_sensors = np.delete(uncovered_sensors, best_covered_indices, axis=0)

        return np.array(hover_positions)

    def cluster_hover_positions(self, hover_positions, num_uavs):
        kmeans = KMeans(n_clusters=num_uavs, random_state=0).fit(hover_positions)
        clusters = {}

        for i, label in enumerate(kmeans.labels_):
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(hover_positions[i])

        return kmeans.cluster_centers_, clusters

    def plan_flight_paths(self, hover_positions, num_uavs):
        # Для всех дронов одновременно
        cluster_centers, clusters = self.cluster_hover_positions(hover_positions, num_uavs)
        paths = []
        for cluster_id in clusters:
            cluster_positions = np.array(clusters[cluster_id])
            current_position = self.dcc_position
            path = [current_position]
            remaining_positions = cluster_positions

            while len(remaining_positions) > 0:
                distances = cdist([current_position], remaining_positions)[0]
                nearest_idx = np.argmin(distances)
                nearest_position = remaining_positions[nearest_idx]

                path.append(nearest_position)
                current_position = nearest_position
                remaining_positions = np.delete(remaining_positions, nearest_idx, axis=0)

            path.append(self.dcc_position)  # Возвращаемся в DCC
            paths.append(np.array(path))

        return paths

    def calculate_energy_consumption(self, paths, hover_time_per_position=10):
        total_energy = 0
        for path in paths:
            flight_distance = 0
            for i in range(len(path)-1):
                flight_distance += np.linalg.norm(path[i+1] - path[i])

            flight_time = flight_distance / self.uav_speed
            hover_time = (len(path) - 2) * hover_time_per_position

            energy = (self.fly_power * flight_time + self.hover_power * hover_time) / 3.6
            total_energy += energy

        return total_energy

    def optimize_uav_deployment(self, sensor_positions, max_uavs=5):
        hover_positions = self.find_hover_positions(sensor_positions)
        last_invalid_paths = []
        last_invalid_hover_positions = hover_positions
        last_invalid_uav_count = max_uavs

        for num_uavs in range(1, max_uavs + 1):
            paths = self.plan_flight_paths(hover_positions, num_uavs)
            total_energy = self.calculate_energy_consumption(paths)

            if total_energy <= self.battery_capacity:
                print(f"Найдено валидное развертывание с {num_uavs} UAV")
                print(f"Общее энергопотребление: {total_energy:.2f} мАч")
                return num_uavs, paths, hover_positions
            else:
                last_invalid_paths = paths
                last_invalid_hover_positions = hover_positions
                last_invalid_uav_count = num_uavs

        print("❌ Не удалось найти валидное развертывание с заданным количеством UAV")

        return last_invalid_uav_count, last_invalid_paths, last_invalid_hover_positions

    def visualize_deployment(self, sensor_positions, hover_positions, paths):
        plt.figure(figsize=(10, 10))

        # Отображаем сенсоры
        plt.scatter(sensor_positions[:, 0], sensor_positions[:, 1], c='blue', label='Сенсоры')

        # Отображаем позиции зависания
        plt.scatter(hover_positions[:, 0], hover_positions[:, 1], c='red', marker='D', label='Позиции зависания')

        # Отображаем DCC
        plt.scatter(self.dcc_position[0], self.dcc_position[1], c='green', marker='*', s=200, label='DCC')

        # Отображаем пути UAV
        colors = ['purple', 'orange', 'brown', 'pink', 'gray']
        for i, path in enumerate(paths):
            color = colors[i % len(colors)]
            if path is not None and len(path) > 1:
                plt.plot(path[:, 0], path[:, 1], '--', color=color, linewidth=2, label=f'UAV {i+1} путь')
                if len(path) > 2:
                    plt.scatter(path[1:-1, 0], path[1:-1, 1], color=color, marker='o')

        # Отображаем зоны покрытия
        for pos in hover_positions:
            circle = plt.Circle((pos[0], pos[1]), self.coverage_radius, color='gray', alpha=0.1)
            plt.gca().add_patch(circle)

        plt.xlim(0, self.area_size[0])
        plt.ylim(0, self.area_size[1])
        plt.title('Развертывание UAV для сбора данных')
        plt.xlabel('X координата (м)')
        plt.ylabel('Y координата (м)')
        plt.legend()
        plt.grid(True)
        plt.show()

# Пример использования:
#area_size = (1000, 1000)
#dcc_position = [500, 500]
#uav_speed = 5  # м/с
#hover_power = 10  # Вт
#fly_power = 50  # Вт
#battery_capacity = 10000  # мАч
#coverage_radius = 100  # м

#sensor_positions = np.random.rand(20, 2) * 1000  # случайные позиции сенсоров

#uav_planner = UAVPathPlanning(area_size, 100, coverage_radius, dcc_position, uav_speed, hover_power, fly_power, battery_capacity)
#num_uavs, paths, hover_positions = uav_planner.optimize_uav_deployment(sensor_positions, max_uavs=5)
#uav_planner.visualize_deployment(sensor_positions, hover_positions, paths)











































from controller import Robot, Camera, Compass, GPS, Gyro, InertialUnit, Keyboard, LED, Motor
import math
import struct

def sign(x):
    return (x > 0) - (x < 0)

def clamp(value, low, high):
    return max(low, min(value, high))

def initialize_robot():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    return robot, timestep

def initialize_devices(robot, timestep):
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)
    receiver.setChannel(1)  # Устанавливаем канал для ресивера

    camera = robot.getDevice("camera")
    camera.enable(timestep)
    front_left_led = robot.getDevice("front left led")
    front_right_led = robot.getDevice("front right led")
    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    keyboard = Keyboard()
    keyboard.enable(timestep)
    camera_roll_motor = robot.getDevice("camera roll")
    camera_pitch_motor = robot.getDevice("camera pitch")
    motors = [
        robot.getDevice("front left propeller"),
        robot.getDevice("front right propeller"),
        robot.getDevice("rear left propeller"),
        robot.getDevice("rear right propeller")
    ]
    for motor in motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(1.0)
    return receiver,camera, front_left_led, front_right_led, imu, gps, compass, gyro, keyboard, camera_roll_motor, camera_pitch_motor, motors

def wait_for_start(robot, timestep):
    while robot.step(timestep) != -1:
        if robot.getTime() > 1.0:
            break

def print_controls():
    print("You can control the drone with your computer keyboard:")
    print("- 'up': move forward.")
    print("- 'down': move backward.")
    print("- 'right': turn right.")
    print("- 'left': turn left.")
    print("- 'shift + up': increase the target altitude.")
    print("- 'shift + down': decrease the target altitude.")
    print("- 'shift + right': strafe right.")
    print("- 'shift + left': strafe left.")

def update_leds(front_left_led, front_right_led, time):
    led_state = int(time) % 2
    front_left_led.set(led_state)
    front_right_led.set(not led_state)

def stabilize_camera(camera_roll_motor, camera_pitch_motor, roll_velocity, pitch_velocity):
    camera_roll_motor.setPosition(-0.115 * roll_velocity)
    camera_pitch_motor.setPosition(-0.1 * pitch_velocity)

def process_keyboard_input(keyboard, target_altitude):
    roll_disturbance = 0.0
    pitch_disturbance = 0.0
    yaw_disturbance = 0.0
    key = keyboard.getKey()
    while key > 0:
        if key == Keyboard.UP:
            pitch_disturbance = -2.0
        elif key == Keyboard.DOWN:
            pitch_disturbance = 2.0
        elif key == Keyboard.RIGHT:
            yaw_disturbance = -1.3
        elif key == Keyboard.LEFT:
            yaw_disturbance = 1.3
        elif key == (Keyboard.SHIFT + Keyboard.RIGHT):
            roll_disturbance = -1.0
        elif key == (Keyboard.SHIFT + Keyboard.LEFT):
            roll_disturbance = 1.0
        elif key == (Keyboard.SHIFT + Keyboard.UP):
            target_altitude += 0.05
            print(f"target altitude: {target_altitude} [m]")
        elif key == (Keyboard.SHIFT + Keyboard.DOWN):
            target_altitude -= 0.05
            print(f"target altitude: {target_altitude} [m]")
        key = keyboard.getKey()
    return roll_disturbance, pitch_disturbance, yaw_disturbance, target_altitude

def compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude):
    k_vertical_thrust = 68.5
    k_vertical_offset = 0.6
    k_vertical_p = 3.0
    k_roll_p = 50.0
    k_pitch_p = 30.0
    
    roll_input = k_roll_p * clamp(roll, -1.0, 1.0) + roll_velocity
    pitch_input = k_pitch_p * clamp(pitch, -1.0, 1.0) + pitch_velocity
    clamped_difference_altitude = clamp(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
    vertical_input = k_vertical_p * (clamped_difference_altitude ** 3)
    return roll_input, pitch_input, vertical_input

def update_motors(motors, roll_input, pitch_input, vertical_input, yaw_input):
    k_vertical_thrust = 68.5
    motors[0].setVelocity(k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input)
    motors[1].setVelocity(- (k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input))
    motors[2].setVelocity(- (k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input))
    motors[3].setVelocity(k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input)

def move(direction):
    # pitch, roll
    # left, forward, right, back
    if direction == 1:
        return 0.0, 1.0
    elif direction == 2:
        return -2.0, 0.0
    elif direction == 3:
        return 0.0, -1.0
    elif direction == 4:
        return 2.0, 0.0

def update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance):
    roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
    update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)

def move_bod(x1, y1, x2, y2, x_current, y_current, altitude, target_altitude, roll, pitch, yaw, roll_velocity, pitch_velocity, motors, yaw_disturbance, roll_disturbance, pitch_disturbance):
    start = [x1, y1]
    end = [x2, y2]
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    yaw_luboj = math.atan2(delta_y, delta_x)
    
    if yaw < yaw_luboj - tolerance or yaw > yaw_luboj + tolerance:
        print("menayu uhol pre move_bod")
        print(yaw)
        yaw_disturbance = 0.2
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
    elif x2 > x_current and y2 > y_current:
        print("vpered pre move_bod")
        pitch_disturbance, roll_disturbance = move(2)
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
    else:
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
     
def go_to_x(x_target, y_target, x, y, altitude, target_altitude, roll, pitch, yaw, roll_velocity, pitch_velocity, motors, yaw_disturbance, roll_disturbance, pitch_disturbance):
    # Проверка на отклонение по углу yaw относительно target_yaw
    if (yaw < target[0] - tolerance or yaw > target[0] + tolerance) and x > x_target:
        print("menayu uhol pre x")
        print(yaw)
        yaw_disturbance = 0.2
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
    
    # Если x меньше чем целевой x_target, продолжаем движение
    elif x < x_target:
        print("Vse rabotaet pre x")
        print(x)
        pitch_disturbance, roll_disturbance = move(2)
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
    
    # Если x уже достиг целевого, завершаем выполнение
    else:
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
        return 1
    return 0


    
def go_to_y(x_target, y_target, x, y, altitude, target_altitude, roll, pitch, yaw, roll_velocity, pitch_velocity, motors, yaw_disturbance, roll_disturbance, pitch_disturbance):
    # Проверка на отклонение по углу yaw относительно target_way_y
    if (yaw < target[1] - tolerance or yaw > target[1] + tolerance) and y < y_target:
        print("menayu uhol pre y")
        print(yaw)
        yaw_disturbance = 0.2
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
    
    # Если y меньше чем целевой y_target, продолжаем движение
    elif y < y_target:
        print("Vse rabotaet pre y")
        print(y)
        pitch_disturbance, roll_disturbance = move(2)
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
    
    # Если y уже достиг целевого, завершаем выполнение
    else:
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
        return 1
    return 0


def zmejka(x_target, y_target, x, y, altitude, target_altitude, roll, pitch, yaw, roll_velocity, pitch_velocity, motors, yaw_disturbance, roll_disturbance, pitch_disturbance):
    if x < 0:
        return 1, 1
    elif (yaw < target[4] - tolerance or yaw > target[4] + tolerance) and y > target[3] and pravda[1] == 1:
        print(f"menayu uhol pre zmejka nalevo {pravda[1]}")
        print(yaw)
        yaw_disturbance = 0.2
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
        return 0, 1
    elif pravda[2] == 1 and pravda[7] is not None and x > pravda[7] and x > 0:
        print("Vse rabotaet pre zmejka vpered")
        print(f"{x} {pravda[7]}")
        pitch_disturbance, roll_disturbance = move(2)
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
        return 0, 1
    elif pravda[3] == 1 and pravda[3] is not None and (yaw < target[2] - tolerance or yaw > target[2] + tolerance):
        print("menayu uhol pre zmejka vniz")
        print(yaw)
        yaw_disturbance = 0.2
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
        return 0, 1
    elif pravda[4] and pravda[6] == 0 and y > 1:
        print("Vse rabotaet pre zmejka vniz")
        print(f"{y}")
        pitch_disturbance, roll_disturbance = move(2)
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
        return 0, 1
    elif (yaw < target[1] - tolerance or yaw > target[1] + tolerance) and x < y_target and pravda[5] == 1:
        print("menayu uhol pre zmejka last povorot")
        print(yaw)
        yaw_disturbance = 0.2
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
        return 0, 1
    elif pravda[6] == 1:
        print("Vse rabotaet pre zmejka nazad")
        print(y)
        pitch_disturbance, roll_disturbance = move(2)
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
        return 0, 1
    else:
        print("Stojat na meste")
        update_all(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude, motors, roll_disturbance, pitch_disturbance, yaw_disturbance)
        return 0, 0


    
def get_gps_from_others(receiver):
    gps_data = {}
    print(receiver.getQueueLength())
    while receiver.getQueueLength() > 0:
        message = receiver.getString()
        receiver.nextPacket()
        try:
            name_part, coords = message.split(':')
            x_str, y_str, z_str = coords.split(',')
            x, y, z = float(x_str), float(y_str), float(z_str)
            gps_data[name_part] = (x, y, z)
        except Exception as e:
            print("Ошибка разбора:", message, e)
    return gps_data

import asyncio
import websockets

# Функция для отправки данных на сервер OMNeT++
async def send_data_to_omnet(sensor_positions):
    uri = "ws://localhost:8080"  # Адрес WebSocket-сервера OMNeT++
    async with websockets.connect(uri) as websocket:
        # Преобразуем данные в строку, например, JSON или строковый формат
        data_to_send = str(sensor_positions)  # Преобразуем список координат в строку
        await websocket.send(data_to_send)
        print(f"Sent to OMNeT++: {data_to_send}")




















    
target = [
    0,                 # target_yaw
    math.pi / 2,       # target_way_y
    -math.pi / 2,      # target_vniz
    0,                 # target_zmejka
    math.pi,           # target_povorot_zmejka
    10,                # x_target
    10                 # y_target
]

pravda = [
    0,  # zmejka_def_pravda
    1,  # pravda_povorot
    1,  # pravda_vpered
    0,  # pravda_vniz
    0,  # pravda_move_vniz
    0,  # pravda_nazad
    0,  # pravda_last_povorot
    None, # pravda_x
    0,  # pravda_x_start
    0,  # pravda_y_start
    0   # pravda_poisk_dorogi_done
]

stop = 1
tolerance = 0.4   
east = 0
sds = []


# Глобальные параметры
area_size = (30, 30)  # Размер области
uav_altitude = 2        # Высота полета UAV
coverage_radius = 8     # Радиус покрытия
dcc_position = [0, 0]  # Позиция DCC
uav_speed = 8             # Скорость UAV
hover_power = 100         # Мощность при зависании
fly_power = 200           # Мощность при полете
battery_capacity = 5000   # Емкость батареи
max_uavs_to_test = 1      # Максимальное количество UAV для тестирования
sensor_positions = []



def main():
    global pravda
    global sensor_positions
    sensor_positions = []  # Явная инициализация

    robot, timestep = initialize_robot()
    devices = initialize_devices(robot, timestep)
    receiver, camera, front_left_led, front_right_led, imu, gps, compass, gyro, keyboard, camera_roll_motor, camera_pitch_motor, motors = devices

    print("Start the drone...")
    wait_for_start(robot, timestep)
    print_controls()

    target_altitude = 1.0

    # Получаем GPS координаты от других роботов
    data = get_gps_from_others(receiver)
    if data:
        print("Полученные GPS-координаты от e-puck’ов:", data)
        for name, coords in data.items():
            try:
                x, y, z = coords
                sensor_positions.append([x, y])  # Использовать [x, y, z], если нужно 3D
            except Exception as e:
                print(f"Ошибка при обработке данных '{name}': {e}")
    else:
        print("Я не нашел сенсоры")

    print("sensor_positions main", sensor_positions)
    print("sensor_positions len main", len(sensor_positions))

    print("Otpravka dat")
    # Отправляем данные на OMNeT++
    asyncio.get_event_loop().run_until_complete(send_data_to_omnet(sensor_positions))
    print("Data otpravleno")
    # Используем класс UAVPathPlanning для планирования развертывания
    try:
        uav_planner = UAVPathPlanning(
            area_size=area_size,
            uav_altitude=uav_altitude,
            coverage_radius=coverage_radius,
            dcc_position=dcc_position,
            uav_speed=uav_speed,
            hover_power=hover_power,
            fly_power=fly_power,
            battery_capacity=battery_capacity
        )

        num_uavs, paths, hover_positions = uav_planner.optimize_uav_deployment(sensor_positions, max_uavs=max_uavs_to_test)

        uav_planner.visualize_deployment(
            sensor_positions=np.array(sensor_positions),
            hover_positions=np.array(hover_positions),
            paths=[np.array(path) for path in paths]
        )

        pravda[10] = 1
    except Exception as e:
        print("Ошибка при вызове методов UAVPathPlanning:", e)




    
    while robot.step(timestep) == -1:
        time = robot.getTime()
        roll, pitch, yaw = imu.getRollPitchYaw()
        x, y, altitude = gps.getValues()
        roll_velocity, pitch_velocity, _ = gyro.getValues()

        update_leds(front_left_led, front_right_led, time)
        stabilize_camera(camera_roll_motor, camera_pitch_motor, roll_velocity, pitch_velocity)

        roll_disturbance, pitch_disturbance, yaw_disturbance = 0, 0, 0

        if pravda[8] == 0:
            pravda[8] = go_to_x(target[5], 0, x, y, altitude, target_altitude,
                                roll, pitch, yaw, roll_velocity, pitch_velocity,
                                motors, yaw_disturbance, roll_disturbance, pitch_disturbance)

        elif pravda[9] == 0:
            pravda[9] = go_to_y(0, target[6], x, y, altitude, target_altitude,
                                roll, pitch, yaw, roll_velocity, pitch_velocity,
                                motors, yaw_disturbance, roll_disturbance, pitch_disturbance)

        elif pravda[0] == 0:
            if pravda[1] == 1:
                pravda[0], pravda[1] = zmejka(target[5], target[6], x, y, altitude, target_altitude,
                                              roll, pitch, yaw, roll_velocity, pitch_velocity,
                                              motors, yaw_disturbance, roll_disturbance, pitch_disturbance)
                pravda[2] = 1
                pravda[7] = 0

            elif pravda[2] == 1:
                if pravda[7] is None:
                    print("srabotalo")
                    pravda[7] = x - 5
                pravda[0], pravda[2] = zmejka(target[5], target[6], x, y, altitude, target_altitude,
                                              roll, pitch, yaw, roll_velocity, pitch_velocity,
                                              motors, yaw_disturbance, roll_disturbance, pitch_disturbance)
                pravda[3] = 1

            elif pravda[3] == 1:
                pravda[0], pravda[3] = zmejka(target[5], target[6], x, y, altitude, target_altitude,
                                              roll, pitch, yaw, roll_velocity, pitch_velocity,
                                              motors, yaw_disturbance, roll_disturbance, pitch_disturbance)
                pravda[4] = 1

            elif pravda[4] == 1:
                pravda[0], pravda[4] = zmejka(target[5], target[6], x, y, altitude, target_altitude,
                                              roll, pitch, yaw, roll_velocity, pitch_velocity,
                                              motors, yaw_disturbance, roll_disturbance, pitch_disturbance)
                pravda[5] = 1

            elif pravda[5] == 1:
                pravda[0], pravda[5] = zmejka(target[5], target[6], x, y, altitude, target_altitude,
                                              roll, pitch, yaw, roll_velocity, pitch_velocity,
                                              motors, yaw_disturbance, roll_disturbance, pitch_disturbance)
                pravda[6] = 1

            elif pravda[6] == 1:
                pravda[0], pravda[6] = zmejka(target[5], target[6], x, y, altitude, target_altitude,
                                              roll, pitch, yaw, roll_velocity, pitch_velocity,
                                              motors, yaw_disturbance, roll_disturbance, pitch_disturbance)

            else:
                print(f"AXAXAXAXAX {pravda[3]}")

        else:
            print("Zakonceno")
            print(y)
            roll_input, pitch_input, vertical_input = compute_motor_inputs(
                roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
            update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance,
                          vertical_input, yaw_disturbance)

if __name__ == "__main__":
    main()

