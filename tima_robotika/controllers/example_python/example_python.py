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

def go_to_x(x_target,y_target,x,y,altitude,target_altitude,roll,pitch,yaw,roll_velocity, pitch_velocity,motors,yaw_disturbance,roll_disturbance,pitch_disturbance):
    if(yaw < target_yaw - tolerance or yaw > target_yaw + tolerance) and x > x_target:
        print("menayu uhol pre x")
        print(yaw)
        yaw_disturbance = 0.2
        roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
        update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
    elif x < x_target:
        print("Vse rabotaet pre x")
        print(x)
        pitch_disturbance,roll_disturbance = move(2)
        roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
        update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
    else:
        roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
        update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
        return 1
    return 0
    
def go_to_y(x_target,y_target,x,y,altitude,target_altitude,roll,pitch,yaw,roll_velocity, pitch_velocity,motors,yaw_disturbance,roll_disturbance,pitch_disturbance):
    #0.03 < -1.53 - 0.2 = -1.73 # 0.03 < -1.73
    #and
    #0.03 > -1.53 + 0.2 = -1.33 # 0.03 > -1.33 
    if(yaw < target_way_y - tolerance or yaw > target_way_y + tolerance) and y < y_target:
        print("menayu uhol pre y")
        print(yaw)
        yaw_disturbance = 0.2
        roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
        update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
    elif y < y_target:
        print("Vse rabotaet pre y")
        print(y)
        pitch_disturbance,roll_disturbance = move(2)
        roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
        update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
    else:
        roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
        update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
        return 1
    return 0

def zmejka(x_target,y_target,x,y,altitude,target_altitude,roll,pitch,yaw,roll_velocity, pitch_velocity,motors,yaw_disturbance,roll_disturbance,pitch_disturbance):    
    if(x < 0):
        return 1,1
    elif(yaw < target_povorot_zmejka - tolerance or yaw > target_povorot_zmejka + tolerance) and y > target_zmejka and pravda_povorot == 1:
        print(f"menayu uhol pre zmejka nalevo {pravda_povorot}")
        print(yaw)
        yaw_disturbance = 0.2
        roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
        update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
        return 0,1
    elif(pravda_vpered == 1 and pravda_x is not None and x > pravda_x and x > 0):
        print("Vse rabotaet pre zmejka vpered")
        print(f"{x} {pravda_x}")
        pitch_disturbance,roll_disturbance = move(2)
        roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
        update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
        return 0,1
    elif pravda_vniz == 1 and pravda_vniz is not None and (yaw < target_vniz - tolerance or yaw > target_vniz + tolerance):
        print("menayu uhol pre zmejka vniz")
        print(yaw)
        yaw_disturbance = 0.2
        roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
        update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
        return 0,1
    elif pravda_move_vniz and pravda_last_povorot == 0 and y > 1:
        print("Vse rabotaet pre zmejka vniz")
        print(f"{y}")
        pitch_disturbance,roll_disturbance = move(2)
        roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
        update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
        return 0,1 
    elif(yaw < target_way_y - tolerance or yaw > target_way_y + tolerance) and x < y_target and pravda_nazad == 1:
        print("menayu uhol pre zmejka last povorot")
        print(yaw)
        yaw_disturbance = 0.2
        roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
        update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
        return 0,1
    elif pravda_last_povorot == 1:
        print("Vse rabotaet pre zmejka nazad")
        print(y)
        pitch_disturbance,roll_disturbance = move(2)
        roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
        update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
        return 0,1
    else:
        print("Stojat na meste")
        roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
        update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
        return 0,0
    

def get_gps_from_others(receiver):
    gps_data_from_others = []  # Список для хранения GPS-данных
    if receiver.getQueueLength() > 0:
        while receiver.getQueueLength() > 0:
            # Получаем данные как строку и декодируем их в байты
            data = receiver.getString().encode('utf-8')  # Преобразуем строку в байты
            receiver.nextPacket()  # Переходим к следующему пакету данных

            # Попробуем распаковать данные как 'fff' (три float)
            try:
                x, y, z = struct.unpack('fff', data)  # Распаковка данных в 3 float
                gps_data_from_others.append((x, y, z))  # Сохраняем координаты
            except struct.error:
                print("Ошибка распаковки данных")

    print(gps_data_from_others)
    return gps_data_from_others

    
target_yaw = 0
target_way_y = math.pi/2
target_vniz = -math.pi/2
target_zmejka = 0
target_povorot_zmejka = math.pi

#peremenije dla def zmejka
zmejka_def_pravda = 0
pravda_povorot = 1
pravda_vpered = 0
pravda_vniz = 0
pravda_move_vniz = 0
pravda_nazad = 0
pravda_last_povorot = 0
pravda_x = None
stop = 1

tolerance = 0.4   
x_target = 10
y_target = 10
east = 0
pravda_x_start = 0  # Инициализация переменной
pravda_y_start = 0  # Инициализация переменной
sds = []
def main():
    global pravda_x_start  
    global pravda_y_start  
    global zmejka_def_pravda
    global pravda_povorot
    global x_current
    global pravda_vpered
    global pravda_x
    global pravda_vniz
    global pravda_move_vniz
    global pravda_nazad
    global last_povorot
    robot, timestep = initialize_robot()
    devices = initialize_devices(robot, timestep)
    receiver,camera, front_left_led, front_right_led, imu, gps, compass, gyro, keyboard, camera_roll_motor, camera_pitch_motor, motors = devices
    get_gps_from_others(receiver)
    print("Start the drone...")
    wait_for_start(robot, timestep)
    print_controls()
    
    target_altitude = 1.0

    while robot.step(timestep) != -1:
        time = robot.getTime()

        roll, pitch, yaw = imu.getRollPitchYaw()
        x, y, altitude = gps.getValues()
        roll_velocity, pitch_velocity, _ = gyro.getValues()
        update_leds(front_left_led, front_right_led, time)
        stabilize_camera(camera_roll_motor, camera_pitch_motor, roll_velocity, pitch_velocity)
        roll_disturbance, pitch_disturbance, yaw_disturbance = 0,0,0
        
        new_element = None
        new_element = get_gps_from_others(receiver)
        if new_element:
            sds.append(new_element)
            
        if time > 2.0:
            print(sds)
            break
        elif stop == 1:
            print("TY NE PROJDES")
            continue 
        if pravda_x_start == 0:
            pravda_x_start = go_to_x(x_target,0,x,y,altitude,target_altitude,roll,pitch,yaw,roll_velocity, pitch_velocity,motors,yaw_disturbance,roll_disturbance,pitch_disturbance)
        elif pravda_y_start == 0:
            pravda_y_start = go_to_y(0,y_target,x,y,altitude,target_altitude,roll,pitch,yaw,roll_velocity, pitch_velocity,motors,yaw_disturbance,roll_disturbance,pitch_disturbance)
        elif zmejka_def_pravda == 0:
            if pravda_povorot == 1:
                zmejka_def_pravda,pravda_povorot = zmejka(x_target,y_target,x,y,altitude,target_altitude,roll,pitch,yaw,roll_velocity, pitch_velocity,motors,yaw_disturbance,roll_disturbance,pitch_disturbance)
                pravda_vpered = 1
                pravda_last_povorot = 0
            elif pravda_vpered == 1:
                if pravda_x is None:
                    print("srabotalo")
                    pravda_x = x - 5
                zmejka_def_pravda,pravda_vpered = zmejka(x_target,y_target,x,y,altitude,target_altitude,roll,pitch,yaw,roll_velocity, pitch_velocity,motors,yaw_disturbance,roll_disturbance,pitch_disturbance)
                pravda_vniz = 1
            elif pravda_vniz == 1 :
                zmejka_def_pravda,pravda_vniz = zmejka(x_target,y_target,x,y,altitude,target_altitude,roll,pitch,yaw,roll_velocity, pitch_velocity,motors,yaw_disturbance,roll_disturbance,pitch_disturbance)
                pravda_move_vniz = 1
            elif pravda_move_vniz == 1:
                zmejka_def_pravda,pravda_move_vniz = zmejka(x_target,y_target,x,y,altitude,target_altitude,roll,pitch,yaw,roll_velocity, pitch_velocity,motors,yaw_disturbance,roll_disturbance,pitch_disturbance)
                pravda_nazad = 1
            elif pravda_nazad == 1:
                zmejka_def_pravda,pravda_nazad = zmejka(x_target,y_target,x,y,altitude,target_altitude,roll,pitch,yaw,roll_velocity, pitch_velocity,motors,yaw_disturbance,roll_disturbance,pitch_disturbance)    
                pravda_last_povorot = 1
            elif pravda_last_povorot == 1:
                
                zmejka_def_pravda,pravda_last_povorot = zmejka(x_target,y_target,x,y,altitude,target_altitude,roll,pitch,yaw,roll_velocity, pitch_velocity,motors,yaw_disturbance,roll_disturbance,pitch_disturbance)    
            else:
                print(f"AXAXAXAXAX {pravda_vniz}")
        else:
            #(x_target,0,x,y,altitude,target_altitude,roll,pitch,yaw,roll_velocity, pitch_velocity,motors,yaw_disturbance,roll_disturbance,pitch_disturbance)
            print("Zakonceno")
            print(y)
            roll_input, pitch_input, vertical_input = compute_motor_inputs(roll, pitch, roll_velocity, pitch_velocity, altitude, target_altitude)
            update_motors(motors, roll_input + roll_disturbance, pitch_input + pitch_disturbance, vertical_input, yaw_disturbance)
        
    

if __name__ == "__main__":
    main()