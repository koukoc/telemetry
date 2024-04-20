#!/usr/bin/env python3
import serial
from SystemState import SystemState
import time
import random

def simulate_rocket_data(packet_count):
    header = "HDR"
    mission_time = random.randint(0, 4294967295)
    system_state = SystemState()
    systemStateData = [system_state.FirstStageIgnition] # 自己改格式
    flight_mode = format(random.randint(0, 15), '04b')
    sensor_data = format(random.randint(0, 255), '08b')
    navigation_data = ",".join([str(random.random()) for _ in range(7)])
    tvc_data = ",".join([str(random.randint(-32768, 32767)) for _ in range(3)])
    pressure_sensor = ",".join([str(random.randint(0, 65535)) for _ in range(9)])
    checksum = random.randint(0, 4294967295)
    tailer = "TLR"

    data = f"{header},{packet_count},{mission_time},{system_state},{flight_mode},{sensor_data},{navigation_data},{tvc_data},{pressure_sensor},{checksum},{tailer}"
    return data




if __name__ == '__main__':
    sender_ser = serial.Serial() # define

    # Settings
    sender_ser.port = "COM5"
    sender_ser.baudrate = 9600
    sender_ser.timeout = 0
    sender_ser.bytesize = 8

    packet_count = 0

    sender_ser.open()
    print("Start to send data from port :" + str(sender_ser.port))

    try:
        while True:
            data = simulate_rocket_data(packet_count)
            data = data + "\n"
            sender_ser.write(data.encode())

            print('Send Data:\n',data)

            time.sleep(1.5)

            packet_count += 1
    except KeyboardInterrupt:
        print("Sender terminated by user.")
    finally:
        sender_ser.close