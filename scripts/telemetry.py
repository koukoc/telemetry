#!/usr/bin/env python3
import serial
from SystemState import SystemState
from FCUSensorState import FCUSensorState
from FlightMode import FlightMode
from NavagationData import NavagationData
from PressureSensorData import PressureSensorData
from TVCData import TVCData
import time
import random

def rocket_data(packet_count):
    header = "HDR"
    mission_time = random.randint(0, 4294967295)
    system_state = SystemState()
    systemStateData = [system_state.FirstStageIgnition] # 自己改格式
    flight_mode = FlightMode()
    flight_modeData = [flight_mode.Hold]
    FCU_sensor = FCUSensorState()
    FCU_sensor_data = []
    navigation = NavagationData()
    navigation_data = []
    tvc = TVCData()
    tvc_data = []
    pressure_sensor = PressureSensorData()
    pressure_sensor_data = []
    checksum = random.randint(0, 4294967295)
    tailer = "TLR"

    data = f"{header},{packet_count},{mission_time},{},{},{},{},{},{},{checksum},{tailer}"
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
            data = rocket_data(packet_count)
            data = data + "\n"
            sender_ser.write(data.encode())

            print('Send Data:\n',data)

            time.sleep(1.5)

            packet_count += 1
    except KeyboardInterrupt:
        print("Sender terminated by user.")
    finally:
        sender_ser.close