#!/usr/bin/env python3
import time
import rospy
import struct
import random
import asyncio
from asyncio import Queue
from ros_rocketclass import PressureSensorData, SystemState, FCUSensor
from std_msgs.msg import UInt32
import serial_asyncio

PressureSensorDatas = PressureSensorData()
SystemStateDatas = SystemState()
FCUSensorDatas = FCUSensor()
rospy.sleep(1)
class Data:
    def Bool2Byte(self, bool_values):
        return sum((1 << i) for i, val in enumerate(bool_values) if val)

    def MissionTime(self):
        Time_scale_factor = 10000
        current_time = time.time()
        midnight = time.mktime(time.strptime(time.strftime('%Y-%m-%d 00:00:00', time.localtime(current_time)), '%Y-%m-%d %H:%M:%S'))
        time_int = int((current_time - midnight) * Time_scale_factor)
        return struct.pack('>I', time_int)

    def calculate_checksum(self, data):
        return sum(data) & 0xFF

    def create_packet(self, packet_id, data_format, data_values):
        data = struct.pack(data_format, *data_values) if isinstance(data_values, (list, tuple)) else struct.pack(data_format, data_values)
        mtime = self.MissionTime()
        checksum = self.calculate_checksum(data)
        return struct.pack('B', packet_id) + mtime + data + struct.pack('B', checksum)

    def SystemStatePacket(self):
        # SSP = [True, False, True, True, False, True, False, True]
        SSP = SystemStateDatas.getData()
        return self.create_packet(0x01, 'B', self.Bool2Byte(SSP))

    def FlightModePacket(self):
        FMP = [True, True, True, True, False, False, False, True]
        return self.create_packet(0x02, 'B', self.Bool2Byte(FMP))

    def FCUStatePacket(self):
        # FSP = [False, False, True, False, False, True, False, True]
        FSP = FCUSensorDatas.getData()
        return self.create_packet(0x03, 'B', self.Bool2Byte(FSP))

    def NaviDataPacket(self):
        NDP = [random.uniform(0.000, 10000.000) for _ in range(7)]
        return self.create_packet(0x04, '7f', NDP)

    def TVCDataPacket(self):
        TDP = [random.randint(-180, 180) for _ in range(3)]
        return self.create_packet(0x05, '3h', TDP)

    def PressureDataPacket(self):
        # PDP = [random.randint(0, 10000) for _ in range(9)]
        PDP = [int(PressureSensorDatas.FirstEnginePressure*1000),int(PressureSensorDatas.FirstTankPressure*1000),
               int(PressureSensorDatas.SecondEnginePressure*1000),int(PressureSensorDatas.SecondTankPressure*1000),
               int(PressureSensorDatas.RCSPressure1*1000),int(PressureSensorDatas.RCSPressure2*1000),
               int(PressureSensorDatas.RCSPressure3*1000),int(PressureSensorDatas.RCSPressure4*1000),int(PressureSensorDatas.RCSTankPressure*1000)]
        print(PDP)
        return self.create_packet(0x06, '9H', PDP)
    # 'Info': [
    #         ('FirstEnginePressure'),
    #         ('FirstTankPressure'),
    #         ('SecondEnginePressure'),
    #         ('SecondTankPressure'),
    #         ('RCSPressure1'),
    #         ('RCSPressure2'),
    #         ('RCSPressure3'),
    #         ('RCSPressure4'),
    #         ('RCSTankPressure')
    #     ]


class SerialWriter(asyncio.Protocol):
    def __init__(self, q: Queue):
        self.queue = q
        self.transport = None
        self.counters = {
            0x01: 0,
            0x02: 0,
            0x03: 0,
            0x04: 0,
            0x05: 0,
            0x06: 0
        }

    def connection_made(self, transport):
        print("Serial port opened:", transport)
        self.transport = transport
        asyncio.create_task(self.send_packets())

    async def send_packets(self):
        while True:
            packet = await self.queue.get()  
            packet_id = packet[0]
            self.transport.write(packet)
            # print("Send packet:", repr(packet), "packet length", len(packet))
            if packet_id in self.counters:
                self.counters[packet_id] += 1
                # print(f"Packet ID 0x{packet_id:02X} sent count: {self.counters[packet_id]}")

    def connection_lost(self, exc):
        print("Serial port closed")
        self.transport = None

async def task_main(queue: Queue, packet_func, samp_rate):
    while True:
        packet = packet_func()
        await queue.put(packet)
        await asyncio.sleep(samp_rate)

async def main():
    data = Data()
    queue = Queue()

    transport, protocol = await serial_asyncio.create_serial_connection(
        asyncio.get_running_loop(),
        lambda: SerialWriter(queue),
        '/dev/ttyUSB0',
        baudrate=9600
    )

    asyncio.create_task(task_main(queue, data.SystemStatePacket, 0.1))
    asyncio.create_task(task_main(queue, data.FlightModePacket, 0.1))
    asyncio.create_task(task_main(queue, data.FCUStatePacket, 0.1))
    # asyncio.create_task(task_main(queue, data.NaviDataPacket, 0.05))
    # asyncio.create_task(task_main(queue, data.TVCDataPacket, 0.05))
    asyncio.create_task(task_main(queue, data.PressureDataPacket, 0.2))

    await asyncio.Future()

if __name__ == '__main__':
    rospy.init_node('telemetry_node', anonymous=True)

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print('KeyboardInterrupt')
    finally:
        print('Port closed')