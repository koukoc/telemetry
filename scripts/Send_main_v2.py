import serial
import struct
import time
import random
import asyncio

#ser = serial.Serial('/dev/ttyUSB0', 9600)
ser = serial.Serial('COM5', 9600)

print ("serial start at port", ser.port)
print ("serial baudrate set to", ser.baudrate)
print ("serial timeout set to", ser.timeout)

AccumlateBytes = 0

class Data:
    def SystemStateData(self):
        return [True, False, True, True, False, True, False, True]
    def FlightModeData(self):
        return [True, True, True, True, False, False, False, True]
    def FCUStateData(self):
        return [False, False, True, False, False, True, False, True]
    def NavigationData(self):
        return [random.uniform(0.000, 10000.000) for _ in range(7)]
    def TVCData(self):
        return [random.randint(-180, 180) for _ in range(3)]
    def PressureData(self):
        return [random.randint(0, 10000) for _ in range(9)]

    def Bool2Byte(self, bool_value):
        byte_value = 0
        for i, bool_val in enumerate(bool_value):
            if bool_val:
                byte_value |= (1 << i)
        return byte_value
    
    def MissionTime(self):
        Currentstamp = time.time()
        local_time = time.localtime(Currentstamp)
        midnight = time.struct_time((
            local_time.tm_year,
            local_time.tm_mon, 
            local_time.tm_mday,
            0, 0, 0,            
            local_time.tm_wday,
            local_time.tm_yday, 
            local_time.tm_isdst
        ))
        midnight_timestamp = time.mktime(midnight)
        times = Currentstamp - midnight_timestamp
        scale_factor = 10000
        time_int = int(times * scale_factor)
        packet_time = struct.pack('>I', time_int)
        return packet_time
    
    def calculate_checksum(self, data):
        checksum = 0
        for byte in data:
            checksum += byte
        return checksum & 0xFF
    
    def SystemStatePacket(self):
        systemState = self.Bool2Byte(self.SystemStateData())
        data = struct.pack('B', systemState)
        Mtime = self.MissionTime()
        checksum = self.calculate_checksum(data)
        packet1 = b'\x01' + Mtime + data + struct.pack('B', checksum) + b'\xff'
        return packet1  

    def FlightModePacket(self):
        flightmode = self.Bool2Byte(self.FlightModeData())
        data = struct.pack('B', flightmode)
        Mtime = self.MissionTime()
        checksum = self.calculate_checksum(data)
        packet2 = b'\x02' + Mtime + data + struct.pack('B', checksum) + b'\xff'
        return packet2

    def FCUStatePacket(self):
        sensor_state = self.Bool2Byte(self.FCUStateData())
        data = struct.pack('B', sensor_state)
        Mtime = self.MissionTime()
        checksum = self.calculate_checksum(data)
        packet3 = b'\x03' + Mtime + data + struct.pack('B', checksum) + b'\xff'
        return packet3

    def NaviDataPacket(self):
        navi = self.NavigationData()
        data = struct.pack('7f', *navi)
        Mtime = self.MissionTime()
        checksum = self.calculate_checksum(data)
        packet4 = b'\x04' + Mtime + data + struct.pack('B', checksum) + b'\xff'
        return packet4

    def TVCDataPacket(self):
        tvc = self.TVCData()
        data = struct.pack('3h', *tvc)
        Mtime = self.MissionTime()
        checksum = self.calculate_checksum(data)
        packet5 = b'\x05' + Mtime + data + struct.pack('B', checksum) + b'\xff'
        return packet5

    def PressureDataPacket(self):
        pressure = self.PressureData()
        data = struct.pack('9H', *pressure)
        Mtime = self.MissionTime()
        checksum = self.calculate_checksum(data)
        packet6 = b'\x06' + Mtime + data + struct.pack('B', checksum) + b'\xff'
        return packet6     

task = Data()

async def task1_main():
    global AccumlateBytes
    while True:
        data1 = task.SystemStatePacket()
        ser.write(data1)
        AccumlateBytes += len(data1)
        print ("Send SysStatPack :", data1,"Data length :", len(data1))
        await asyncio.sleep(0.1)

async def task2_main():
    global AccumlateBytes
    while True:
        data2 = task.FlightModePacket()
        ser.write(data2)
        AccumlateBytes += len(data2)
        print ("Send FligModePack :", data2,"Data length :", len(data2))
        await asyncio.sleep(0.1)

async def task3_main():
    global AccumlateBytes
    while True:
        data3 = task.FCUStatePacket()
        ser.write(data3)
        AccumlateBytes += len(data3)
        print ("Send FCUStatPack :", data3,"Data length :", len(data3))
        await asyncio.sleep(0.1)

async def task4_main():
    global AccumlateBytes
    while True:
        data4 = task.NaviDataPacket()
        ser.write(data4)
        AccumlateBytes += len(data4)
        print ("Send NaviDatPack :", data4,"Data length :", len(data4))
        await asyncio.sleep(0.05)

async def task5_main():
    global AccumlateBytes
    while True:
        data5 = task.TVCDataPacket()
        ser.write(data5)
        AccumlateBytes += len(data5)
        print ("Send TVCDatPack :", data5,"Data length :", len(data5))
        await asyncio.sleep(0.05)

async def task6_main():
    global AccumlateBytes
    while True:
        data6 = task.PressureDataPacket()
        ser.write(data6)
        AccumlateBytes += len(data6)
        print ("Send PresDatPack :", data6,"Data length :", len(data6))
        await asyncio.sleep(0.2)

async def main():
    task1 = asyncio.create_task(task1_main())
    task2 = asyncio.create_task(task2_main())
    task3 = asyncio.create_task(task3_main())
    task4 = asyncio.create_task(task4_main())
    task5 = asyncio.create_task(task5_main())
    task6 = asyncio.create_task(task6_main())

    await asyncio.gather(task1, task2, task3, task4, task5, task6)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        ser.close()
        print('KeyboardInterrupt')
    finally:
        ser.close()
        print('Port closed')