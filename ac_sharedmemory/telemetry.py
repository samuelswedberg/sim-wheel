from construct import struct
from pyaccsharedmemory import accSharedMemory
import serial
import time

asm = accSharedMemory()

print("Starting memory reading")
while(1):
    sm = asm.read_shared_memory()
    ser = serial.Serial('COM3', baudrate=115200, timeout=.01)
    def LCD_Telemetry():
        grabTelemetry("speedKmh", int(sm.Physics.speed_kmh))
        grabTelemetry("gear", sm.Physics.gear)
        grabTelemetry("rpm", sm.Physics.rpm)

    def grabTelemetry(key, value):
        command = f"{key}:{value}\n"
        #length = len(command)
        #formatted_command = f"{length}{command}"  # Format as "length:command"
        ser.write(command.encode('utf-8'))
        response = ser.readline().decode('utf-8').strip()
        if(response):
            print(f"Response from STM32: {response}")
        
    if (sm is not None):
        LCD_Telemetry()
        ser.close()
        #time.sleep(.1)
# flag = True
# while True:
#     ser = serial.Serial('COM3', baudrate=115200, timeout=.1)
#     if(flag):
#         command = "speedKmh:100\n"
#         #length = len(command)
#         #formatted_command = f"{length}{command}"  # Format as "length:command"
#         ser.write(command.encode('utf-8'))
#         #flag = False
#     response = ser.readline().decode('utf-8').strip()
#     if(response):
#         print(f"Response from STM32: {response}")
#     ser.close()
#     #time.sleep(.01)  # Adjust delay as needed

    
    
