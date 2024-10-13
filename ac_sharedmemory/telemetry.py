from construct import struct
from pyaccsharedmemory import accSharedMemory
import serial
import time

asm = accSharedMemory()
ser = serial.Serial('COM3', baudrate=115200, timeout=1)

print("Starting memory reading")
# while(1):
#     sm = asm.read_shared_memory()

    # def LCD_Telemetry():
    #     grabTelemetry("speedKmh", int(sm.Physics.speed_kmh))
    #     grabTelemetry("gear", sm.Physics.gear)
    #     grabTelemetry("rpm", sm.Physics.rpm)

    # def grabTelemetry(key, value):
    #     command = f"{key}:{value}\n"
    #     length = len(command)
    #     formatted_command = f"{length}{command}"  # Format as "length:command"
    #     ser.write(formatted_command.encode('utf-8'))
    #     time.sleep(0.1)  # Add small delay to avoid overwhelming the STM32
    #     response = ser.readline().decode('utf-8').strip()
    #     if(response):
    #         print(f"Response from STM32: {response}")

    # if (sm is not None):
    #     LCD_Telemetry()
    #     time.sleep(.1)
flag = True
while True:
    if(flag):
        command = "gear:100\n"
        length = len(command)
        formatted_command = f"{length}{command}"  # Format as "length:command"
        ser.write(formatted_command.encode('utf-8'))
        flag = False
    response = ser.readline().decode('utf-8').strip()
    if(response):
        print(f"Response from STM32: {response}")
    time.sleep(.1)  # Adjust delay as needed

    
    
