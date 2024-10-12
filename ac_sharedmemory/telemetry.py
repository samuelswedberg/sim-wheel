from construct import struct
from pyaccsharedmemory import accSharedMemory
import serial
import time

asm = accSharedMemory()
struct_format = '8sf'
ser = serial.Serial('COM3', baudrate=115200, timeout=1)

print("Starting memory reading")
while(1):
    sm = asm.read_shared_memory()

    def LCD_Telemetry():
        grabTelemetry("speedKmh", int(sm.Physics.speed_kmh))
        grabTelemetry("gear", sm.Physics.gear)
        grabTelemetry("rpm", sm.Physics.rpm)

    def grabTelemetry(key, value):
        data = key+str(value)
        ser.write(data.encode())
        #print(f"Send data: {data}")
        response = ser.read(ser.inWaiting()).decode().strip()  # Read all available characters
        if(response):
            print(f"Response from STM32: {response}")
    def receive_response():
        """
        Receives a response from the STM32.
        """
        response = ser.readline().decode('utf-8').strip()  # Read until newline and decode
        print(f"Received from STM32: {response}")

    if (sm is not None):
        LCD_Telemetry()
        #receive_response()  # Receive the response from STM32
        time.sleep(.1)
        
        

    
    
