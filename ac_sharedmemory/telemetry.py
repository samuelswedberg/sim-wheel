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
        data = ("speedKmh", sm.Physics.rpm)
        strName = data[0][:8].ljust(8, '\0')  # Make sure the string is exactly 8 bytes
        lcd_telemetry = struct.pack(struct_format, strName.encode('utf-8'), data[1])
        ser.write(lcd_telemetry)
        print(f"Send data: {data}")

    def receive_response():
        """
        Receives a response from the STM32.
        """
        response = ser.readline().decode('utf-8').strip()  # Read until newline and decode
        print(f"Received from STM32: {response}")
    if (sm is not None):
        LCD_Telemetry()
        receive_response()  # Receive the response from STM32
        #time.sleep(1)
        
        

    
    
