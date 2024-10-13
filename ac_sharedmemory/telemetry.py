from pyaccsharedmemory import accSharedMemory
import serial
import time
import json

asm = accSharedMemory()

usart = serial.Serial('COM3', baudrate=921600, timeout=.01)

def send_json_data():
    """Function to send data to usart."""
    data = {
    "rpm": sm.Physics.rpm,
    "gear": sm.Physics.gear,
    "speedKmh": int(sm.Physics.speed_kmh),
    "hasDRS": int(sm.Static.hasDRS),
    "drs": int(sm.Physics.drs),
    "pitLim": int(sm.Physics.pit_limiter_on),
    "fuel": int(sm.Physics.fuel),
    "brakeBias": int(sm.Physics.brake_bias)
    }
    # usartialize the dictionary to a JSON string
    json_data = json.dumps(data) + '\n'  # Add a newline for message termination
    # Send the JSON string over UART
    usart.write(json_data.encode('utf-8'))  # Encode to bytes
        
def receive_data():
    """Function to receive data from usart."""
    while True:
        if usart.in_waiting > 0:  # Check if there is data to read
            raw_line = usart.readline().decode('utf-8').rstrip()  # Read a line and decode it
            print("Received:", raw_line)  # Print the received data

if __name__ == "__main__":
    try:
        print("Starting memory reading")
        while True:
            sm = asm.read_shared_memory()
            if (sm is not None):
                send_json_data()
                time.sleep(.01)
                #receive_data()  # Continuously receive data from usart
    except KeyboardInterrupt:
        print("Program terminated.")
    finally:
        usart.close()