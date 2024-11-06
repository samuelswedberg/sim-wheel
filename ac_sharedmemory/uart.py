from pyaccsharedmemory import accSharedMemory
import serial
import time
import json
import sys

asm = accSharedMemory()

usart = serial.Serial('COM3', baudrate=115200, timeout=1)

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
    "brakeBias": int(sm.Physics.brake_bias),
    "forceFB": round(sm.Physics.final_ff, 2)
    }
    # data = { # Demo Data
    # "rpm": 3600,
    # "gear": 1,
    # "speedKmh": 120,
    # "hasDRS": 0,
    # "drs": 0,
    # "pitLim": 0,
    # "fuel": 45,
    # "brakeBias": 0,
    # "forceFB": 0.1
    # }

    print_data(data)
    # serialize the dictionary to a JSON string
    json_data = json.dumps(data) + '\n'  # Add a newline for message termination
    # Send the JSON string over UART
    usart.write(json_data.encode('utf-8'))  # Encode to bytes
        
def receive_data():
    """Function to receive data from usart."""
    if usart.in_waiting > 0:  # Check if there is data to read
        raw_line = usart.readline().decode('utf-8').rstrip()  # Read a line and decode it
        print("Received:", raw_line)  # Print the received data

def print_data(data):
    """Function to print the data struct in a single, updating line."""
    # Clear the line and move cursor to the beginning
    sys.stdout.write("\r")  
    # Format the data as a single string
    formatted_data = " | ".join([f"{key}: {value}" for key, value in data.items()])
    # Print the formatted data
    sys.stdout.write(formatted_data)
    # Flush the output to update the line immediately
    sys.stdout.flush()   

if __name__ == "__main__":
    try:
        print("Starting memory reading")
        while True:
            sm = asm.read_shared_memory()
            if (sm is not None):
                send_json_data()
                time.sleep(.05)
                receive_data()  # Continuously receive data from usart
    except KeyboardInterrupt:
        print("Program terminated.")
    finally:
        usart.close()