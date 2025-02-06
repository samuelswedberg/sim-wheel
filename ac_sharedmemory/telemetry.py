from pyaccsharedmemory import accSharedMemory
import serial
import time
import json
import struct 

STRUCT_FORMAT = "<9i1f" # <#i1f the # is the number of items - 1
PORT = "COM10"
BAUDRATE = 115200
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

ASM = accSharedMemory()

"""
Steering Wheel Wheel Telemetry Data
"""
def send_telemetry(sm, ser):
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
    "maxRpm": int(sm.Static.max_rpm),
    "forceFB": round(sm.Physics.final_ff, 2)
    }

    packed_data = struct.pack(
        STRUCT_FORMAT,
        data["rpm"],
        data["gear"],
        data["speedKmh"],
        data["hasDRS"],
        data["drs"],
        data["pitLim"],
        data["fuel"],
        data["brakeBias"],
        data["maxRpm"],
        data["forceFB"]
    )

    try:
        ser.write(packed_data)
        print("Data sent successfully.")
    except serial.SerialException as e:
        print(f"Error sending data: {e}")

"""
Receives communications from the wheelbase
"""
def receive_data(ser):
    """Receive binary data and unpack the struct."""
    try:
        # Read the exact number of bytes for the struct
        raw_data = ser.read(STRUCT_SIZE)
        
        # Ensure enough data was received
        if len(raw_data) != STRUCT_SIZE:
            print(f"Incomplete data received. Expected {STRUCT_SIZE} bytes, got {len(raw_data)}.")
            return

        # Unpack the binary data
        unpacked_data = struct.unpack(STRUCT_FORMAT, raw_data)

        # Print the unpacked data for debugging
        print(f"Received Data: {unpacked_data}")

    except struct.error as e:
        raise ValueError(f"Error unpacking data: {e}") from e
    except Exception as e:
        raise RuntimeError(f"Unexpected error in receive_data: {e}") from e

"""
Attempt to reconnect to the serial port
"""
def reconnect_serial():

    while True:
        try:
            print("Attempting to reconnect...")
            ser = serial.Serial(PORT, BAUDRATE, timeout=1)
            print("Reconnected successfully.")
            return ser
        except serial.SerialException as e:
            print(f"Reconnection failed: {e}")
            time.sleep(5)  # Wait before retrying

"""
Main loop to send data and handle reconnections
"""
def main():
    ser = None
    while True:
        try:
            # Try to establish a connection if not already connected
            if ser is None or not ser.is_open:
                print("Attempting to connect to COM port...")
                ser = serial.Serial(PORT, BAUDRATE, timeout=1)
                print("Connected successfully!")

            sm = ASM.read_shared_memory()
            
            if (sm is not None):
                send_telemetry(sm, ser)
                time.sleep(.1)
                receive_data(ser)

            # time.sleep(1)

        except (serial.SerialException, RuntimeError) as e:
            print(f"Connection lost: {e}")
            if ser:
                ser.close()
                ser = reconnect_serial()

            # Retry connection every 3 seconds
            time.sleep(3)
        except KeyboardInterrupt:
            print("Program terminated.")
            ser.close()

if __name__ == "__main__":
    main()
