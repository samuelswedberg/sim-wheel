from pyaccsharedmemory import accSharedMemory
import serial
import time
import json
import struct 

STRUCT_FORMAT = "<8i1f"
PORT = "COM10"
BAUDRATE = 115200
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

ASM = accSharedMemory()

# def send_json_data():
#     """Function to send data to usart."""
#     data = {
#     "rpm": sm.Physics.rpm,
#     "gear": sm.Physics.gear,
#     "speedKmh": int(sm.Physics.speed_kmh),
#     "hasDRS": int(sm.Static.hasDRS),
#     "drs": int(sm.Physics.drs),
#     "pitLim": int(sm.Physics.pit_limiter_on),
#     "fuel": int(sm.Physics.fuel),
#     "brakeBias": int(sm.Physics.brake_bias),
#     "forceFB": round(sm.Physics.final_ff, 2)
#     }
#     # serialize the dictionary to a JSON string
#     json_data = json.dumps(data) + '\n'  # Add a newline for message termination
#     # Send the JSON string over UART
#     ser.write(json_data.encode('utf-8'))  # Encode to bytes
        
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
        data["forceFB"]
    )

    try:
        ser.write(packed_data)
        print("Data sent successfully.")
    except serial.SerialException as e:
        print(f"Error sending data: {e}")

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
        print(f"Error unpacking data: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

def main():
    """Main loop to send data and handle reconnections."""
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

        except serial.SerialException as e:
            print(f"Connection lost: {e}")
            if ser:
                ser.close()
                ser = None

            # Retry connection every 3 seconds
            time.sleep(3)
        except KeyboardInterrupt:
            print("Program terminated.")
            ser.close()

if __name__ == "__main__":
    main()
    # try:
    #     print("Starting memory reading")
    #     while True:
    #         sm = asm.read_shared_memory()
    #         if (sm is not None):
    #             send_telemetry()
    #             time.sleep(.05)
    #             receive_data()  # Continuously receive data from usart
    # except KeyboardInterrupt:
    #     print("Program terminated.")
    # finally:
    #     ser.close()