from telemetry import AC_Telemetry
import socket, time
ac_client = AC_Telemetry('localhost')  # Replace with actual IP

# Listen for handshaker responses
ac_client.on("HANDSHAKER_RESPONSE", lambda data: print(f"Handshaker Response: {data}"))
ac_client.on("RT_CAR_INFO", lambda data: print(f"Car Info: {data.gear}"))
ac_client.on("RT_LAP", lambda data: print(f"Lap Info: {data}"))

# Start the client and send a handshake
ac_client.start()
ac_client.handshake()

ac_client.subscribe_update()
ac_client.subscribe_spot()
# Keep the program running for a while to test
try:
    while True:
        pass
except KeyboardInterrupt:
    ac_client.stop()
