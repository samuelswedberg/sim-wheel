import socket
import struct
import threading

from parsers import RTCarInfoParser, RTLapParser, AC_Telemetry_Parser, HandshakerResponseParser

# Constants
AC_SERVER_PORT = 9996;
AC_SERVER_VERSION = 1;

deviceIdentifier = {
  "eIPhoneDevice": 0,
  "eIPadDevice": 1,
  "eAndroidPhone": 2,
  "eAndroidTablet": 3,
}

# Operation types
operation = {
    "HANDSHAKE": 0, 
    "SUBSCRIBE_UPDATE": 1, 
    "SUBSCRIBE_SPOT": 2, 
    "DISMISS": 3 
}

# Event types
event = {
    "HANDSHAKER_RESPONSE": "HANDSHAKER_RESPONSE",
    "RT_CAR_INFO": "RT_CAR_INFO",
    "RT_LAP": "RT_LAP",
}

class AC_Telemetry:
    def __init__(self, acServerIp):
        super()
        self.listeners = {}
        self.acServerIp = acServerIp
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client.settimeout(300)  # Optional timeout
        self.is_running = False

    def on(self, event, listener):
        if event not in self.listeners:
            self.listeners[event] = []
        self.listeners[event].append(listener)

    def emit(self, event, *args):
        if event in self.listeners:
            for listener in self.listeners[event]:
                listener(*args)

    def start(self):
        try:
            if(not self.client): 
                return;
        
            self.is_running = True
            listen_thread = threading.Thread(target=self.listen)
            listen_thread.start()
            print(f"UDP Client listening on {self.acServerIp}:{AC_SERVER_PORT} 🏎")

        except socket.error as e:
            print(f"Socket error: {e}")
        
    def listen(self):
        while self.is_running:
            try:
                data, addr = self.client.recvfrom(1024)  # Buffer size
                self.parse_message(data, addr)
            except socket.error as e:
                print(f"Socket error: {e}")
                break

    def stop(self):
        self.is_running = False
        self.client.close()
        print("UDP Client closed 🏁")

    def send_handshaker(self, op, identifier=deviceIdentifier["eIPhoneDevice"], version=AC_SERVER_VERSION):
        message = struct.pack('<iii', identifier, version, op)  # 3 integers (12 bytes total)
        self.client.sendto(message, (self.acServerIp, AC_SERVER_PORT))

    def handshake(self):
        self.send_handshaker(operation["HANDSHAKE"])

    def subscribe_update(self):
        self.send_handshaker(operation["SUBSCRIBE_UPDATE"])

    def subscribe_spot(self):
        self.send_handshaker(operation["SUBSCRIBE_SPOT"])

    def dismiss(self):
        self.send_handshaker(operation["DISMISS"])

    def parse_message(self, msg, addr):
        size = len(msg)
        if size == 408:
            self.emit(event["HANDSHAKER_RESPONSE"], HandshakerResponseParser().fromBuffer(msg))
        elif size == 328:
            self.emit(event["RT_CAR_INFO"], RTCarInfoParser().fromBuffer(msg))
        elif size == 212:
            self.emit(event["RT_LAP"], RTLapParser().fromBuffer(msg))
        else:
            print(f"Unknown message size: {len(msg)}")