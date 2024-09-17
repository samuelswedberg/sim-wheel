from construct import Struct, Float32l, Int32ul, Int8ul, PaddedString, Padding

class AC_Telemetry_Parser:
    def __init__(self, struct_def):
        self.parser = struct_def

    def fromBuffer(self, buffer):
        return self.parser.parse(buffer)
    
class RTLapParser(AC_Telemetry_Parser):
    def __init__(self):
        # Define the binary structure according to the given offsets and data types
        lap_parser_struct = Struct(
            "carIdentifierNumber" / Int32ul,  # Offset: 0 (32-bit little-endian int)
            "lap" / Int32ul,                  # Offset: 4 (32-bit little-endian int)
            "driverName" / PaddedString(100, 'utf_16_le'),  # Offset: 8 (100 characters, UTF-16 little-endian)
            "carName" / PaddedString(100, 'utf_16_le'),     # Offset: 108 (100 characters, UTF-16 little-endian, strip null bytes)
            "time" / Int32ul                  # Offset: 208 (32-bit little-endian int)
        )
        super().__init__(lap_parser_struct)

class RTCarInfoParser(AC_Telemetry_Parser):
    def __init__(self):
        car_info_parser_struct = Struct(
            "identifier" / PaddedString(4, 'utf_16_le'),  # Offset: 0 (4 UTF-16 characters, strip null bytes)
            "size" / Int32ul,                             # Offset: 4 (32-bit little-endian int)

            "speedKmh" / Float32l,                        # Offset: 8 (32-bit little-endian float)
            "speedMph" / Float32l,                        # Offset: 12
            "speedMs" / Float32l,                         # Offset: 16

            "isAbsEnabled" / Int8ul,                      # Offset: 20 (8-bit unsigned int)
            "isAbsInAction" / Int8ul,                     # Offset: 21
            "isTcInAction" / Int8ul,                      # Offset: 22
            "isTcEnabled" / Int8ul,                       # Offset: 23
            "isInPit" / Int8ul,                           # Offset: 24
            "isEngineLimiterOn" / Int8ul,                 # Offset: 25
            Padding(2),                                   # Offset: 26-27 (skip 2 bytes)

            "accGVertical" / Float32l,                    # Offset: 28 (32-bit little-endian float)
            "accGHorizontal" / Float32l,                  # Offset: 32
            "accGFrontal" / Float32l,                     # Offset: 36

            "lapTime" / Int32ul,                          # Offset: 40 (32-bit little-endian int)
            "lastLap" / Int32ul,                          # Offset: 44
            "bestLap" / Int32ul,                          # Offset: 48
            "lapCount" / Int32ul,                         # Offset: 52

            "gas" / Float32l,                             # Offset: 56
            "brake" / Float32l,                           # Offset: 60
            "clutch" / Float32l,                          # Offset: 64
            "engineRPM" / Float32l,                       # Offset: 68
            "steer" / Float32l,                           # Offset: 72
            "gear" / Int32ul,                             # Offset: 76
            "cgHeight" / Float32l,                        # Offset: 80

            # Wheel angular speed fields
            "wheelAngularSpeed1" / Float32l,              # Offset: 84
            "wheelAngularSpeed2" / Float32l,              # Offset: 88
            "wheelAngularSpeed3" / Float32l,              # Offset: 92
            "wheelAngularSpeed4" / Float32l,              # Offset: 96

            # Slip angle fields
            "slipAngle1" / Float32l,                      # Offset: 100
            "slipAngle2" / Float32l,                      # Offset: 104
            "slipAngle3" / Float32l,                      # Offset: 108
            "slipAngle4" / Float32l,                      # Offset: 112

            # Slip angle contact patch fields
            "slipAngleContactPatch1" / Float32l,          # Offset: 116
            "slipAngleContactPatch2" / Float32l,          # Offset: 120
            "slipAngleContactPatch3" / Float32l,          # Offset: 124
            "slipAngleContactPatch4" / Float32l,          # Offset: 128

            # Slip ratio fields
            "slipRatio1" / Float32l,                      # Offset: 132
            "slipRatio2" / Float32l,                      # Offset: 136
            "slipRatio3" / Float32l,                      # Offset: 140
            "slipRatio4" / Float32l,                      # Offset: 144

            # Tyre slip fields
            "tyreSlip1" / Float32l,                       # Offset: 148
            "tyreSlip2" / Float32l,                       # Offset: 152
            "tyreSlip3" / Float32l,                       # Offset: 156
            "tyreSlip4" / Float32l,                       # Offset: 160

            # ND slip fields
            "ndSlip1" / Float32l,                         # Offset: 164
            "ndSlip2" / Float32l,                         # Offset: 168
            "ndSlip3" / Float32l,                         # Offset: 172
            "ndSlip4" / Float32l,                         # Offset: 176

            # Load fields
            "load1" / Float32l,                           # Offset: 180
            "load2" / Float32l,                           # Offset: 184
            "load3" / Float32l,                           # Offset: 188
            "load4" / Float32l,                           # Offset: 192

            # Dy fields
            "Dy1" / Float32l,                             # Offset: 196
            "Dy2" / Float32l,                             # Offset: 200
            "Dy3" / Float32l,                             # Offset: 204
            "Dy4" / Float32l,                             # Offset: 208

            # Mz fields
            "Mz1" / Float32l,                             # Offset: 212
            "Mz2" / Float32l,                             # Offset: 216
            "Mz3" / Float32l,                             # Offset: 220
            "Mz4" / Float32l,                             # Offset: 224

            # Tyre dirty level fields
            "tyreDirtyLevel1" / Float32l,                 # Offset: 228
            "tyreDirtyLevel2" / Float32l,                 # Offset: 232
            "tyreDirtyLevel3" / Float32l,                 # Offset: 236
            "tyreDirtyLevel4" / Float32l,                 # Offset: 240

            # Camber RAD fields
            "camberRAD1" / Float32l,                      # Offset: 244
            "camberRAD2" / Float32l,                      # Offset: 248
            "camberRAD3" / Float32l,                      # Offset: 252
            "camberRAD4" / Float32l,                      # Offset: 256

            # Tyre radius fields
            "tyreRadius1" / Float32l,                     # Offset: 260
            "tyreRadius2" / Float32l,                     # Offset: 264
            "tyreRadius3" / Float32l,                     # Offset: 268
            "tyreRadius4" / Float32l,                     # Offset: 272

            # Tyre loaded radius fields
            "tyreLoadedRadius1" / Float32l,               # Offset: 276
            "tyreLoadedRadius2" / Float32l,               # Offset: 280
            "tyreLoadedRadius3" / Float32l,               # Offset: 284
            "tyreLoadedRadius4" / Float32l,               # Offset: 288

            # Suspension height fields
            "suspensionHeight1" / Float32l,               # Offset: 292
            "suspensionHeight2" / Float32l,               # Offset: 296
            "suspensionHeight3" / Float32l,               # Offset: 300
            "suspensionHeight4" / Float32l,               # Offset: 304

            "carPositionNormalized" / Float32l,           # Offset: 308
            "carSlope" / Float32l,                        # Offset: 312

            # Car coordinates fields
            "carCoordinatesX" / Float32l,                 # Offset: 316
            "carCoordinatesY" / Float32l,                 # Offset: 320
            "carCoordinatesZ" / Float32l                  # Offset: 324
        )
        super().__init__(car_info_parser_struct)

class HandshakerResponseParser(AC_Telemetry_Parser):
    def __init__(self):
        handshaker_response_parser_struct = Struct(
            "carName" / PaddedString(100, 'utf_16_le'),      # Offset: 0
            "driverName" / PaddedString(100, 'utf_16_le'),    # Offset: 100
            "identifier" / Int32ul,                         # Offset: 200 (32-bit little-endian int)
            "version" / Int32ul,                            # Offset: 204
            "trackName" / PaddedString(100, 'utf_16_le'),     # Offset: 208
            "trackConfig" / PaddedString(100, 'utf_16_le'),    # Offset: 308
        )
        super().__init__(handshaker_response_parser_struct)