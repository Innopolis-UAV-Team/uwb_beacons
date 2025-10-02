import serial
from common.serial_messages import CircularBuffer

ser = serial.Serial('/dev/ttyUSB0', 460800, timeout=None, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, parity="E")

buffer: CircularBuffer = CircularBuffer(100)

resp = b''
bufsize = 0
while True:
    res = bytearray(60)
    # response = ser.readall()
    # print(response)
    # print(response.decode('utf-8', errors='ignore'))
    response = ser.readinto(res)
    print(res)
    print(res.decode('utf-8', errors='ignore'))
    # if response is None or len(response) == 0:
        # continue
    # print(response)
    # print(response.decode('ascii', errors='ignore'))
    # print(res)
    # print(res.decode('utf-8', errors='ignore'))
    # if response is None or len(response) == 0:
        # continue
    # print(response)
    # print(response.decode('utf-8', errors='ignore'))
