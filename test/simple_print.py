import serial
from common.serial_messages import CircularBuffer

ser = serial.Serial('/dev/ttyUSB0', 230400, timeout=None, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, parity="E")

buffer: CircularBuffer = CircularBuffer(100)

resp = b''
while True:
    response = ser.readline()
    if response is None or len(response) == 0:
        continue
    print(response.decode('utf-8', errors='ignore'))
