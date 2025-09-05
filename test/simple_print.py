import serial
from common.serial_messages import CircularBuffer

ser = serial.Serial('/dev/ttyUSB0', 230400, timeout=None, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, parity="E")

buffer: CircularBuffer = CircularBuffer(100)

resp = b''
while True:
    response = ser.read_all()
    if response is None or len(response) == 0:
        continue
    resp += response
    if b'\r\n' not in resp:
        print(resp)
        continue
    response = resp.split(b'\r\n')
    buffer.append(response)
    if buffer.size == 0:
        continue
    for i in range(buffer.size):
        response = buffer.pop()
        if response is None:
            continue
        if len(response) == 0:
            continue
        print(response)
    try:
        print(response.decode('utf-8'))
    except UnicodeDecodeError:
        print(response)
        continue
