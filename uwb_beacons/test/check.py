import datetime
import time
from typing import Dict, Tuple
from numpy import mean
import serial
import struct
import argparse
from common.serial_messages import Message, CircularBuffer
from common.algoritms import solve_position, multilateration
from common.algoritms import calibrated_qubic,calibrated_quad

# Трилатерация по 3 якорям:
#     A1=(0,0,0), A2=(L2,0,0), A3=(0,L3,0).
L2 = 2000
L3 = 2000
idl2 = 2 #  X
idl3 = 6 #  Y
idl1 = 4 #  0

##CUBIC
a = 2.17754063e-07
b = -6.50749910e-04  
c = 2.60398686e-02
d = 3.62156905e-01
##QUAD
a2 = -6.76197533e-05  
b2 = 6.43933686e-03  
c2 = 4.33136513e-01

# Многолатерация по 3 якорям, можно добавить сколько угодно в формате id: (x, y, z)
positions: dict[int, tuple[float, float, float]] = {
    1: (0, 0, 0),
    6: (L2, 0, 0),
    7: (0, L3, 0),
}

def solve_multilateration(points: dict[int, dict[str, list[float]]]) -> tuple[float, float, float] | None:
    """
    Trilateration using the three points.
    :param points: Dictionary of points with id as key and distance as value.
    :return: Dictionary of points with id as key and distance as value.
    """
    try:
        data: dict[int, float] = {}
        for id in points.keys():
            if points[id]['data'] is None or len(points[id]['data']) == 0:
                continue
            val = mean(points[id]['data'])
            if not isinstance(val, float):
                continue
            data[id] = val
        res: Tuple[float, float, float] =  multilateration(data, positions, z_sign=1)
        print(f"Position: x={res[0]:.2f}\t\t y={res[1]:.2f}\t\t z={res[2]:.2f}")
        return res
    except ValueError as e:
        return None


def trilateration(points: dict[int, dict[str, list[float]]]) -> tuple[float, float, float] | None:
    """
    Trilateration using the three points.
    :param points: Dictionary of points with id as key and distance as value.
    :return: Dictionary of points with id as key and distance as value.
    """
    if points is None or len(points.keys()) < 3:
        print(points.keys())
        return None

    for id in points.keys():
        if points[id]['data'] is None or len(points[id]['data']) == 0:
            return None
    ids = list(points.keys())

    center = points[idl1]['data'][0]
    #center = center - (center/1000.0 - calibrated_qubic(a, b, c, d, center/1000.0))*1000
    center = (center/1000.0 - calibrated_quad(a2, b2, c2, center/1000.0))*1000
    
    xxx = points[idl2]['data'][0]
    #xxx = xxx - (xxx/1000.0 - calibrated_qubic(a, b, c, d, xxx/1000.0))*1000
    xxx = (xxx/1000.0 - calibrated_quad(a2, b2, c2, xxx/1000.0))*1000

    yyy = points[idl3]['data'][0]
    #yyy = yyy - (yyy/1000.0 - calibrated_qubic(a, b, c, d, yyy/1000.0))*1000
    yyy = (yyy/1000.0 - calibrated_quad(a2, b2, c2, yyy/1000.0))*1000
    
    res: Tuple[float, float, float] = solve_position(d1=center,
                   d2=xxx,
                   d3=yyy,
                   L2=L2,
                   L3=L3)
    
    #res: Tuple[float, float, float] = solve_position(d1=points[idl1]['data'][0],
    #               d2=points[idl2]['data'][0],
    #               d3=points[idl3]['data'][0],
    #               L2=L2,
    #               L3=L3)
    print(f"Position: x={res[0]:.2f}\t\t y={res[1]:.2f}\t\t z={res[2]:.2f}")
    return res

def check_ttl_serial(port: str, baudrate: int = 9600, timeout: float|None = None):
    """
    Check the functionality of a TTL to serial converter.

    :param port: The serial port to which the TTL converter is connected (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux).
    :param baudrate: The baud rate for communication. Default is 9600.
    :param timeout: Read timeout value. Default is 1 second.
    """

    try:
        # Initialize serial connection
        ser = serial.Serial(port, baudrate, timeout=timeout, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, parity="E")
        print(f"Connected to {port} at {baudrate} baud.")
        # Test data to send
        buffer: CircularBuffer = CircularBuffer(100)
        messages: Dict[int, dict] = {}
        last_message = time.time()

        while True:
            # Try decoding with error handling
            if ser.in_waiting > 0:
                try:
                    response = ser.read_until(b'\xFF\xFF\xFF\x00')
                    buffer.append(response)
                    if buffer.size == 0:
                        continue
                    for i in range(buffer.size):
                        msg = buffer.pop()
                        if msg is None:
                            continue
                        message = Message(msg)
                        if message.id not in messages:
                            messages[message.id] = {}
                            messages[message.id]['data'] = []
                        if message.data is None:
                            continue
                        assert isinstance(message.data, int)
                        messages[message.id]['data'].append(message.data)
                        messages[message.id]['last_message'] = time.time()

                    if time.time() - last_message < 0.2:

                        continue
                    last_message = time.time()
                    # all_messages_str = ""
                    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                    row = {}

                    res = solve_multilateration(messages)
                    # res = trilateration(messages)

                    for id, data in messages.items():
                        if time.time() - data['last_message'] > 10:
                            data['data'] = []
                            continue
                        row[f"{id}_pts"] = str(len(data['data']))
                        row[f"{id}_mean"] = int(mean(data['data'])) if len(data['data']) > 0 else 0
                        #row[f"{id}_cor"] = round((row[f"{id}_mean"]/1000.0 - calibrated_qubic(a, b, c, d, row[f"{id}_mean"]/1000.0))*1000)
                        row[f"{id}_cor"] = round((row[f"{id}_mean"]/1000.0 - calibrated_quad(a2, b2, c2, row[f"{id}_mean"]/1000.0))*1000)
                        messages[id]['data'] = []
                    # sort the keys
                    keys = sorted(row.keys())
                    string = f"ts: {ts} "
                    if res is not None:
                        string += f"res: x={res[0]:.2f} y={res[1]:.2f} z={res[2]:.2f} "
                    for i, key in enumerate(keys):
                        string += f"{key}: {row[key]}\t"
                    #print(string)
                except struct.error as e:
                    print(e)
                    print("wrong format: ", response)
    except serial.SerialException as e:
        print(f"Serial exception: {e}")
    except OSError as e:
        print(f"OS error: {e}")
    except Exception as e:
        print(f"An error occurred: {e}\n data: {response}")
    finally:
        # Close the serial connection
        ser.close()
        print("Serial connection closed.")

# Example usage
if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description='Save data from the TTL to serial converter.')
    argparser.add_argument('-p', '--port', type=str, default='/dev/ttyUSB0', help='The serial port to which the TTL converter is connected (e.g., \'COM3\' on Windows or \'/dev/ttyUSB0\' on Linux).')
    argparser.add_argument('-b', '--baudrate', type=int, default=230400  , help='The baud rate for communication. Default is 9600.')

    args = argparser.parse_args()
    check_ttl_serial(port=args.port, baudrate=args.baudrate)
