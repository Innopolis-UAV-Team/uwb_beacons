import datetime
import time
from typing import Dict
from numpy import mean
import serial
import struct
import argparse
from common.serial_messages import Message, CircularBuffer

def check_ttl_serial(port: str, baudrate: int = 9600, timeout: float = None):
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
                    response = ser.read_until(b'\xFF\x00')
                    buffer.append(response)
                    if buffer.size == 0:
                        continue
                    for i in range(buffer.size):
                        msg = buffer.pop()
                        if msg is None:
                            continue
                        # print(Message(msg))
                        message = Message(msg)
                        if message.id not in messages:
                            messages[message.id] = {}
                            messages[message.id]['data'] = []
                        if message.data is None:
                            continue
                        assert isinstance(message.data, int)
                        messages[message.id]['data'].append(message.data)
                        messages[message.id]['last_message'] = time.time()

                    if time.time() - last_message < 1:
                        continue
                    last_message = time.time()
                    # all_messages_str = ""
                    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                    row = {}

                    for id, data in messages.items():
                        if time.time() - data['last_message'] > 10:
                            data['data'] = []
                            continue
                        row[f"{id}_pts"] = str(len(data['data']))
                        row[f"{id}_mean"] = str(int(mean(data['data']))) if len(data['data']) > 0 else "N/A"
                        messages[id]['data'] = []
                    # sort the keys
                    keys = sorted(row.keys())
                    srting = f"ts: {ts}\t"
                    for i, key in enumerate(keys):
                        srting += f"{key}: {row[key]}\t"
                    print(srting)
                except struct.error as e:
                    print(e)
                    print(response)
    except serial.SerialException as e:
        print(f"Serial exception: {e}")
    except OSError as e:
        print(f"OS error: {e}")

    finally:
        # Close the serial connection
        ser.close()
        print("Serial connection closed.")

# Example usage
if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description='Save data from the TTL to serial converter.')
    argparser.add_argument('-p', '--port', type=str, default='/dev/ttyUSB0', help='The serial port to which the TTL converter is connected (e.g., \'COM3\' on Windows or \'/dev/ttyUSB0\' on Linux).')
    argparser.add_argument('-b', '--baudrate', type=int, default=460800  , help='The baud rate for communication. Default is 9600.')

    args = argparser.parse_args()
    check_ttl_serial(port=args.port, baudrate=args.baudrate)
