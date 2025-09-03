import os
import struct
import sys
from typing import List, Tuple
import serial
import time
import argparse
from common.serial_messages import Message, CircularBuffer, decode

num_entries = 0

def save_to_file(string: str, file: str) -> Tuple[int, float]:
    messages = decode(string)
    data = ""
    for m in messages:
        global num_entries
        num_entries += 1
        if num_entries > 1000:
            raise Exception("Got nessecary amount of data")
        data += f"{m}\n"
    with open(file, 'a') as f:
        f.write(data)
        f.close()

def connect_and_save(port: str, baudrate: int = 9600, timeout: float = None,
                        file_name: str = None, num_points: int = 1000):
    """
    Check the functionality of a TTL to serial converter.

    :param port: The serial port to which the TTL converter is connected (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux).
    :param baudrate: The baud rate for communication. Default is 9600.
    :param timeout: Read timeout value. Default is 1 second.
    """
    # open file to store results
    response = ""
    try:
        # Initialize serial connection
        ser = serial.Serial(port, baudrate, timeout=timeout, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, parity="E")
        print(f"Connected to {port} at {baudrate} baud.")
        max_size = 100
        buffer: CircularBuffer = CircularBuffer(max_size)
        point_count = 0
        while True:
            # Try decoding with error handling
            if ser.in_waiting > 0:
                try:
                    response = ser.read_until(b'\xFF\x00')
                    buffer.append(response)
                    if buffer.size > max_size/2:
                        with open(file_name, 'a') as f:
                            for i in range(buffer.size):
                                point_count += 1
                                if point_count > num_points:
                                    print(f"Saved {num_points} points to {file_name}")
                                    f.close()
                                    ser.close()
                                    return

                                msg = buffer.pop()
                                f.write(str(Message(msg)) + '\n')
                            f.close()
                except struct.error as e:
                    print(e)
                    print(response)

    except serial.SerialException as e:
        print(f"Serial exception: {e}")
    except Exception as e:
        print(f"An error occurred: {e}\n data: {response}")
    finally:
        # Close the serial connection
        ser.close()
        print("Serial connection closed.")



# Example usage
if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description='Save data from the TTL to serial converter.')
    argparser.add_argument('--port', type=str, default='/dev/ttyUSB0', help='The serial port to which the TTL converter is connected (e.g., \'COM3\' on Windows or \'/dev/ttyUSB0\' on Linux).')
    argparser.add_argument('--baudrate', type=int, default=230400, help='The baud rate for communication. Default is 230400.')
    argparser.add_argument('--num_points', type=int, default=1000, help='Number of points to collect before stopping. Default is 1000.')
    
    args = argparser.parse_args()

    file_name = f'{args.num_points}_{time.strftime("%Y-%m-%d_%H-%M-%S")}.txt'
    # get the current file directory

    file_dir = os.path.dirname(os.path.realpath(__file__))

    full_path = os.path.join(file_dir, "data", file_name)
    print(f"File directory: {file_dir}")

    # Replace 'COM3' with the appropriate port for your environment
    connect_and_save(port=args.port, baudrate=args.baudrate,
                        file_name=full_path, num_points=args.num_points)
