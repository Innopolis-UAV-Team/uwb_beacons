import os
import struct
import sys
from typing import List, Tuple
import serial
import time
import argparse

num_entries = 0

class Message:
    def __init__(self, data: bytes):
        self.id =  struct.unpack('B', data[0:1])[0]
        self.data = 0
        self.data = struct.unpack('<H', data[1:3])[0]  # '<H' denotes little-endian, uint16_t

    def __str__(self):
        return f"id: {self.id}, data: {self.data}"

def decode(data: bytes) -> List[Message]:
    data = data.split(b'\xFF\x00')
    messages = []
    for d in data:
        messages.append(Message(d))
    return messages


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

def connect_and_save(port: str, baudrate: int = 9600, timeout: float = None, file_name: str = None):
    """
    Check the functionality of a TTL to serial converter.

    :param port: The serial port to which the TTL converter is connected (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux).
    :param baudrate: The baud rate for communication. Default is 9600.
    :param timeout: Read timeout value. Default is 1 second.
    """
    # open file to store results
    try:
        # Initialize serial connection
        ser = serial.Serial(port, baudrate, timeout=timeout, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, parity="E")
        print(f"Connected to {port} at {baudrate} baud.")

        while True:
            # Try decoding with error handling
            if ser.in_waiting > 0:
                response = ser.read_all()  # Read a line and strip newline characters
                save_to_file(response, file_name)


    except serial.SerialException as e:
        print(f"Serial exception: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Close the serial connection
        ser.close()
        print("Serial connection closed.")



# Example usage
if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description='Save data from the TTL to serial converter.')
    argparser.add_argument('--port', type=str, default='/dev/ttyUSB0', help='The serial port to which the TTL converter is connected (e.g., \'COM3\' on Windows or \'/dev/ttyUSB0\' on Linux).')
    argparser.add_argument('--baudrate', type=int, default=230400, help='The baud rate for communication. Default is 9600.')

    argparser.add_argument('--position_x', type=int, help='Set position.')
    argparser.add_argument('--position_y', type=int, help='Set position.')

    argparser.add_argument('--id2_pos_x', type=int, default=None, help='Set position.')
    argparser.add_argument('--id2_pos_y', type=int, default=None, help='Set position.')
    argparser.add_argument('--id3_pos_x', type=int, default=None, help='Set position.')
    argparser.add_argument('--id3_pos_y', type=int, default=None, help='Set position.')
    argparser.add_argument('--id4_pos_x', type=int, default=None, help='Set position.')
    argparser.add_argument('--id4_pos_y', type=int, default=None, help='Set position.')

    args = argparser.parse_args()

    file_name = f'{args.position_x}_{args.position_y}_{time.strftime("%Y-%m-%d_%H-%M-%S")}.txt'
    # get the current file directory

    file_dir = os.path.dirname(os.path.realpath(__file__))

    full_path = os.path.join(file_dir, "data", file_name)
    with open(full_path, 'w') as f:
        f.write(f"position_x: {args.position_x}, position_y: {args.position_y}\n")
        f.write(f"id2_pos_x: {args.id2_pos_x}, id2_pos_y: {args.id2_pos_y}\n")
        f.write(f"id3_pos_x: {args.id3_pos_x}, id3_pos_y: {args.id3_pos_y}\n")
        f.write(f"id4_pos_x: {args.id4_pos_x}, id4_pos_y: {args.id4_pos_y}\n")
        f.close()
    print(f"File directory: {file_dir}")

    # Replace 'COM3' with the appropriate port for your environment
    connect_and_save(port=args.port, baudrate=args.baudrate, file_name=full_path)
