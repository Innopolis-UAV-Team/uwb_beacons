import os
import struct
import sys
from typing import List, Tuple
import serial
import time
import argparse
from common.serial_messages import Message, CircularBuffer, decode
from common.algoritms import calibrated_qubic

num_entries = 0
#a = 3.45936080e-05
#b = -2.47483248e-03
#c = 4.62066120e-02
#d = 1.93440057e-01

a = 2.17754063e-07
b = -6.50749910e-04  
c = 2.60398686e-02
d = 3.62156905e-01

def connect_and_save(port: str, baudrate: int = 9600, timeout: float|None = None,
                        file_name: str = "output.txt", num_points: int = 1000):
    """
    Check the functionality of a TTL to serial converter.

    :param port: The serial port to which the TTL converter is connected (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux).
    :param baudrate: The baud rate for communication. Default is 9600.
    :param timeout: Read timeout value. Default is 1 second.
    """
    # open file to store results
    response: bytes = b''
    try:
        # Initialize serial connection
        ser = serial.Serial(port, baudrate, timeout=timeout, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, parity="E")
        print(f"Connected to {port} at {baudrate} baud.")
        max_size = 100
        buffer: CircularBuffer = CircularBuffer(max_size)
        point_count = 0
        last_info_time = time.time()
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
                                if msg is None:
                                    continue
                                parsed_message = Message(msg)
                                calibrated_distance = (parsed_message.data/1000.0 - calibrated_qubic(a, b, c, d, parsed_message.data/1000.0))*1000
                                f.write(str(Message(msg)) + ' calibrated: ' + str(calibrated_distance) + '\n')
                            f.close()
                except struct.error as e:
                    print(e)
                    print(response)
            if time.time() - last_info_time > 1:
                last_info_time = time.time()
                print(f"Received {point_count}\tpoints out of {num_points}")

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
    argparser.add_argument('-p', '--port', type=str, default='/dev/ttyUSB0', help='The serial port to which the TTL converter is connected (e.g., \'COM3\' on Windows or \'/dev/ttyUSB0\' on Linux).')
    argparser.add_argument('-b', '--baudrate', type=int, default=230400, help='The baud rate for communication. Default is 230400.')
    argparser.add_argument('-n', '--num_points', type=int, default=1000, help='Number of points to collect before stopping. Default is 1000.')
    argparser.add_argument('-d', '--out_dir', type=str, default='data', help='Directory to save the data to. Default is "data".')
    argparser.add_argument('-r', '--real_distance', type=int, help='Real distance to tag (for error calculation).')

    args = argparser.parse_args()

    args = argparser.parse_args()

    file_name = f'{args.num_points}_{time.strftime("%Y-%m-%d_%H-%M-%S")}.txt'
    # get the current file directory

    output_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), args.out_dir)
    if args.real_distance is not None:
        real_distance = args.real_distance
        print(f"Real distance: {real_distance}")
        output_dir = os.path.join(output_dir, str(real_distance))
    if os.path.exists(output_dir):
        print(f"Data directory {output_dir} exists.")
    else:
        os.mkdir(output_dir)
    
    full_path = os.path.join(output_dir, file_name)
    print(f"The output file will be saved to {full_path}")
    # Replace 'COM3' with the appropriate port for your environment
    connect_and_save(port=args.port, baudrate=args.baudrate,
                        file_name=full_path, num_points=args.num_points)
