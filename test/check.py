import time
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
        n = 0
        # Test data to send
        buffer: CircularBuffer = CircularBuffer(100)
        while True:
            # Try decoding with error handling
            if ser.in_waiting > 0:
                try:
                    response = ser.read_until(b'\xFF\x00')
                    buffer.append(response)
                    if buffer.size != 0:
                        last_message = time.time()
                    else :
                        continue
                    for i in range(buffer.size):
                        msg = buffer.pop()
                        if msg is None:
                            continue
                        print(Message(msg))
                        n+=1
                    if (n > 100):
                        print(f"Time: {time.time() - last_message}")
                        n = 0
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
    argparser.add_argument('--port', type=str, default='/dev/ttyUSB0', help='The serial port to which the TTL converter is connected (e.g., \'COM3\' on Windows or \'/dev/ttyUSB0\' on Linux).')
    argparser.add_argument('--baudrate', type=int, default=460800  , help='The baud rate for communication. Default is 9600.')

    args = argparser.parse_args()
    check_ttl_serial(port=args.port, baudrate=args.baudrate)
