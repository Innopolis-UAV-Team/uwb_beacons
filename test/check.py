import binascii
import serial
import time

def check_ttl_serial(port: str, baudrate: int = 9600, timeout: float = None):
    """
    Check the functionality of a TTL to serial converter.

    :param port: The serial port to which the TTL converter is connected (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux).
    :param baudrate: The baud rate for communication. Default is 9600.
    :param timeout: Read timeout value. Default is 1 second.
    """

    try:
        # Initialize serial connection
        ser = serial.Serial(port, baudrate, timeout=timeout, stopbits=serial.STOPBITS_ONE, bytesize=serial.SEVENBITS, parity="N")
        print(f"Connected to {port} at {baudrate} baud.")

        # Test data to send
        test_data = "Hello, TTL!"
        # encodings = ['ascii', 'latin-1', 'utf-8', 'utf-16', 'utf-32']
        # encodings = ['utf-16']
        encodings = ['ascii']
        i = 0
        ser.write("Hello, TTL!".encode('ascii'))

        while True:
            encoding = encodings[i % len(encodings)]

            # Try decoding with error handling
            if ser.in_waiting > 0:
                response = ser.read_all()  # Read a line and strip newline characters
                # print(f"Raw response: {binascii.hexlify(response)}")
                try:
                    message = response.decode(encoding)
                    # message = response.decode(encoding).strip()
                    print(f"Received message: {message}")
                except UnicodeDecodeError as e:
                    print(f"An error occurred: {e}")
            i += 1



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
    # Replace 'COM3' with the appropriate port for your environment
    check_ttl_serial(port='/dev/ttyUSB0', baudrate=230400)
