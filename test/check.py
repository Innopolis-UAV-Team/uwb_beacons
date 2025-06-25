import serial
import struct

class Message:
    def __init__(self, data: bytes):
        self.id =  struct.unpack('B', data[0:1])[0]
        self.data = 0
        self.data = struct.unpack('<H', data[1:3])[0]  # '<H' denotes little-endian, uint16_t

    def __str__(self):
        return f"id: {self.id}, data: {self.data}"

def decode(data: bytes) -> str:
    # first byte is id
    print(list(data))
    data = data.split(b'\xFF\x00')
    for d in data:
        print(Message(d))

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
        while True:
            # Try decoding with error handling
            if ser.in_waiting > 0:
                response = ser.read_all()  # Read a line and strip newline characters
                decode(response)


    except serial.SerialException as e:
        print(f"Serial exception: {e}")

    finally:
        # Close the serial connection
        ser.close()
        print("Serial connection closed.")

# Example usage
if __name__ == "__main__":
    check_ttl_serial(port='/dev/ttyUSB0', baudrate=230400)
