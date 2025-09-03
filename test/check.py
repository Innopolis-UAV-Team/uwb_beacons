import time
import serial
import struct

class Message:
    def __init__(self, data: bytes):
        self.id =  struct.unpack('B', data[0:1])[0]
        self.data = 0
        self.data = struct.unpack('<H', data[1:3])[0]  # '<H' denotes little-endian, uint16_t

    def __str__(self):
        return f"id: {self.id}, data: {self.data}"

def decode(data: bytes) -> int:
    # first byte is id
    msgs = data.split(b'\xFF\x00')
    i = 0
    for d in msgs:
        print(Message(d))
        i += 1
    return i

class CircularBuffer:
    def __init__(self, size: int):
        self.buffer = [b'\x00'for i in range(size)]
        self.head = 0
        self.size = 0
        self.tail = 0

    def append(self, item: bytes | None):
        if item is None:
            return
        items = item.split(b'\xff\x00')
        for item in items:
            if len(item) == 0:
                continue
            self.buffer[self.head] = item
            self.head = (self.head + 1) % len(self.buffer)
            self.size += 1
        if self.size > len(self.buffer):
            self.size = len(self.buffer)

    def pop(self):
        if self.size == 0:
            return None
        item = self.buffer[self.tail]
        self.tail = (self.tail + 1) % len(self.buffer)
        self.size -= 1
        return item

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
    finally:
        # Close the serial connection
        ser.close()
        print("Serial connection closed.")

# Example usage
if __name__ == "__main__":
    check_ttl_serial(port='/dev/ttyUSB0', baudrate=460800)
