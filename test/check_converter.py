import serial
import time

def check_ttl_serial(port: str, baudrate: int = 9600, timeout: float = 1.0):
    """
    Check the functionality of a TTL to serial converter.

    :param port: The serial port to which the TTL converter is connected (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux).
    :param baudrate: The baud rate for communication. Default is 9600.
    :param timeout: Read timeout value. Default is 1 second.
    """
    try:
        # Initialize serial connection
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"Connected to {port} at {baudrate} baud.")

        # Test data to send
        test_data = "a\r"
        while True:
            # Send data
            ser.write(test_data.encode())
            print(f"Sent: {test_data}")
            # Give the device a moment to echo the data back or respond
            time.sleep(0.1)
            # Read response
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)

                print(f"Received: {response}")
            else:
                print("No response received.")
            time.sleep(1)

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
