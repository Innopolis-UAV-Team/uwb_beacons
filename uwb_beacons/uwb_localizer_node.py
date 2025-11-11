#!/usr/bin/env python3
from typing import Iterable
import rclpy
from rclpy.node import Node
import serial
import struct
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String
from scipy.optimize import least_squares
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from uwb_beacons.algoritms import multilateration
from uwb_beacons.serial_messages import CircularBuffer, Message
from uwb_beacons.parameters import DEFAULT_PARAMS

def calibrated_linear(raw, a, b):
    return a * raw + b

def calibrated_quadratic(raw, a, b, c):
    return a * raw**2 + b * raw + c

def calibrated_cubic(raw, a, b, c, d):
    return a * raw**3 + b * raw**2 + c * raw + d

class UWBLocalizer(Node):
    def __init__(self):
        super().__init__('uwb_beacons')
        self.__declare_parameters()
        self.declare_parameters(namespace='', parameters=self.parameters)
        self.__list_params()
        self.get_logger().info(f'Loaded parameters')
        # Get parameter values
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.anchor_positions = {}
        for anchor_name in self.get_parameter('anchors_names').get_parameter_value().string_array_value:
            self.anchor_positions[anchor_name] = self.get_parameter(anchor_name).get_parameter_value().double_array_value
        if len(self.anchor_positions) < 3:
            self.get_logger().error('At least 3 anchor positions are required for trilateration')
            raise ValueError('Insufficient anchor positions')

        self.calib_type = self.get_parameter('calibration').get_parameter_value().string_value
        self.calib_params = self.get_parameter('calib_params').get_parameter_value().double_array_value
        self.z_sign = self.get_parameter('z_sign').get_parameter_value().integer_value
        timer_frequency = self.get_parameter('timer_frequency').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.field_of_view = self.get_parameter('field_of_view').get_parameter_value().double_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.publication_period = 1 / self.get_parameter('publication_frequency').get_parameter_value().double_value
        # Anchor positions are already set above

        # Validate parameters
        self._validate_parameters()
        
        # Create publishers
        self.pose_pub = self.create_publisher(PoseStamped, 'uwb/pose', 10)
        self.range_pub = self.create_publisher(Range, 'uwb/ranges', 10)
        self.debug_pub = self.create_publisher(String, 'uwb/debug', 10)

        self.get_logger().info(f'Anchor positions: {self.anchor_positions}')
        self.get_logger().info(f'Calibration type: {self.calib_type}, params: {self.calib_params}')
        self.debug_pub.publish(String(data=f"Anchor positions: {self.anchor_positions}"))
        self.debug_pub.publish(String(data=f"Calibration type: {self.calib_type}, params: {self.calib_params}"))
        self.debug_pub.publish(String(data='No data received'))

        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
            self.get_logger().info(f'Connected to UWB device on {port} at {baud} baud')
            self.debug_pub.publish(String(data=f'Connected to UWB device on {port} at {baud} baud'))
        except Exception as e:
            self.get_logger().info(f'Failed to connect to UWB device: {e}')
            self.debug_pub.publish(String(data=f'Failed to connect to UWB device: {e}'))
            raise

        # Create timer for main loop
        timer_period = 1.0 / timer_frequency
        self.timer = self.create_timer(timer_period, self.spin_once)

        # Initialize buffer and ranges
        # self.buffer = bytearray()
        self.ranges = {}
        self.last_msg_time = {}
        self.last_publication_time = 0.0
        self.buffer = CircularBuffer(100)

    def __parse_parameter(self, param_name, default_value):
        descriptor = None
        if   isinstance(default_value, dict):
            for key, value in default_value.items():
                self.__parse_parameter(f"{param_name}.{key}", value)
        if   isinstance(default_value, Iterable):
            if isinstance(default_value[0], str):
                descriptor = ParameterDescriptor(description=f"Parameter {param_name}",
                                                    type=ParameterType.PARAMETER_STRING_ARRAY)
            elif isinstance(default_value[0], int):
                descriptor = ParameterDescriptor(description=f"Parameter {param_name}",
                                                    type=ParameterType.PARAMETER_INTEGER_ARRAY)
            else:
                descriptor = ParameterDescriptor(description=f"Parameter {param_name}",
                                                    type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        elif isinstance(default_value, str):
            descriptor = ParameterDescriptor(description=f"Parameter {param_name}",
                                                type=ParameterType.PARAMETER_STRING)
        elif isinstance(default_value, int):
            descriptor = ParameterDescriptor(description=f"Parameter {param_name}",
                                                type=ParameterType.PARAMETER_INTEGER)
        else:
            descriptor = ParameterDescriptor(description=f"Parameter {param_name}",
                                                type=ParameterType.PARAMETER_DOUBLE)
        self.parameters.append((param_name, default_value, descriptor))

    def __declare_parameters(self):
        """Declare parameters"""
        self.parameters = []
        for param_name, default_value in DEFAULT_PARAMS.items():
            self.__parse_parameter(param_name, default_value)

    def _validate_parameters(self):
        """Validate parameter values"""
        # Validate calibration type
        valid_calib_types = ['linear', 'quadratic', 'cubic', 'none']
        if self.calib_type not in valid_calib_types:
            self.get_logger().warn(f'Invalid calibration type: {self.calib_type}. Using linear.')
            self.calib_type = 'linear'
        
        # Validate calibration parameters based on type
        if self.calib_type == 'linear' and len(self.calib_params) != 2:
            self.get_logger().warn('Linear calibration requires 2 parameters. Using defaults.')
            self.calib_params = [1.0, 0.0]
        elif self.calib_type == 'quadratic' and len(self.calib_params) != 3:
            self.get_logger().warn('Quadratic calibration requires 3 parameters. Using defaults.')
            self.calib_params = [0.0, 1.0, 0.0]
        elif self.calib_type == 'cubic' and len(self.calib_params) != 4:
            self.get_logger().warn('Cubic calibration requires 4 parameters. Using defaults.')
            self.calib_params = [0.0, 0.0, 1.0, 0.0]

        # Validate anchor positions
        if len(self.anchor_positions) < 3:
            self.get_logger().error('At least 3 anchor positions are required for trilateration')
            raise ValueError('Insufficient anchor positions')

        # Validate range parameters
        if self.min_range >= self.max_range:
            self.get_logger().warn('min_range should be less than max_range. Adjusting.')
            self.max_range = self.min_range + 1.0

        # Validate publication frequency
        if self.publication_period <= 0:
            self.get_logger().warn('publication_frequency should be greater than 0. Adjusting.')
            self.publication_period = 0.1

    def __list_params(self):
        """List all parameters"""
        for param_name, default_value, descriptor in self.parameters:
            value = self.get_parameter(param_name).get_parameter_value()
            print(f'{descriptor}\n\t{param_name}: {value}\n')

    def parse_message(self, data: bytearray) -> tuple[int, float]|tuple[None, None]:
        if len(data) < 5:
            return None, None
        anchor_id = data[0]
        raw_val = struct.unpack("<I", data[1:5])[0] / 1000.0  # Convert mm to meters
        return anchor_id, raw_val

    def calibrate(self, raw: float) -> float:
        if self.calib_type == "linear":
            return calibrated_linear(raw, *self.calib_params)
        elif self.calib_type == "quadratic":
            return calibrated_quadratic(raw, *self.calib_params)
        elif self.calib_type == "cubic":
            return calibrated_cubic(raw, *self.calib_params)
        else:
            return raw

    def multilaterate(self, ranges):
        if len(ranges) < 3:
            return None
        try:
            return multilateration(ranges, self.anchor_positions, self.z_sign)
        except ValueError as e:
            self.get_logger().error(f'Error in multilateration: {e}')
            return None

    def get_current_time(self):
        return self.get_clock().now().to_msg().sec

    def publish_ranges(self):
        for anchor_id, dist in self.ranges.items():
            if dist is None:
                continue
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"anchor_{anchor_id}"
            msg.field_of_view = self.field_of_view
            msg.min_range = self.min_range
            msg.max_range = self.max_range
            msg.range = dist
            self.range_pub.publish(msg)

    def spin_once(self):
        try:
            # Read available data
            for id, time in self.last_msg_time.items():
                if time < self.get_current_time() - self.publication_period * 2:
                    self.ranges[id] = None
                    self.last_msg_time[id] = 0.0

            if self.ser.in_waiting > 0:
                response = self.ser.read_until(b'\xFF\xFF\xFF\x00')
                self.buffer.append(response)
                if self.buffer.size == 0:
                    return
                for i in range(self.buffer.size):
                    msg = self.buffer.pop()
                    if msg is None:
                        continue
                    message = Message(msg)
                    anchor_id = message.id
                    raw_val = message.data / 1000.0  # Convert mm to meters
                    self.last_msg_time[anchor_id] = self.get_current_time()

                    if anchor_id is None:
                        continue

                    dist = self.calibrate(raw_val)
                    self.ranges[anchor_id] = dist
                    if dist < self.min_range or dist > self.max_range:
                        self.get_logger().warn(f'Range {dist} from anchor {anchor_id} out of bounds ({self.min_range}, {self.max_range})')
                        self.ranges[anchor_id] = None
                        continue
                    # Publish debug message
                    debug_msg = String()
                    debug_msg.data = f'Anchor {anchor_id}: raw {raw_val:.3f} m, calibrated {dist:.3f} m'
                    self.debug_pub.publish(debug_msg)
                    # self.get_logger().info(f'Range {dist} from anchor {anchor_id}')
            if self.get_clock().now().to_msg().sec - self.last_publication_time < self.publication_period:
                return
            self.publish_ranges()
            self.last_publication_time = self.get_clock().now().to_msg().sec

            # Calculate and publish position
            pos = self.multilaterate(self.ranges)
            if pos is not None:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = self.frame_id
                pose.pose.position.w = 1.0
                pose.pose.position.x = pos[0]
                pose.pose.position.y = pos[1]
                pose.pose.position.z = pos[2] * self.z_sign
                self.pose_pub.publish(pose)


        except Exception as e:
            self.get_logger().error(f'Error in spin_once: {e}')

def main(args=None):
    rclpy.init(args=args)

    try:
        node = UWBLocalizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
