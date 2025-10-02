#!/usr/bin/env python3
import rospy
import serial
import struct
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String
from scipy.optimize import least_squares

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "uwb_beacons", "test"))

from common.algoritms import calibrated_linear


def calibrated_linear(raw, a, b):
    return a * raw + b

def calibrated_quadratic(raw, a, b, c):
    return a * raw**2 + b * raw + c

def calibrated_cubic(raw, a, b, c, d):
    return a * raw**3 + b * raw**2 + c * raw + d

class UWBLocalizer:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = rospy.get_param("~baud", 115200)
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.anchor_positions = rospy.get_param("~anchors", {
            "1": [0.0, 0.0, 0.0],
            "2": [3.0, 0.0, 0.0],
            "3": [0.0, 3.0, 0.0],
            "4": [3.0, 3.0, 2.0],
        })
        self.calib_type = rospy.get_param("~calibration", "linear")
        self.calib_params = rospy.get_param("~calib_params", [1.0, 0.0])
        self.z_sign = rospy.get_param("~z_sign", 1)

        self.ser = serial.Serial(port, baud, timeout=1)

        self.pose_pub = rospy.Publisher("uwb/pose", PoseStamped, queue_size=10)
        self.range_pub = rospy.Publisher("uwb/ranges", Range, queue_size=10)
        self.debug_pub = rospy.Publisher("uwb/debug", String, queue_size=10)

    def parse_message(self, data):
        if len(data) < 5:
            return None, None
        anchor_id = data[0]
        raw_val = struct.unpack("<I", data[1:5])[0]
        return anchor_id, raw_val

    def calibrate(self, raw):
        if self.calib_type == "linear":
            return calibrated_linear(raw, *self.calib_params)
        elif self.calib_type == "quadratic":
            return calibrated_quadratic(raw, *self.calib_params)
        elif self.calib_type == "cubic":
            return calibrated_cubic(raw, *self.calib_params)
        else:
            return raw

    def multilateration(self, ranges):
        anchors = []
        dists = []
        for aid, dist in ranges.items():
            if str(aid) in self.anchor_positions:
                anchors.append(self.anchor_positions[str(aid)])
                dists.append(dist)
        if len(anchors) < 3:
            return None
        anchors = np.array(anchors)
        dists = np.array(dists)
        x0 = np.mean(anchors, axis=0)
        def residuals(x):
            return np.linalg.norm(anchors - x, axis=1) - dists
        res = least_squares(residuals, x0)
        return res.x

    def spin(self):
        buffer = bytearray()
        ranges = {}
        while not rospy.is_shutdown():
            b = self.ser.read(1)
            if not b:
                continue
            buffer.extend(b)
            if buffer.endswith(b"\xff\xff\xff\x00"):
                packet = buffer[:-4]
                buffer.clear()
                anchor_id, raw_val = self.parse_message(packet)
                if anchor_id is None:
                    continue
                dist = self.calibrate(raw_val)
                ranges[anchor_id] = dist

                msg = Range()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = f"anchor_{anchor_id}"
                msg.radiation_type = Range.INFRARED
                msg.field_of_view = 0.1
                msg.min_range = 0.0
                msg.max_range = 100.0
                msg.range = dist
                self.range_pub.publish(msg)

                pos = self.multilateration(ranges)
                if pos is not None:
                    pose = PoseStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = self.frame_id
                    pose.pose.position.x = pos[0]
                    pose.pose.position.y = pos[1]
                    pose.pose.position.z = pos[2] * self.z_sign
                    self.pose_pub.publish(pose)

                self.debug_pub.publish(f"Anchor {anchor_id}, raw={raw_val}, dist={dist:.3f}")

if __name__ == "__main__":
    rospy.init_node("uwb_localizer")
    node = UWBLocalizer()
    node.spin()
