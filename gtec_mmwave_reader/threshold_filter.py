#!/usr/bin/env python3

""" MIT License

Copyright (c) 2020 Group of Electronic Technology and Communications. University of A Coruna.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. """


import rclpy
from rclpy.node import Node
import time
import serial
import os
import tf2_ros
import csv
import math
from std_msgs.msg import Header

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, Point


class ThresholdFilter(Node):
    """
    A ROS 2 node that filters point cloud data based on SNR and Doppler values.

    Attributes:
        do_filter_snr (bool): Flag to enable SNR filtering.
        do_filter_doppler (bool): Flag to enable Doppler filtering.
        publisher_filtered (Publisher): Publisher for filtered point cloud data.
        min_snr (float): Minimum SNR value for filtering.
        max_snr (float): Maximum SNR value for filtering.
        min_doppler (float): Minimum Doppler value for filtering.
        max_doppler (float): Maximum Doppler value for filtering.
    """

    def __init__(self):
        """
        Initializes the ThresholdFilter node, setting up parameters and subscriptions.
        """
        super().__init__('threshold_filter')
        self.do_filter_snr = False
        self.do_filter_doppler = False

        raw_measurements_topic = self.declare_parameter('raw_measurements_topic', '/raw_measurements').get_parameter_value().string_value
        publish_topic = self.declare_parameter('publish_filtered_topic', '/filtered_measurements').get_parameter_value().string_value

        self.publisher_filtered = self.create_publisher(PointCloud2, publish_topic, 100)

        self.do_filter_snr = self.declare_parameter('do_filter_snr', False).get_parameter_value().bool_value
        self.do_filter_doppler = self.declare_parameter('do_filter_doppler', False).get_parameter_value().bool_value

        self.min_snr = self.declare_parameter('min_snr', 0.0).get_parameter_value().double_value
        self.max_snr = self.declare_parameter('max_snr', 0.0).get_parameter_value().double_value
        self.min_doppler = self.declare_parameter('min_doppler', 0.0).get_parameter_value().double_value
        self.max_doppler = self.declare_parameter('max_doppler', 0.0).get_parameter_value().double_value

        self.create_subscription(PointCloud2, raw_measurements_topic, self.filter_measurement, 10)

        if self.do_filter_snr:
            self.filter_snr(self.min_snr, self.max_snr)

        if self.do_filter_doppler:
            self.filter_doppler(self.min_doppler, self.max_doppler)

    def filter_snr(self, min_snr: float, max_snr: float) -> None:
        """
        Enable SNR filtering with specified minimum and maximum values.

        Args:
            min_snr (float): Minimum SNR value.
            max_snr (float): Maximum SNR value.
        """
        self.do_filter_snr = True
        self.min_snr = min_snr
        self.max_snr = max_snr

    def filter_doppler(self, min_doppler: float, max_doppler: float) -> None:
        """
        Enable Doppler filtering with specified minimum and maximum values.

        Args:
            min_doppler (float): Minimum Doppler value.
            max_doppler (float): Maximum Doppler value.
        """
        self.do_filter_doppler = True
        self.min_doppler = min_doppler
        self.max_doppler = max_doppler

    def filter_measurement(self, cloud_points_msg: PointCloud2) -> None:
        """
        Filters incoming point cloud data based on SNR and Doppler values.

        Args:
            cloud_points_msg (PointCloud2): Incoming point cloud message.
        """
        filtered_points = []

        for point in pc2.read_points(cloud_points_msg, field_names=("x", "y", "z", "snr", "doppler"), skip_nans=True):
            point_is_valid = True

            if self.do_filter_snr and point_is_valid:
                if point[3] < self.min_snr or point[3] > self.max_snr:
                    point_is_valid = False
                    self.get_logger().info(f'SNR filtered: {point[3]}')

            if self.do_filter_doppler and point_is_valid:
                if abs(point[4]) < self.min_doppler or abs(point[4]) > self.max_doppler:
                    point_is_valid = False
                    self.get_logger().info(f'Doppler filtered: {point[4]}')

            if point_is_valid:
                filtered_points.append(point)

        if filtered_points:
            filtered_cloud_points_msg = pc2.create_cloud(cloud_points_msg.header, cloud_points_msg.fields, filtered_points)
            self.publisher_filtered.publish(filtered_cloud_points_msg)


def main(args=None):
    """
    Main function to initialize and spin the ThresholdFilter node.

    Args:
        args: Command line arguments passed to the node.
    """
    rclpy.init(args=args)
    threshold_filter = ThresholdFilter()
    rclpy.spin(threshold_filter)
    threshold_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
