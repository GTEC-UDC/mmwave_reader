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

"""
IWR6843ISK People Counting Reader Node

This module provides a ROS2 node for reading and processing data from the IWR6843ISK radar sensor
in people counting mode. It handles the communication with the radar, data parsing, and publishing
of various ROS2 messages including point clouds, radar scans, and target positions.

The node supports 3D people counting with configurable sensor position and detection boundaries.

Author: GTEC
Date: 2024
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import numpy as np


import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped, Point
from radar_msgs.msg import RadarScan, RadarReturn

from gtec_mmwave_reader.IWR6843ISK.tlv_parser import TLVParser 
from gtec_mmwave_reader.IWR6843ISK.tlv_uart_reader import TLVUartReader, LabId
import gtec_mmwave_reader.IWR6843ISK.rotation_helper as rot_helper

import threading

class RadarPose(object):
    """
    Represents the physical position and orientation of the radar sensor.
    
    Attributes:
        sensor_height (float): Height of the sensor from the ground
        sensor_x (float): X position of the sensor
        sensor_y (float): Y position of the sensor
        elev_tilt (float): Elevation tilt angle of the sensor
        rot_matrix (numpy.ndarray): 3x3 rotation matrix for sensor orientation
    """
    def __init__(self, sensor_height, sensor_x, sensor_y, elev_tilt, radar_pitch, radar_yaw, radar_roll):
        self.sensor_height = sensor_height
        self.sensor_x = sensor_x
        self.sensor_y = sensor_y
        self.elev_tilt = elev_tilt
        self.rot_matrix = rot_helper.euler_to_rotation_matrix(radar_yaw, radar_pitch, radar_roll)

class BoundaryBox(object):
    """
    Defines a 3D boundary box for radar detection area.
    
    Attributes:
        min_x (float): Minimum X coordinate
        min_y (float): Minimum Y coordinate
        min_z (float): Minimum Z coordinate
        max_x (float): Maximum X coordinate
        max_y (float): Maximum Y coordinate
        max_z (float): Maximum Z coordinate
    """
    def __init__(self, min_x, min_y, min_z, max_x, max_y, max_z):
        self.min_x = min_x
        self.min_y = min_y
        self.min_z = min_z
        self.max_x = max_x
        self.max_y = max_y
        self.max_z = max_z

class IWR6843ISKPeopleCountingReader(Node):
    """
    ROS2 node for reading and processing IWR6843ISK radar data in people counting mode.
    
    This node handles:
    - Communication with the radar sensor
    - Data parsing and processing
    - Publishing of various ROS2 messages:
        - RadarScan: Raw radar returns
        - PointCloud2: Point cloud data
        - PointStamped: Individual target positions
        - PointCloud2: All targets as a point cloud
    
    Attributes:
        uart_port (str): Path to the UART port for configuration
        data_port (str): Path to the UART port for data
        config_file_path (str): Path to the radar configuration file
        radar_id (str): Identifier for the radar
        uart_reader (TLVUartReader): UART reader instance
        radar_pose (RadarPose): Sensor position and orientation
        boundary_box (BoundaryBox): Detection boundary box
        publishers (dict): Dictionary of ROS2 publishers
    """

    def __init__(self):
        """
        Initialize the people counting reader node.
        
        Sets up:
        - ROS2 parameters
        - UART communication
        - Publishers
        - Configuration
        - Main processing loop
        """
        super().__init__('iwr6843isk_people_counting_reader')

        # Declare parameters
        self.declare_parameter('uart_port', '/dev/ttyUSB0')
        self.declare_parameter('data_port', '/dev/ttyUSB1')
        self.declare_parameter('config_file_path', 'config.cfg')
        self.declare_parameter('publish_radar_topic', 'radar_scan')
        self.declare_parameter('publish_target_topic', 'target')
        self.declare_parameter('publish_cloud_topic', 'cloud')
        self.declare_parameter('publish_all_target_topic', 'all_targets')
        self.declare_parameter('sensor_height', 0.0)
        self.declare_parameter('sensor_x', 0.0)
        self.declare_parameter('sensor_y', 0.0)
        self.declare_parameter('elev_tilt', 0.0)
        self.declare_parameter('radar_pitch', 0.0)
        self.declare_parameter('radar_yaw', 0.0)
        self.declare_parameter('radar_roll', 0.0)
        self.declare_parameter('radar_id', 'radar')
        self.declare_parameter('boundary_box_x_min', -10.0)
        self.declare_parameter('boundary_box_y_min', -10.0)
        self.declare_parameter('boundary_box_z_min', -10.0)
        self.declare_parameter('boundary_box_x_max', 10.0)
        self.declare_parameter('boundary_box_y_max', 10.0)
        self.declare_parameter('boundary_box_z_max', 10.0)

        # Get parameters
        uart_port = self.get_parameter('uart_port').get_parameter_value().string_value
        data_port = self.get_parameter('data_port').get_parameter_value().string_value
        config_file_path = self.get_parameter('config_file_path').get_parameter_value().string_value
        publish_radar_topic = self.get_parameter('publish_radar_topic').get_parameter_value().string_value
        publish_target_topic = self.get_parameter('publish_target_topic').get_parameter_value().string_value
        publish_cloud_topic = self.get_parameter('publish_cloud_topic').get_parameter_value().string_value
        publish_all_target_topic = self.get_parameter('publish_all_target_topic').get_parameter_value().string_value
        sensor_height = self.get_parameter('sensor_height').get_parameter_value().double_value
        sensor_x = self.get_parameter('sensor_x').get_parameter_value().double_value
        sensor_y = self.get_parameter('sensor_y').get_parameter_value().double_value
        elev_tilt = self.get_parameter('elev_tilt').get_parameter_value().double_value
        radar_pitch = self.get_parameter('radar_pitch').get_parameter_value().double_value
        radar_yaw = self.get_parameter('radar_yaw').get_parameter_value().double_value
        radar_roll = self.get_parameter('radar_roll').get_parameter_value().double_value
        radar_id = self.get_parameter('radar_id').get_parameter_value().string_value
        boundary_box_x_min = self.get_parameter('boundary_box_x_min').get_parameter_value().double_value
        boundary_box_y_min = self.get_parameter('boundary_box_y_min').get_parameter_value().double_value
        boundary_box_z_min = self.get_parameter('boundary_box_z_min').get_parameter_value().double_value
        boundary_box_x_max = self.get_parameter('boundary_box_x_max').get_parameter_value().double_value
        boundary_box_y_max = self.get_parameter('boundary_box_y_max').get_parameter_value().double_value
        boundary_box_z_max = self.get_parameter('boundary_box_z_max').get_parameter_value().double_value

        # Store configuration
        self.uart_port = uart_port
        self.data_port = data_port
        self.config_file_path = config_file_path
        self.radar_id = radar_id
        self.uart_reader = TLVUartReader(lab_id=LabId.PeopleCounting3D)

        # Create radar pose and boundary box
        self.radar_pose = RadarPose(sensor_height, sensor_x, sensor_y, elev_tilt, radar_pitch, radar_yaw, radar_roll)
        self.boundary_box = BoundaryBox(boundary_box_x_min, boundary_box_y_min, boundary_box_z_min,
                                      boundary_box_x_max, boundary_box_y_max, boundary_box_z_max)

        # Create publishers
        self.publisher_radar = self.create_publisher(RadarScan, publish_radar_topic, 10)
        self.publisher_cloud = self.create_publisher(PointCloud2, publish_cloud_topic, 10)
        self.publishers_target = []
        for n in range(8):
            self.publishers_target.append(
                self.create_publisher(PointStamped, f"{publish_target_topic}/target_{n}", 10))
        self.publisher_all_targets = self.create_publisher(PointCloud2, publish_all_target_topic, 10)

        # Setup and timer
        if self.connectCom():
            if self.sendConfigToDevice():
                self.get_logger().info("Radar connected and configured")
                # self.create_timer(0.1, self.loop)
                threading.Thread(target=self.read_loop, daemon=True).start()
            else:
                self.get_logger().error("Could not send configuration to radar")
        else:
            self.get_logger().error("Could not connect to radar")

    def read_loop(self):
        while rclpy.ok():
            try:
                hadFail, frameBytes, numTLVs, tlvHeaderLength, numDetectedObj = self.uart_reader.readAndParseUart()
                if not hadFail:
                    self.process_data(frameBytes, numTLVs, tlvHeaderLength, numDetectedObj)
            except Exception as e:
                self.get_logger().error(f"Error en lectura UART: {e}")

    def connectCom(self):
        """
        Connect to the radar's UART ports.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.uart_reader.connectComPorts(self.uart_port, self.data_port)
            return True
        except Exception as e:
            self.get_logger().error(f"Error connecting to COM ports: {e}")
            return False

    def sendConfigToDevice(self):
        """
        Send configuration to the radar device.
        
        Returns:
            bool: True if configuration successful, False otherwise
        """
        try:
            with open(self.config_file_path, 'r') as cfg_file:
                cfg = cfg_file.readlines()
            self.uart_reader.sendCfgOverwriteSensorPosition(cfg, self.radar_pose, self.boundary_box)
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending config to device: {e}")
            return False

    def process_data(self, frameBytes, numTLVs, tlvHeaderLength, numDetectedObj):
        """
        Main processing loop.
        
        This method:
        1. Reads and parses data from the radar
        2. Processes the data into appropriate formats
        3. Publishes the processed data as ROS2 messages:
           - RadarScan for raw returns
           - PointCloud2 for point cloud data
           - PointStamped for individual targets
           - PointCloud2 for all targets
        """
        #hadFail, frameBytes, numTLVs, tlvHeaderLength, numDetectedObj = self.uart_reader.readAndParseUart()
        
        # if not hadFail:
        parser = TLVParser(
            labType=self.uart_reader.labId,
            num_azimuth_antennas=8,
            num_range_bins=256,
            num_doppler_bins=16
        )
        data = parser.parseMsg(frameBytes, numTLVs, tlvHeaderLength, numDetectedObj)

        polarPoints = data[0]
        cartesianPoints = data[1]
        targets = data[2]
        indexes = data[3]
        numPoints = data[4]
        numTargets = data[5]
        fail = data[7]

        if not fail:
            current_time = self.get_clock().now().to_msg()
            returns = []
            ca_points = []

            for i in range(numPoints):
                ranging = polarPoints[0][i]
                azim = polarPoints[1][i]
                elevation = polarPoints[2][i]
                doppler = polarPoints[3][i]
                snr = polarPoints[4][i]

                radar_return = RadarReturn(range=ranging, azimuth=azim, elevation=elevation, 
                                        doppler_velocity=doppler, amplitude=snr)
                returns.append(radar_return)

                ca_point = [cartesianPoints[0][i], cartesianPoints[1][i], cartesianPoints[2][i],
                            cartesianPoints[3][i], cartesianPoints[4][i]]
                ca_points.append(ca_point)

            if numPoints > 0:
                # Publish RadarScan
                header_radar_scan = Header(stamp=current_time, frame_id=self.radar_id)
                radar_scan = RadarScan(header=header_radar_scan, returns=returns)
                self.publisher_radar.publish(radar_scan)

                # Publish PointCloud2
                header_point_cloud = Header(stamp=current_time, frame_id=self.radar_id)
                fields_point_cloud = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='snr', offset=12, datatype=PointField.FLOAT32, count=1),
                    PointField(name='doppler', offset=16, datatype=PointField.FLOAT32, count=1)
                ]
                point_cloud_ti = pc2.create_cloud(header_point_cloud, fields_point_cloud, ca_points)
                self.publisher_cloud.publish(point_cloud_ti)

            if numTargets > 0:
                header_target = Header(stamp=current_time, frame_id=f"target_{self.radar_id}")
                targets_cloud_points = []

                for n in range(numTargets):
                    target_point = rot_helper.Point(targets[1,n], targets[2,n], targets[3,n])
                    point_rotated = rot_helper.apply_rotation_matrix(target_point, self.radar_pose.rot_matrix)
                    
                    target_pos = PointStamped()
                    target_pos.header = header_target
                    target_pos.point = Point(x=point_rotated.x, y=point_rotated.y, z=point_rotated.z)
                    self.publishers_target[n].publish(target_pos)
                    
                    targets_cloud_points.append([point_rotated.x, point_rotated.y, point_rotated.z])

                # Publish all targets as PointCloud2
                header_target_cloud = Header(stamp=current_time, frame_id="odom")
                targets_fields_cloud = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
                ]
                targets_cloud = pc2.create_cloud(header_target_cloud, targets_fields_cloud, targets_cloud_points)
                self.publisher_all_targets.publish(targets_cloud)

def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    
    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)
    node = IWR6843ISKPeopleCountingReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()