#!/usr/bin/env python3

""" MIT License

Copyright (c) 2023 Group of Electronic Technology and Communications. University of A Coruna.

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
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from math import floor

class PointCloudProcessor(Node):
    """
    A ROS 2 node that processes point cloud data by buffering and filtering based on a grid size.

    Attributes:
        input_topic (str): Topic name for input point cloud data.
        output_topic (str): Topic name for output point cloud data.
        grid_size (float): Size of the grid for filtering points.
        new_points (list): List to store new points from the point cloud.
        new_tiles_with_points (list): List to store tiles with points.
    """

    def __init__(self, input_topic, output_topic, grid_size):
        """
        Initializes the PointCloudProcessor node, setting up subscriptions and publishers.

        Args:
            input_topic (str): Topic name for input point cloud data.
            output_topic (str): Topic name for output point cloud data.
            grid_size (float): Size of the grid for filtering points.
        """
        super().__init__('buffer_filter')
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.grid_size = grid_size
        self.new_points = []
        self.new_tiles_with_points = []

        self.create_subscription(PointCloud2, self.input_topic, self.point_cloud_callback, 10)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)

    def point_cloud_callback(self, msg: PointCloud2) -> None:
        """
        Callback function to process incoming point cloud data.

        Args:
            msg (PointCloud2): Incoming point cloud message.
        """
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            self.new_points.append(point)

        points_to_publish = []
        for new_point in self.new_points:
            tile_x = floor(new_point[0] / self.grid_size)
            tile_y = floor(new_point[1] / self.grid_size)
            new_tile = (tile_x, tile_y)

            if new_tile in self.new_tiles_with_points:
                points_to_publish.append(new_point)
            elif (tile_x + 1, tile_y) in self.new_tiles_with_points:
                points_to_publish.append(new_point)
            elif (tile_x - 1, tile_y) in self.new_tiles_with_points:
                points_to_publish.append(new_point)
            elif (tile_x, tile_y + 1) in self.new_tiles_with_points:
                points_to_publish.append(new_point)
            elif (tile_x, tile_y - 1) in self.new_tiles_with_points:
                points_to_publish.append(new_point)

        self.new_tiles_with_points = []
        self.new_points = []

        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            tile_x = floor(point[0] / self.grid_size)
            tile_y = floor(point[1] / self.grid_size)
            tile = (tile_x, tile_y)
            self.new_tiles_with_points.append(tile)

        if len(points_to_publish) > 0:
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id
            pointcloud = pc2.create_cloud_xyz32(header, points_to_publish)
            self.publisher.publish(pointcloud)


def main(args=None):
    """
    Main function to initialize and spin the PointCloudProcessor node.

    Args:
        args: Command line arguments passed to the node.
    """
    rclpy.init(args=args)

    input_topic = '/input_topic'
    output_topic = '/output_topic'
    grid_size = 1.0

    processor = PointCloudProcessor(input_topic, output_topic, grid_size)
    rclpy.spin(processor)

    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
