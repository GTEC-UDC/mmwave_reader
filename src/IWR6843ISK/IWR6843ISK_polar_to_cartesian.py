#!/usr/bin/env python

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


import configparser
import rospy
import time
import serial
import os
import tf2_ros
import csv
import math
from std_msgs.msg import Header
from sklearn.cluster import OPTICS, cluster_optics_dbscan
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from oob_parser import uartParserSDK

from radar_msgs.msg import RadarScan, RadarReturn
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, Point


class IWR6843ISKPolarToCartesian(object):

    def __init__(self, publisher_cloud):
        self.publisher_cloud = publisher_cloud

    def radar_listener(self, radarScan):
        polarPoints = radarScan.returns
        numPoints = len(polarPoints)
        cartesianPoints = np.zeros((numPoints,5))
        for i in range(numPoints):
            cartesianPoints[i]  = self.polar_to_cartesian(polarPoints[i])
        
        header_point_cloud = Header()
        header_point_cloud.stamp = rospy.Time.now()
        header_point_cloud.frame_id = "radar"
        fields_point_cloud =  [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('snr', 12, PointField.FLOAT32, 1 ),
            PointField('doppler', 16, PointField.FLOAT32, 1 )]

        point_cloud_2 = pc2.create_cloud(header_point_cloud, fields_point_cloud, cartesianPoints)
        self.publisher_cloud.publish(point_cloud_2)

    def polar_to_cartesian(self, polarPoint):

        x = polarPoint.range*math.cos(polarPoint.elevation)*math.cos(polarPoint.azimuth)
        y = polarPoint.range*math.cos(polarPoint.elevation)*math.sin(polarPoint.azimuth)
        z = polarPoint.range*math.sin(polarPoint.elevation)
        # x = polarPoint.range*math.cos(polarPoint.elevation)*math.sin(polarPoint.azimuth)
        # y = polarPoint.range*math.cos(polarPoint.elevation)*math.cos(polarPoint.azimuth)
        # z = polarPoint.range*math.sin(polarPoint.elevation)
        # x = polarPoint.range*math.cos(polarPoint.elevation)*math.sin(polarPoint.azimuth)
        # y = polarPoint.range*math.sin(polarPoint.elevation)*math.sin(polarPoint.azimuth)
        # z = polarPoint.range*math.cos(polarPoint.azimuth)

        cartesianPoint = [x,y,z, polarPoint.amplitude, polarPoint.doppler_velocity]
        return cartesianPoint


if __name__ == "__main__":

    rospy.init_node('IWR6843ISKPolarToCartesian', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Read parameters

    radar_topic = rospy.get_param('~radar_topic')
    publish_cloud_topic = rospy.get_param('~publish_cloud_topic')
    pub_cloud = rospy.Publisher(publish_cloud_topic, PointCloud2, queue_size=100)

    polarToCartesian = IWR6843ISKPolarToCartesian(pub_cloud)

    rospy.Subscriber(radar_topic, RadarScan,
                     polarToCartesian.radar_listener)

    print("=========== GTEC mmWave IWR6843ISK Polar to Radar ============")

    rospy.spin()
