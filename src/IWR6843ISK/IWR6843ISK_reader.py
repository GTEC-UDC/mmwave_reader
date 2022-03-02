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


class IWR6843ISKReader(object):

    def __init__(self, uart_port, data_port, config_file_path, publisher_radar, publisher_cloud, publisher_target):
        self.uart_port = uart_port
        self.data_port = data_port
        self.parser = uartParserSDK(type='3D People Counting')
        self.publisher_radar = publisher_radar
        self.publisher_cloud = publisher_cloud
        self.publisher_target = publisher_target
        self.config_file_path = config_file_path

    def connectCom(self):
        try:
            uart = self.uart_port
            data = self.data_port
            self.parser.connectComPorts(uart, data)
            return True
        except Exception as e:
            print(e)
            return False

    def sendConfigToDevice(self):
        try:
            cfg_file = open(self.config_file_path, 'r')
            cfg = cfg_file.readlines()
            self.parser.sendCfg(cfg)
            return True
        except Exception as e:
            print(e)
            return False

    def loop(self):
        data = self.parser.readAndParseUart()
        classifierOutput = []
        polarPoints = data[0]
        pointCloud = data[1]
        targets = data[2]
        indexes = data[3]
        numPoints = data[4]
        numTargets = data[5]
        frameNum = data[6]
        fail = data[7]
        classifierOutput = data[8]

        #print('Num Points: ', numPoints)
        #print('Points:', pointCloud)


        cartesianPoints = np.zeros((numPoints,4))
        returns = []

        for i in range(numPoints):
            ranging = polarPoints[0][i]
            azim = polarPoints[1][i]
            elevation = polarPoints[2][i]
            doppler = polarPoints[3][i]
            snr = polarPoints[4][i]

            x = polarPoints[0,i]*math.cos(polarPoints[2,i])*math.sin(polarPoints[1,i])
            y = polarPoints[0,i]*math.cos(polarPoints[2,i])*math.cos(polarPoints[1,i])
            z = polarPoints[0,i]*math.sin(polarPoints[2,i])

            cartesianPoints[i] = [x,y,z, snr]

            radar_return = RadarReturn(range=ranging, azimuth=azim, elevation=elevation, doppler_velocity=doppler, amplitude=snr)
            returns.append(radar_return)

        current_time = rospy.Time.now()

        if numPoints>0:
            
            header_radar_scan = Header()
            header_radar_scan.stamp = current_time
            header_radar_scan.frame_id = "map"
            radar_scan = RadarScan(header_radar_scan, returns)
            self.publisher_radar.publish(radar_scan)


            header_point_cloud = Header()
            header_point_cloud.stamp = current_time
            header_point_cloud.frame_id = "map"

            fields_point_cloud =  [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('snr', 12, PointField.FLOAT32, 1 )]

            point_cloud_2 = pc2.create_cloud(header_point_cloud, fields_point_cloud, cartesianPoints)
            self.publisher_cloud.publish(point_cloud_2)

        if (numTargets>0):
            header_target = Header()
            header_target.stamp = current_time
            header_target.frame_id = "map"
            for n in range(numTargets):
                target_point = Point(targets[1,n], targets[2,n], targets[3,n])
                target_pos = PointStamped(header_target, target_point)
                self.publisher_target.publish(target_pos)






if __name__ == "__main__":

    rospy.init_node('IWR6843ISKReader', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Read parameters
    uart_port = rospy.get_param('~uart_port')
    data_port = rospy.get_param('~data_port')
    config_file_path = rospy.get_param('~config_file_path')
    publish_radar_topic = rospy.get_param('~publish_radar_topic')
    publish_cloud_topic = rospy.get_param('~publish_cloud_topic')
    publish_target_topic = rospy.get_param('~publish_target_topic')
    pub_radar = rospy.Publisher(publish_radar_topic, RadarScan, queue_size=100)
    pub_cloud = rospy.Publisher(publish_cloud_topic, PointCloud2, queue_size=100)
    pub_target = rospy.Publisher(publish_target_topic, PointStamped, queue_size=100)
    reader = IWR6843ISKReader(uart_port, data_port, config_file_path, pub_radar, pub_cloud, pub_target)

    print("=========== GTEC mmWave IWR6843ISK Reader ============")

    connected = reader.connectCom()
    if connected:
        configured = reader.sendConfigToDevice()
        if configured:
            while not rospy.is_shutdown():
                reader.loop()
                rate.sleep()
