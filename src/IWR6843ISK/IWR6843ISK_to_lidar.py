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
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from oob_parser import uartParserSDK

from radar_msgs.msg import RadarScan, RadarReturn
from sensor_msgs.msg import LaserScan
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, Point
from gtec_msgs.msg import RadarCube

import numpy as np


class IWR6843ISKToLidar(object):

    def __init__(self, publisher_lidar):

        self.publisher_lidar = publisher_lidar
        self.NUM_ANGLE_BINS = 64
        self.numRangeBins = 256
        digOutSampleRate = 5209
        freqSlopeConst_actual = 70
        numAntennas = 8

        rangeIdxToMeters = 3e8 * digOutSampleRate * 1e3 / (2 * abs(freqSlopeConst_actual)* 1e12 * self.numRangeBins)
        #self.angles = np.deg2rad(np.arange(-self.NUM_ANGLE_BINS/2, self.NUM_ANGLE_BINS/2))
        self.angles = np.arange(-self.NUM_ANGLE_BINS/2+1, self.NUM_ANGLE_BINS/2-1)*(2/self.NUM_ANGLE_BINS)
        #print(self.angles)
      
        self.theta = np.arcsin(self.angles)
        #print(self.theta)
        self.range = np.arange(0,self.numRangeBins) * rangeIdxToMeters


        # posX = np.multiply(range.reshape(1,np.size(range)),np.sin(theta).reshape(np.size(theta),1)).T
        # posY = np.multiply(range.reshape(1,np.size(range)),np.cos(theta).reshape(np.size(theta),1)).T

        # self.x = posX.ravel()
        # self.y = posY.ravel()


    def radar_cube_listener(self, radar_cube_msg):
        frameNum = radar_cube_msg.header.seq
        fftSize = radar_cube_msg.fftSize
        angleBins = radar_cube_msg.angleBins
        data = radar_cube_msg.data

        mat  = np.zeros((self.numRangeBins, self.NUM_ANGLE_BINS-1))
        
        for rowIndex in range(self.numRangeBins):
            for colIndex in range(self.NUM_ANGLE_BINS-1):
                # print("%d, %d"%(rowIndex,colIndex))
                mat[rowIndex, colIndex] = data[rowIndex*(self.NUM_ANGLE_BINS-1) + colIndex]

        #SUB matrix only with closer elements
        #mat = mat[1:100,:]

        maxSnr = np.amax(mat, axis=0)
        ind=np.where(mat==maxSnr)
        cords = list(zip(ind[0], ind[1]))

        ranges = np.zeros(self.NUM_ANGLE_BINS-1)
        intensities = np.zeros(self.NUM_ANGLE_BINS-1)

        for cord in cords:
            col = cord[1]
            row = cord[0]
            intensities[col] = maxSnr[col]
            ranges[col] = self.range[row]

        header_lidar = Header()
        header_lidar.stamp = rospy.Time.now()
        header_lidar.frame_id = "radar"
        msg_lidar = LaserScan()
        msg_lidar.header = header_lidar
        msg_lidar.angle_min = math.pi/2 - np.max(self.theta)
        msg_lidar.angle_max = math.pi/2 + np.abs(np.min(self.theta)) 
        msg_lidar.angle_increment = (np.abs(msg_lidar.angle_max) + np.abs(msg_lidar.angle_min))/(self.NUM_ANGLE_BINS)
        msg_lidar.time_increment = 0
        msg_lidar.scan_time = 0.25
        msg_lidar.range_min = 0
        msg_lidar.range_max = 12
        msg_lidar.ranges = list(ranges[::-1])
        msg_lidar.intensities = list(intensities[::-1])

        self.publisher_lidar.publish(msg_lidar)

        # polarPoints = radarScan.returns
        # numPoints = len(polarPoints)
        # cartesianPoints = np.zeros((numPoints,5))
        # for i in range(numPoints):
        #     cartesianPoints[i]  = self.polar_to_cartesian(polarPoints[i])
        
        # header_point_cloud = Header()
        # header_point_cloud.stamp = rospy.Time.now()
        # header_point_cloud.frame_id = "radar"
        # fields_point_cloud =  [
        #     PointField('x', 0, PointField.FLOAT32, 1),
        #     PointField('y', 4, PointField.FLOAT32, 1),
        #     PointField('z', 8, PointField.FLOAT32, 1),
        #     PointField('snr', 12, PointField.FLOAT32, 1 ),
        #     PointField('doppler', 16, PointField.FLOAT32, 1 )]

        # point_cloud_2 = pc2.create_cloud(header_point_cloud, fields_point_cloud, cartesianPoints)
        # self.publisher_cloud.publish(point_cloud_2)

if __name__ == "__main__":

    rospy.init_node('IWR6843ISKToLidar', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Read parameters

    radar_cube_topic = rospy.get_param('~radar_cube_topic')
    publish_lidar_topic = rospy.get_param('~publish_lidar_topic')
    pub_lidar = rospy.Publisher(publish_lidar_topic, LaserScan, queue_size=100)

    toLidar = IWR6843ISKToLidar(pub_lidar)

    rospy.Subscriber(radar_cube_topic, RadarCube,
                     toLidar.radar_cube_listener)

    print("=========== GTEC mmWave IWR6843ISK to Lidar ============")

    rospy.spin()