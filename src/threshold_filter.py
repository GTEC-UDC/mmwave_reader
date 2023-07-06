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


import rospy
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


class ThresholdFilter(object):

    def __init__(self, publisher_filtered)-> None:
        self.do_filter_snr = False
        self.do_filter_doppler = False
        self.publisher_filtered = publisher_filtered

    def filter_snr(self, min_snr:float, max_snr:float)->None:
        self.do_filter_snr = True
        self.min_snr = min_snr
        self.max_snr = max_snr

    def filter_doppler(self, min_doppler:float, max_doppler:float)->None:
        self.do_filter_doppler= True
        self.min_doppler = min_doppler
        self.max_doppler = max_doppler        

    def filter_measurement(self, cloud_points_msg: PointCloud2)-> None:

        filtered_points = []

        for point in pc2.read_points(cloud_points_msg, field_names=("x", "y", "z", "snr", "doppler"), skip_nans=True):
            point_is_valid = True

            if (self.do_filter_snr==True and point_is_valid==True):
                if (point[3]<self.min_snr or point[3]>self.max_snr):
                    point_is_valid = False
                    print(f'SNR filtered: {point[3]}')

            if (self.do_filter_doppler==True and point_is_valid==True):
                if (abs(point[4])<self.min_doppler or abs(point[4])>self.max_doppler):
                    point_is_valid = False
                    print(f'Doppler filtered: {point[4]}')

            if (point_is_valid==True):
                filtered_points.append(point)

        if (len(filtered_points)>0):
            filtered_cloud_points_msg = pc2.create_cloud(cloud_points_msg.header, cloud_points_msg.fields, filtered_points)
            self.publisher_filtered.publish(filtered_cloud_points_msg)


if __name__ == "__main__":

    rospy.init_node('ThresholdFilter', anonymous=True)
    rate = rospy.Rate(10)  # 10hz


    raw_measurements_topic = rospy.get_param('~raw_measurements_topic')
    publish_topic = rospy.get_param('~publish_filtered_topic')

    pub_filtered = rospy.Publisher(publish_topic, PointCloud2, queue_size=100)

    do_filter_snr = rospy.get_param('~do_filter_snr', False)
    do_filter_doppler = rospy.get_param('~do_filter_doppler', False)

    min_snr = float(rospy.get_param('~min_snr',0.0))
    max_snr = float(rospy.get_param('~max_snr',0.0))
    min_doppler = float(rospy.get_param('~min_doppler',0.0))
    max_doppler = float(rospy.get_param('~max_doppler',0.0))
    
    threshold_filter = ThresholdFilter(pub_filtered)

    if (do_filter_snr==True):
        threshold_filter.filter_snr(min_snr, max_snr)

    if (do_filter_doppler==True):
        threshold_filter.filter_doppler(min_doppler, max_doppler)

    rospy.Subscriber(raw_measurements_topic, PointCloud2, threshold_filter.filter_measurement)

    print("=========== GTEC mmWave Threshold Filter ============")

    rospy.spin()
