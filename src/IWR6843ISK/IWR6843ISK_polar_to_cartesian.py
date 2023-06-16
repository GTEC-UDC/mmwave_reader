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

from radar_msgs.msg import RadarScan, RadarReturn
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, Point
import tf2_ros
import tf2_geometry_msgs
import rotation_helper as rot_helper

class IWR6843ISKPolarToCartesian(object):

    def __init__(self, publisher_cloud, publisher_cloud_all, radar_id, radar_pitch, radar_yaw, radar_roll, radar_tf_radar_to_odom, radar_tf_odom_to_radar, radar_z, use_fixed_z, fixed_z):
        self.publisher_cloud = publisher_cloud
        self.publisher_cloud_all = publisher_cloud_all
        self.radar_id = radar_id
        self.radar_tf_radar_to_odom = radar_tf_radar_to_odom
        self.radar_tf_odom_to_radar = radar_tf_odom_to_radar
        self.rot_matrix = rot_helper.euler_to_rotation_matrix(-1*radar_yaw, radar_pitch, radar_roll)
        self.rot_matrix_radar_to_ros = rot_helper.euler_to_rotation_matrix(-1.5708, 0, 0)
        self.use_fixed_z = use_fixed_z
        self.fixed_z = fixed_z
        self.radar_z = radar_z

    def radar_listener(self, radarScan):
        polarPoints = radarScan.returns
        numPoints = len(polarPoints)
        cartesianPoints = np.zeros((numPoints,5))
        cartesianPointsTransformed = np.zeros((numPoints,5))
        for i in range(numPoints):
            (cartesianPoints[i], cartesianPointsTransformed[i])  = self.polar_to_cartesian(polarPoints[i], self.radar_tf_radar_to_odom, self.radar_tf_odom_to_radar)

        
        header_point_cloud = Header()
        header_point_cloud.stamp = rospy.Time.now()
        header_point_cloud.frame_id = radar_id
        fields_point_cloud =  [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('snr', 12, PointField.FLOAT32, 1 ),
            PointField('doppler', 16, PointField.FLOAT32, 1 )]

        point_cloud_2 = pc2.create_cloud(header_point_cloud, fields_point_cloud, cartesianPoints)
        self.publisher_cloud.publish(point_cloud_2)

        header_point_cloud.frame_id = "odom"
        point_cloud_2_tf = pc2.create_cloud(header_point_cloud, fields_point_cloud, cartesianPointsTransformed)
        self.publisher_cloud_all.publish(point_cloud_2_tf)

    def polar_to_cartesian(self, polarPoint, odom_transform, radar_transform):
        
        x = polarPoint.range*math.cos(polarPoint.elevation)*math.sin(polarPoint.azimuth)
        y = polarPoint.range*math.cos(polarPoint.elevation)*math.cos(polarPoint.azimuth)

        if (self.use_fixed_z==True):
            point_msg_in_odom = PointStamped()
            point_msg_in_odom.point = Point(x, y, 0)
            point_in_odom = tf2_geometry_msgs.do_transform_point(point_msg_in_odom, odom_transform)

            point_msg_in_radar = PointStamped()
            point_msg_in_radar.point = Point(point_in_odom.point.x, point_in_odom.point.y, self.fixed_z)
            point_in_radar = tf2_geometry_msgs.do_transform_point(point_msg_in_radar, radar_transform)

            z =  point_in_radar.point.z
        else:
            z = polarPoint.range*math.sin(polarPoint.elevation)

        print(f'Before rot: {x},{y},{z}')
        point_corrected= rot_helper.apply_rotation_matrix(rot_helper.Point(x,y,z), self.rot_matrix_radar_to_ros)
        #print(f'After correct rot: {point_corrected.x},{point_corrected.y},{point_corrected.z}')
        #point_rotated= rot_helper.apply_rotation_matrix(point_corrected, self.rot_matrix)
        print(f'After rot: {point_corrected.x},{point_corrected.y},{point_corrected.z}')
        cartesianPoint = [point_corrected.x, point_corrected.y, point_corrected.z, polarPoint.amplitude, polarPoint.doppler_velocity]

        # elevation_with_tilt = self.elev_tilt - polarPoint.elevation
        # azimut_with_yaw = self.radar_yaw + polarPoint.azimuth
        # x = polarPoint.range*math.cos(elevation_with_tilt)*math.sin(azimut_with_yaw)
        # y = polarPoint.range*math.cos(elevation_with_tilt)*math.cos(azimut_with_yaw)
        # z = polarPoint.range*math.sin(elevation_with_tilt)

        # #We transform from a left-handled axis to a right-handled axis, to work with ROS and RVIZ
        # x_right = y
        # y_right = -x
        # cartesianPoint = [x_right,y_right,z, polarPoint.amplitude, polarPoint.doppler_velocity]

        #We return the point transformed to the Odom space (to fuse differente radars)
        point_msg = PointStamped()
        point_msg.point = Point(point_corrected.x, point_corrected.y, point_corrected.z)

        point_tf = tf2_geometry_msgs.do_transform_point(point_msg, odom_transform)
        cartesianPointTF = [point_tf.point.x, point_tf.point.y, point_tf.point.z, polarPoint.amplitude, polarPoint.doppler_velocity]

        return (cartesianPoint, cartesianPointTF)


if __name__ == "__main__":

    rospy.init_node('IWR6843ISKPolarToCartesian', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Read parameters
    radar_topic = rospy.get_param('~radar_topic')
    publish_cloud_topic = rospy.get_param('~publish_cloud_topic')
    pub_cloud = rospy.Publisher(publish_cloud_topic, PointCloud2, queue_size=100)

    publish_all_cartesian_topic = rospy.get_param('~publish_all_cartesian_topic')
    pub_cloud_all = rospy.Publisher(publish_all_cartesian_topic, PointCloud2, queue_size=100)

    radar_id = rospy.get_param('~radar_id')
    radar_pitch = float(rospy.get_param('~radar_pitch'))
    radar_yaw = float(rospy.get_param('~radar_yaw'))
    radar_roll = float(rospy.get_param('~radar_roll'))

    use_fixed_z = rospy.get_param('~use_fixed_z')

    fixed_z = 0
    if (use_fixed_z==True):
        fixed_z = float(rospy.get_param('~fixed_z'))

    radar_z = float(rospy.get_param('~radar_z'))
    


    tf_buffer = tf2_ros.Buffer(rospy.Duration(2.0)) #tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform_radar_to_odom = tf_buffer.lookup_transform("odom",
                                radar_id, #source frame
                                rospy.Time(0), #get the tf at first available time
                                rospy.Duration(2.0))

    transform_odom_to_radar = tf_buffer.lookup_transform(radar_id,
                                "odom", #source frame
                                rospy.Time(0), #get the tf at first available time
                                rospy.Duration(2.0))

    polarToCartesian = IWR6843ISKPolarToCartesian(pub_cloud, pub_cloud_all, radar_id, radar_pitch, radar_yaw, radar_roll, transform_radar_to_odom, transform_odom_to_radar, radar_z, use_fixed_z, fixed_z)



    rospy.Subscriber(radar_topic, RadarScan,
                     polarToCartesian.radar_listener)

    print("=========== GTEC mmWave IWR6843ISK Polar to Radar ============")

    rospy.spin()
