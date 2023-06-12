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
import math
from std_msgs.msg import Header

from radar_msgs.msg import RadarScan, RadarReturn
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, Point

from tlv_parser import TLVParser 
from tlv_uart_reader import TLVUartReader, LabId


class RadarPose(object):
    def __init__(self, sensor_height, sensor_x, sensor_y, elev_tilt, radar_yaw):
        self.sensor_height = sensor_height
        self.sensor_x = sensor_x
        self.sensor_y = sensor_y
        self.elev_tilt = elev_tilt
        self.radar_yaw = radar_yaw

class BoundaryBox(object):
    def __init__(self, min_x, min_y, min_z, max_x, max_y, max_z):
        self.min_x = min_x
        self.min_y = min_y
        self.min_z = min_z
        self.max_x = max_x
        self.max_y = max_y
        self.max_z = max_z

class IWR6843ISKPeopleCountingReader(object):

    def __init__(self, uart_port, data_port, config_file_path, publisher_radar, publishers_target, publisher_cloud, radar_pose, boundary_box, radar_id, publish_all_target_topic):
        self.uart_port = uart_port
        self.data_port = data_port
        self.uart_reader = TLVUartReader(lab_id=LabId.PeopleCounting3D)
        self.publisher_radar = publisher_radar
        self.publishers_target = publishers_target
        self.publisher_cloud = publisher_cloud
        self.config_file_path = config_file_path
        self.radar_id = radar_id
        self.publish_all_target_topic = publish_all_target_topic

        self.radar_pose = radar_pose
        self.boundary_box = boundary_box

    def connectCom(self):
        try:
            uart = self.uart_port
            data = self.data_port
            self.uart_reader.connectComPorts(uart, data)
            return True
        except Exception as e:
            print(e)
            return False

    def sendConfigToDevice(self):
        try:
            cfg_file = open(self.config_file_path, 'r')
            cfg = cfg_file.readlines()
            self.uart_reader.sendCfgOverwriteSensorPosition(cfg, self.radar_pose, self.boundary_box)
            #self.uart_reader.sendCfg(cfg)
            return True
        except Exception as e:
            print(e)
            return False

    def loop(self):
        hadFail, frameBytes, numTLVs, tlvHeaderLength, numDetectedObj= self.uart_reader.readAndParseUart()
        if not hadFail:
            newParser = TLVParser(labType=self.uart_reader.labId, num_azimuth_antennas=8, num_range_bins=256, num_doppler_bins=16)
            data = newParser.parseMsg(frameBytes, numTLVs, tlvHeaderLength, numDetectedObj)

            polarPoints = data[0]
            cartesianPoints = data[1]
            targets = data[2]
            indexes = data[3]
            numPoints = data[4]
            numTargets = data[5]
            fail = data[7]

            if not fail:
                returns = []
                ca_points = []

                for i in range(numPoints):
                    ranging = polarPoints[0][i]
                    azim = polarPoints[1][i]
                    elevation = polarPoints[2][i]
                    doppler = polarPoints[3][i]
                    snr = polarPoints[4][i]

                    radar_return = RadarReturn(range=ranging, azimuth=azim, elevation=elevation, doppler_velocity=doppler, amplitude=snr)
                    returns.append(radar_return)

                    ca_point = [cartesianPoints[0][i],cartesianPoints[1][i],cartesianPoints[2][i], cartesianPoints[3][i], cartesianPoints[4][i]]
                    ca_points.append(ca_point)

                current_time = rospy.Time.now()

                if numPoints>0:
                    
                    header_radar_scan = Header()
                    header_radar_scan.stamp = current_time
                    header_radar_scan.frame_id = self.radar_id
                    radar_scan = RadarScan(header_radar_scan, returns)
                    self.publisher_radar.publish(radar_scan)

                    #TI Cartesian points
                    header_point_cloud = Header()
                    header_point_cloud.stamp = rospy.Time.now()
                    header_point_cloud.frame_id = self.radar_id
                    fields_point_cloud =  [
                        PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1),
                        PointField('snr', 12, PointField.FLOAT32, 1 ),
                        PointField('doppler', 16, PointField.FLOAT32, 1 )]

                    point_cloud_ti = pc2.create_cloud(header_point_cloud, fields_point_cloud, ca_points)
                    self.publisher_cloud.publish(point_cloud_ti)

                if (numTargets>0):
                    header_target = Header()
                    header_target.stamp = current_time
                    header_target.frame_id = "target_" + self.radar_id
                    
                    for n in range(numTargets):
                        if (n<8):
                            target_point = Point(targets[1,n], targets[2,n], targets[3,n])
                            # Axis change, to be used in ROS
                            target_point_right = Point(math.cos(self.radar_pose.radar_yaw)*target_point.y, -1*math.cos(self.radar_pose.radar_yaw)*target_point.x, target_point.z)
                            target_pos = PointStamped(header_target, target_point_right)
                            self.publishers_target[n].publish(target_pos)
                            self.publish_all_target_topic.publish(target_pos)

if __name__ == "__main__":

    rospy.init_node('IWR6843ISKReader', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Read parameters
    uart_port = rospy.get_param('~uart_port')
    data_port = rospy.get_param('~data_port')
    config_file_path = rospy.get_param('~config_file_path')
    publish_radar_topic = rospy.get_param('~publish_radar_topic')
    publish_target_topic = rospy.get_param('~publish_target_topic')
    sensor_height = float(rospy.get_param('~sensor_height'))
    sensor_x = float(rospy.get_param('~sensor_x'))
    sensor_y = float(rospy.get_param('~sensor_y'))
    elev_tilt = float(rospy.get_param('~elev_tilt'))
    radar_yaw = float(rospy.get_param('~radar_yaw'))
    radar_id = rospy.get_param('~radar_id')
    publish_all_target_topic = rospy.get_param('~publish_all_target_topic')

    boundary_box_x_min = float(rospy.get_param('~boundary_box_x_min'))
    boundary_box_y_min = float(rospy.get_param('~boundary_box_y_min'))
    boundary_box_z_min = float(rospy.get_param('~boundary_box_z_min'))
    boundary_box_x_max = float(rospy.get_param('~boundary_box_x_max'))
    boundary_box_y_max = float(rospy.get_param('~boundary_box_y_max'))
    boundary_box_z_max = float(rospy.get_param('~boundary_box_z_max'))

    boundary_box = BoundaryBox(boundary_box_x_min, boundary_box_y_min, boundary_box_z_min, boundary_box_x_max, boundary_box_y_max, boundary_box_z_max)


    #elev_tilt is in radians, the ti tracker needs it in degrees
    elev_tilt = abs(elev_tilt) * 180.0/math.pi


    radar_pose = RadarPose(sensor_height, sensor_x, sensor_y, elev_tilt, radar_yaw)


    pub_radar = rospy.Publisher(publish_radar_topic, RadarScan, queue_size=100)
    pub_cloud = rospy.Publisher('/gtec/mmwave/'+radar_id+'/ti_cloud', PointCloud2, queue_size=100)

    pubs_target = []
    for n in range(8):
        pubs_target.append(rospy.Publisher(str(publish_target_topic)+'/'+str(n), PointStamped, queue_size=100))

    pub_all_targets = rospy.Publisher(str(publish_all_target_topic), PointStamped, queue_size=100)
    reader = IWR6843ISKPeopleCountingReader(uart_port, data_port, config_file_path, pub_radar, pubs_target, pub_cloud, radar_pose, boundary_box, radar_id, pub_all_targets)

    print("=========== GTEC mmWave IWR6843ISK People Counting Reader ============")

    connected = reader.connectCom()
    if connected:
        configured = reader.sendConfigToDevice()
        if configured:
            while not rospy.is_shutdown():
                reader.loop()
                rate.sleep()
