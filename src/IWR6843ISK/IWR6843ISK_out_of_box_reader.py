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
from std_msgs.msg import Header
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from gtec_msgs.msg import RadarCube, RadarRangeAzimuth, RadarRangeDoppler

from tlv_parser import TLVParser 
from tlv_uart_reader import TLVUartReader, LabId, RadarPose, BoundaryBox

class IWR6843ISKOutOfBoxReader(object):

    def __init__(self, uart_port, data_port, config_file_path, publisher_cloud, publisher_radar_cube, publisher_radar_range_azimuth, publisher_radar_range_doppler):
        self.uart_port = uart_port
        self.data_port = data_port
        self.uart_reader = TLVUartReader(lab_id=LabId.OutOfBoxDemo)
        self.publisher_radar_cube = publisher_radar_cube
        self.publisher_cloud = publisher_cloud
        self.publisher_radar_range_azimuth = publisher_radar_range_azimuth
        self.publisher_radar_range_doppler = publisher_radar_range_doppler
        self.config_file_path = config_file_path
        self.num_antennas = 8
        self.range_fft_size = 256
        self.num_doppler_bins = 16

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
            self.uart_reader.sendCfg(cfg)
            return True
        except Exception as e:
            print(e)
            return False

    def loop(self):
        hadFail, frameBytes, numTLVs, tlvHeaderLength, numDetectedObj= self.uart_reader.readAndParseUart()

        if not hadFail:
            newParser = TLVParser(labType=self.uart_reader.labId, num_azimuth_antennas=8, num_range_bins=256, num_doppler_bins=16)
            data = newParser.parseMsg(frameBytes, numTLVs, tlvHeaderLength, numDetectedObj)

            pointCloud = data[1]
            numPoints = data[4]
            
            frameNum = data[6]
            fail = data[7]
            radarCube = data[9]
            rangeAzimuth = data[10]
            rangeDoppler = data[11]

            if not fail:
                cartesianPoints = np.zeros((numPoints,5))
                for i in range(numPoints):
                    x = pointCloud[0,i]
                    y = pointCloud[1,i]
                    z = pointCloud[2,i]
                    doppler = pointCloud[3,i]
                    snr = pointCloud[4,i]
                    cartesianPoints[i] = [x,y, z, snr, doppler]

                # Point Cloud
                current_time = rospy.Time.now()
                if numPoints>0:
                    header_point_cloud = Header()
                    header_point_cloud.stamp = current_time
                    header_point_cloud.frame_id = "radar"

                    fields_point_cloud =  [
                        PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1),
                        PointField('snr', 12, PointField.FLOAT32, 1),
                        PointField('doppler', 16, PointField.FLOAT32, 1 )]

                    point_cloud_2 = pc2.create_cloud(header_point_cloud, fields_point_cloud, cartesianPoints)
                    self.publisher_cloud.publish(point_cloud_2)


                #Radar range azimuth
                radar_range_msg = RadarRangeAzimuth()
                radar_range_msg.header = Header()
                radar_range_msg.header.stamp = current_time
                radar_range_msg.header.frame_id = "radar"
                radar_range_msg.header.seq = frameNum
                radar_range_msg.numRangeBins = self.range_fft_size
                radar_range_msg.numVirtualAntennas = self.num_antennas
                radar_range_msg.data = []
                for row in rangeAzimuth:
                    for col in row:
                        radar_range_msg.data.append(col)
                self.publisher_radar_range_azimuth.publish(radar_range_msg)

                # Radar Cube
                radar_cube_msg = RadarCube()
                radar_cube_msg.header = Header()
                radar_cube_msg.header.stamp = current_time
                radar_cube_msg.header.frame_id = "radar"
                radar_cube_msg.header.seq = frameNum

                radar_cube_msg.data = []
                for row in radarCube:
                    for col in row:
                        radar_cube_msg.data.append(col)
                
                radar_cube_msg.fftSize = self.range_fft_size
                radar_cube_msg.angleBins = 64

                self.publisher_radar_cube.publish(radar_cube_msg)

                #Radar range doppler
                radar_range_doppler_msg = RadarRangeDoppler()
                radar_range_doppler_msg.header = Header()
                radar_range_doppler_msg.header.stamp = current_time
                radar_range_doppler_msg.header.frame_id = "radar"
                radar_range_doppler_msg.header.seq = frameNum
                radar_range_doppler_msg.numRangeBins = self.range_fft_size
                radar_range_doppler_msg.numDopplerBins = self.num_doppler_bins
                radar_range_doppler_msg.data = rangeDoppler
                # for val in rangeDoppler:
                #         radar_range_doppler_msg.data.append(col)
                self.publisher_radar_range_doppler.publish(radar_range_doppler_msg)


if __name__ == "__main__":

    rospy.init_node('IWR6843ISKReaderOutOfBox', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Read parameters
    uart_port = rospy.get_param('~uart_port')
    data_port = rospy.get_param('~data_port')
    config_file_path = rospy.get_param('~config_file_path')
    publish_cloud_topic = rospy.get_param('~publish_cloud_topic')
    publish_radar_cube_topic = rospy.get_param('~publish_radar_cube_topic')
    publish_radar_range_azimuth_topic = rospy.get_param('~publish_radar_range_azimuth_topic')
    publish_radar_range_doppler_topic = rospy.get_param('~publish_radar_range_doppler_topic')

    pub_radar_cube = rospy.Publisher(publish_radar_cube_topic, RadarCube, queue_size=100)
    pub_cloud = rospy.Publisher(publish_cloud_topic, PointCloud2, queue_size=100)
    pub_radar_range_azimuth = rospy.Publisher(publish_radar_range_azimuth_topic, RadarRangeAzimuth, queue_size=100)
    pub_radar_range_doppler = rospy.Publisher(publish_radar_range_doppler_topic, RadarRangeDoppler, queue_size=100)

    reader = IWR6843ISKOutOfBoxReader(uart_port, data_port, config_file_path, pub_cloud, pub_radar_cube, pub_radar_range_azimuth, pub_radar_range_doppler)

    print("=========== GTEC mmWave IWR6843ISK Out of Box Reader ============")

    connected = reader.connectCom()
    if connected:
        configured = reader.sendConfigToDevice()
        if configured:
            while not rospy.is_shutdown():
                reader.loop()
                rate.sleep()
