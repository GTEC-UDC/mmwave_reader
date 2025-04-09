"""
IWR6843ISK Out of Box Reader Node

This module provides a ROS2 node for reading and processing data from the IWR6843ISK radar sensor
in out-of-box demo mode. It handles the communication with the radar, data parsing, and publishing
of various ROS2 messages including point clouds, radar cubes, and range-azimuth/doppler data.

The node provides basic radar functionality with configurable parameters and real-time data processing.

Author: GTEC
Date: 2024
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from gtec_msgs.msg import RadarCube, RadarRangeAzimuth, RadarRangeDoppler

from gtec_mmwave_reader.IWR6843ISK.tlv_parser import TLVParser 
from gtec_mmwave_reader.IWR6843ISK.tlv_uart_reader import TLVUartReader, LabId, RadarPose, BoundaryBox


class IWR6843ISKOutOfBoxReader(Node):
    """
    ROS2 node for reading and processing IWR6843ISK radar data in out-of-box demo mode.
    
    This node handles:
    - Communication with the radar sensor
    - Data parsing and processing
    - Publishing of various ROS2 messages:
        - PointCloud2: Point cloud data with x,y,z coordinates and SNR
        - RadarCube: Raw radar cube data
        - RadarRangeAzimuth: Range-azimuth heatmap
        - RadarRangeDoppler: Range-doppler heatmap
    
    Attributes:
        uart_port (str): Path to the UART port for configuration
        data_port (str): Path to the UART port for data
        config_file_path (str): Path to the radar configuration file
        num_antennas (int): Number of azimuth antennas
        range_fft_size (int): Size of range FFT
        num_doppler_bins (int): Number of doppler bins
        uart_reader (TLVUartReader): UART reader instance
        publishers (dict): Dictionary of ROS2 publishers
    """

    def __init__(self):
        """
        Initialize the out-of-box reader node.
        
        Sets up:
        - ROS2 parameters
        - UART communication
        - Publishers
        - Configuration
        - Main processing loop
        """
        super().__init__('iwr6843isk_reader_out_of_box')

        # Declarar parámetros
        self.declare_parameter('uart_port', '/dev/ttyUSB0')
        self.declare_parameter('data_port', '/dev/ttyUSB1')
        self.declare_parameter('config_file_path', 'config.cfg')
        self.declare_parameter('publish_cloud_topic', 'cloud')
        self.declare_parameter('publish_radar_cube_topic', 'radar_cube')
        self.declare_parameter('publish_radar_range_azimuth_topic', 'radar_range_azimuth')
        self.declare_parameter('publish_radar_range_doppler_topic', 'radar_range_doppler')

        # Obtener parámetros
        uart_port = self.get_parameter('uart_port').get_parameter_value().string_value
        data_port = self.get_parameter('data_port').get_parameter_value().string_value
        config_file_path = self.get_parameter('config_file_path').get_parameter_value().string_value

        cloud_topic = self.get_parameter('publish_cloud_topic').get_parameter_value().string_value
        cube_topic = self.get_parameter('publish_radar_cube_topic').get_parameter_value().string_value
        azimuth_topic = self.get_parameter('publish_radar_range_azimuth_topic').get_parameter_value().string_value
        doppler_topic = self.get_parameter('publish_radar_range_doppler_topic').get_parameter_value().string_value

        # Guardar configuración
        self.uart_port = uart_port
        self.data_port = data_port
        self.config_file_path = config_file_path
        self.num_antennas = 8
        self.range_fft_size = 256
        self.num_doppler_bins = 16
        self.uart_reader = TLVUartReader(lab_id=LabId.OutOfBoxDemo)

        # Publishers
        self.publisher_cloud = self.create_publisher(PointCloud2, cloud_topic, 10)
        self.publisher_radar_cube = self.create_publisher(RadarCube, cube_topic, 10)
        self.publisher_radar_range_azimuth = self.create_publisher(RadarRangeAzimuth, azimuth_topic, 10)
        self.publisher_radar_range_doppler = self.create_publisher(RadarRangeDoppler, doppler_topic, 10)

        # Setup y timer
        if self.connectCom():
            if self.sendConfigToDevice():
                self.get_logger().info("Radar conectado y configurado")
                self.create_timer(0.1, self.loop)
            else:
                self.get_logger().error("No se pudo enviar configuración al radar")
        else:
            self.get_logger().error("No se pudo conectar al radar")

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
            self.uart_reader.sendCfg(cfg)
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending config to device: {e}")
            return False

    def loop(self):
        """
        Main processing loop.
        
        This method:
        1. Reads and parses data from the radar
        2. Processes the data into appropriate formats
        3. Publishes the processed data as ROS2 messages:
           - PointCloud2 for point cloud data
           - RadarCube for raw radar cube
           - RadarRangeAzimuth for range-azimuth heatmap
           - RadarRangeDoppler for range-doppler heatmap
        """
        hadFail, frameBytes, numTLVs, tlvHeaderLength, numDetectedObj = self.uart_reader.readAndParseUart()

        if not hadFail:
            parser = TLVParser(
                labType=self.uart_reader.labId,
                num_azimuth_antennas=self.num_antennas,
                num_range_bins=self.range_fft_size,
                num_doppler_bins=self.num_doppler_bins
            )
            data = parser.parseMsg(frameBytes, numTLVs, tlvHeaderLength, numDetectedObj)

            pointCloud = data[1]
            numPoints = data[4]
            frameNum = data[6]
            fail = data[7]
            radarCube = data[9]
            rangeAzimuth = data[10]
            rangeDoppler = data[11]

            if not fail:
                current_time = self.get_clock().now().to_msg()

                # PointCloud2
                if numPoints > 0:
                    cartesianPoints = np.zeros((numPoints, 5))
                    for i in range(numPoints):
                        cartesianPoints[i] = [
                            pointCloud[0, i], pointCloud[1, i], pointCloud[2, i],
                            pointCloud[4, i], pointCloud[3, i]  # snr, doppler
                        ]

                    header = Header(stamp=current_time, frame_id="radar")
                    fields = [
                        PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1),
                        PointField('snr', 12, PointField.FLOAT32, 1),
                        PointField('doppler', 16, PointField.FLOAT32, 1)
                    ]
                    cloud_msg = pc2.create_cloud(header, fields, cartesianPoints)
                    self.publisher_cloud.publish(cloud_msg)

                # RadarRangeAzimuth
                az_msg = RadarRangeAzimuth()
                az_msg.header.stamp = current_time
                az_msg.header.frame_id = "radar"
                az_msg.header.seq = frameNum
                az_msg.numRangeBins = self.range_fft_size
                az_msg.numVirtualAntennas = self.num_antennas
                az_msg.data = [val for row in rangeAzimuth for val in row]
                self.publisher_radar_range_azimuth.publish(az_msg)

                # RadarCube
                cube_msg = RadarCube()
                cube_msg.header.stamp = current_time
                cube_msg.header.frame_id = "radar"
                cube_msg.header.seq = frameNum
                cube_msg.fftSize = self.range_fft_size
                cube_msg.angleBins = 64
                cube_msg.data = [val for row in radarCube for val in row]
                self.publisher_radar_cube.publish(cube_msg)

                # RadarRangeDoppler
                doppler_msg = RadarRangeDoppler()
                doppler_msg.header.stamp = current_time
                doppler_msg.header.frame_id = "radar"
                doppler_msg.header.seq = frameNum
                doppler_msg.numRangeBins = self.range_fft_size
                doppler_msg.numDopplerBins = self.num_doppler_bins
                doppler_msg.data = rangeDoppler
                self.publisher_radar_range_doppler.publish(doppler_msg)


def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    
    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)
    node = IWR6843ISKOutOfBoxReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
