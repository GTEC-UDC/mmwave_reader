"""
TLV UART Reader for IWR6843ISK Radar Sensor

This module provides functionality to read and parse data from the IWR6843ISK radar sensor
over UART communication. It handles the low-level communication with the radar device
and provides methods to configure and read data from it.

The module supports two main modes of operation:
1. OutOfBoxDemo: For basic radar functionality
2. PeopleCounting3D: For advanced 3D people counting applications

Author: GTEC
Date: 2024
"""

import struct
import sys
import serial
import binascii
import time
import numpy as np
import math
import gtec_mmwave_reader.IWR6843ISK.fft
from operator import add
from enum import Enum

class RadarPose(object):
    """
    Represents the physical position and orientation of the radar sensor.
    
    Attributes:
        sensor_height (float): Height of the sensor from the ground
        sensor_x (float): X position of the sensor
        sensor_y (float): Y position of the sensor
        elev_tilt (float): Elevation tilt angle of the sensor
    """
    def __init__(self, sensor_height, sensor_x, sensor_y, elev_tilt):
        self.sensor_height = sensor_height
        self.sensor_x = sensor_x
        self.sensor_y = sensor_y
        self.elev_tilt = elev_tilt

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

class LabId(Enum):
    """
    Enumeration of supported radar operation modes.
    
    Values:
        OutOfBoxDemo: Basic radar functionality
        PeopleCounting3D: Advanced 3D people counting
    """
    OutOfBoxDemo = 0
    PeopleCounting3D = 1

class TLVUartReader():
    """
    UART reader for TLV (Type-Length-Value) messages from IWR6843ISK radar sensor.
    
    This class handles the low-level communication with the radar device over UART,
    including configuration and data reading. It supports two main operation modes:
    OutOfBoxDemo and PeopleCounting3D.
    
    Attributes:
        headerLength (int): Length of the message header
        magicWord (int): Magic word for message validation
        labId (LabId): Current operation mode
        uartCom (serial.Serial): UART communication port for configuration
        dataCom (serial.Serial): UART communication port for data
    """
    
    def __init__(self, lab_id:LabId):
        """
        Initialize the UART reader with the specified operation mode.
        
        Args:
            lab_id (LabId): Operation mode (OutOfBoxDemo or PeopleCounting3D)
        """
        self.headerLength = 52
        self.magicWord = 0x708050603040102  # o 0x0807060504030201 dependiendo del endianness
        self.labId = lab_id

    def connectComPorts(self, uartCom, dataCom):
        """
        Connect to the radar's UART ports.
        
        Args:
            uartCom (str): Path to the UART port for configuration
            dataCom (str): Path to the UART port for data
            
        The configuration port runs at 115200 baud and the data port at 921600 baud.
        """
        self.uartCom = serial.Serial(uartCom, 115200,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.3)
        self.dataCom = serial.Serial(dataCom, 921600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.025)
        self.dataCom.reset_output_buffer()
        print('Connected')

    def sendCfgOverwriteSensorPosition(self, cfg, radar_pose, boundary_box):
        """
        Send configuration to the radar with custom sensor position and boundary box.
        
        Args:
            cfg (list): List of configuration commands
            radar_pose (RadarPose): Sensor position and orientation
            boundary_box (BoundaryBox): Detection boundary box
            
        This method sends the configuration while overwriting the sensor position
        and boundary box settings with the provided values.
        """
        for line in cfg:
            if (line.startswith('sensorPosition')):
                self.sendLine('sensorPosition %f 0 %f\n'%(radar_pose.sensor_height, radar_pose.elev_tilt))
            elif (line.startswith('boundaryBox')):
                self.sendLine('boundaryBox %f %f %f %f %f %f\n'%(boundary_box.min_x, boundary_box.max_x, boundary_box.min_y, boundary_box.max_y, boundary_box.min_z , boundary_box.max_z))
            elif (line.startswith('presenceBoundaryBox')):
                self.sendLine('presenceBoundaryBox %f %f %f %f %f %f\n'%(boundary_box.min_x, boundary_box.max_x, boundary_box.min_y, boundary_box.max_y, boundary_box.min_z , boundary_box.max_z))
            else:
                self.sendLine(line)
        time.sleep(3)
        self.uartCom.reset_input_buffer()
        self.uartCom.close()

    def sendCfg(self, cfg):
        """
        Send configuration to the radar.
        
        Args:
            cfg (list): List of configuration commands
            
        Sends each configuration command to the radar and waits for acknowledgment.
        """
        for line in cfg:
            self.sendLine(line)
        time.sleep(3)
        self.uartCom.reset_input_buffer()
        self.uartCom.close()

    def sendLine(self, line):
        """
        Send a single command line to the radar.
        
        Args:
            line (str): Command line to send
            
        Sends the command and waits for acknowledgment. Skips lines starting with '%'.
        """
        if (not line[0]=='%'):
            time.sleep(.1)
            print("---------------------------")
            print(line)
            self.uartCom.write(line.encode())
            ack = self.uartCom.readline()
            ack = self.uartCom.readline()
            print(str(ack))
            print("==========================")

    def readAndParseUart(self):
        """
        Read and parse data from the radar's UART port.
        
        Returns:
            tuple: (fail, dataTosend, numTLVs, tlvHeaderLength, numDetectedObj)
                - fail (int): Error flag (0 if successful)
                - dataTosend (bytes): Raw data to be parsed
                - numTLVs (int): Number of TLVs in the message
                - tlvHeaderLength (int): Length of TLV header
                - numDetectedObj (int): Number of detected objects
        """
        numDetectedObj = 0
        self.fail = 0
        self.byteData = bytes(1)
        numBytes = 0
        if (self.labId == LabId.OutOfBoxDemo):
            numBytes = 8192
        elif (self.labId == LabId.PeopleCounting3D):
            numBytes = 4666

        data = self.dataCom.read(numBytes)
        if (self.byteData is None):
            self.byteData = data
        else:
            self.byteData += data
        dataTosend = None
        if (self.labId == LabId.OutOfBoxDemo):
            dataTosend, numTLVs, tlvHeaderLength, numDetectedObj= self.parseOutOfBoxTLVHeader(self.byteData)
        elif (self.labId == LabId.PeopleCounting3D):
            dataTosend, numTLVs, tlvHeaderLength = self.parsePeople3DHeader(self.byteData)

        return self.fail, dataTosend, numTLVs, tlvHeaderLength, numDetectedObj

    def parseOutOfBoxTLVHeader(self, dataIn):
        """
        Parse the header of an OutOfBoxDemo message.
        
        Args:
            dataIn (bytes): Raw message data
            
        Returns:
            tuple: (dataIn, numTLVs, tlvHeaderLength, numDetectedObj)
                - dataIn (bytes): Remaining data after header
                - numTLVs (int): Number of TLVs
                - tlvHeaderLength (int): Length of TLV header
                - numDetectedObj (int): Number of detected objects
        """
        headerStruct = 'Q8I'
        headerLength = struct.calcsize(headerStruct)
        tlvHeaderLength = 8
        
        # Imprimir primeros bytes para depuración
        print("First 20 bytes of data (hex):", ' '.join(f"{byte:02x}" for byte in dataIn[:20]))
        
        #search until we find magic word
        while(1):
            try:
                magic, version, totalPacketLen, platform, self.frameNum, timeCPUCycles, numDetectedObj, numTLVs, subFrameNum = struct.unpack(headerStruct, dataIn[:headerLength])
                print(f"Magic: {hex(magic)}, expected: {hex(self.magicWord)}")
                print(f"Version: {version}, TotalPacketLen: {totalPacketLen}, numDetectedObj: {numDetectedObj}, numTLVs: {numTLVs}")
                
            except Exception as e:
                #bad data, return
                print(f"Error unpacking header: {e}")
                self.fail = 1
                return dataIn,0,0,0
            if (magic != self.magicWord):
                #wrong magic word, increment pointer by 1 and try again
                print("Wrong magic word, incrementing pointer")
                dataIn = dataIn[1:]
                if len(dataIn) < headerLength:
                    print("Data too short after incrementing")
                    self.fail = 1
                    return dataIn,0,0,0
            else:
                #we have correct magic word, proceed to parse rest of data
                break
        #print('Total Packet length: %d numTLVs: %d'%(totalPacketLen, numTLVs))
        #print('HeaderLength: %d'%(headerLength))
        dataIn = dataIn[headerLength:]
        remainingData = totalPacketLen - len(dataIn)
        #print('Total Packet length (without header): %d'%(remainingData))
        count = 0
        while (remainingData > 0):
            newData = self.dataCom.read(remainingData)
            remainingData = totalPacketLen - len(dataIn) - len(newData)
            dataIn += newData
            count += 1

        #print('Size of result after read all: %d'%(len(result)))
        return dataIn, numTLVs, tlvHeaderLength, numDetectedObj

    def parsePeople3DHeader(self, dataIn):
        """
        Parse the header of a PeopleCounting3D message.
        
        Args:
            dataIn (bytes): Raw message data
            
        Returns:
            tuple: (dataIn, numTLVs, tlvHeaderLength)
                - dataIn (bytes): Remaining data after header
                - numTLVs (int): Number of TLVs
                - tlvHeaderLength (int): Length of TLV header
        """
        #reset point buffers
        self.numDetectedTarget = 0
        self.numDetectedObj = 0
        self.indexes = []
        tlvHeaderLength = 8
        headerLength = 48
        #stay in this loop until we find the magic word or run out of data to parse
        while (1):
            try:
                magic, version, packetLength, platform, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum =  struct.unpack('Q9I2H', dataIn[:headerLength])
            except Exception as e:
                #bad data, return
                #print("Cannot Read Frame Header")
                #print(e)
                self.fail = 1
                return dataIn,0,0
            if (magic != self.magicWord):
                #wrong magic word, increment pointer by 1 and try again
                dataIn = dataIn[1:]
            else:
                #got magic word, proceed to parse
                break
        
        
        #print('HeaderLength: %d'%(headerLength))
        dataIn = dataIn[headerLength:]
        remainingData = packetLength - len(dataIn) - headerLength
        count = 0
        while (remainingData > 0):
            newData = self.dataCom.read(remainingData)
            remainingData = packetLength - len(dataIn) - len(newData) - headerLength
            dataIn += newData
            count += 1
        #print('Total Packet length: %d numTLVs: %d'%(packetLength, numTLVs))
        return dataIn, numTLVs, tlvHeaderLength