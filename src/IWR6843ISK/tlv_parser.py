import struct
import sys
from regex import D
import serial
import binascii
import time
import numpy as np
import math
import fft
from operator import add
from tlv_uart_reader import LabId

class TLVParser():
    def __init__(self,labType:LabId, num_azimuth_antennas:int, num_range_bins:int, num_doppler_bins:int):
        self.labType = labType
        self.maxPoints = 1150
        self.num_azimuth_antennas = num_azimuth_antennas
        self.num_range_bins = num_range_bins
        self.num_doppler_bins = num_doppler_bins
        self.pcPolar = np.zeros((5,self.maxPoints))
        self.pcBufPing = np.zeros((5,self.maxPoints))
        self.numDetectedObj = 0
        self.targetBufPing = np.ones((10,20))*-1
        self.indexBufPing = np.zeros((1,self.maxPoints))
        self.classifierOutput = []
        self.frameNum = 0
        self.missedFrames = 0
        self.byteData = bytes(1)
        self.oldData = []
        self.indexes = []
        self.numDetectedTarget = 0
        self.fail = 0
        self.unique = []
        self.savedData = []
        self.saveNum = 0
        self.saveNumTxt = 0
        self.replayData = []
        self.startTimeLast = 0
        self.saveReplay = 0
        self.savefHist = 0
        self.saveBinary = 0
        self.saveTextFile = 0
        self.fHistRT = np.empty((100,1), dtype=np.object)
        self.plotDimension = 0
        self.getUnique = 0
        self.CaponEC = 0
        self.radarCube = []
        self.rangeFFT = []
        self.rangeDoppler = []
        if (self.labType==LabId.PeopleCounting3D):
            self.textStructCapon3D = np.zeros(1000*3*self.maxPoints*10).reshape((1000,3,self.maxPoints,10))
            #[frame #][header,pt cloud data,target info]


    def parseMsg(self, dataIn, numTLVs, tlvHeaderLength, numDetectedObj):
        if (self.labType==LabId.OutOfBoxDemo):
            self.parseOutOfBoxMsg(dataIn, numTLVs, tlvHeaderLength, numDetectedObj)
            self.numDetectedObj = numDetectedObj
        elif (self.labType == LabId.PeopleCounting3D):
            self.parsePeopleCounting3DMsg(dataIn, numTLVs, tlvHeaderLength)
        
        return self.pcPolar, self.pcBufPing, self.targetBufPing, self.indexes, self.numDetectedObj, self.numDetectedTarget, self.frameNum, self.fail, self.classifierOutput, self.radarCube, self.rangeFFT, self.rangeDoppler


    def parsePeopleCounting3DMsg(self, dataIn, numTLVs, tlvHeaderLength):
        self.pcBufPing = np.zeros((5,self.maxPoints))
        self.pcPolar = np.zeros((5,self.maxPoints))
        self.targetBufPing = np.zeros((13,20))
        self.numDetectedTarget = 0
        self.numDetectedObj = 0
        self.indexes = []
        for i in range(numTLVs):
            try:
                tlvType, tlvLength = self.tlvHeaderDecode(dataIn[:tlvHeaderLength])
            except Exception as e:
                print(e)
                self.fail = 1
                return

            dataIn = dataIn[tlvHeaderLength:]
            dataLength = tlvLength-tlvHeaderLength
            if (tlvType == 6):
                self.parseCapon3DPolar(dataIn[:dataLength], dataLength)
            elif (tlvType == 7):
                self.parseDetectedTracksSDK3x(dataIn[:dataLength], dataLength)
            elif (tlvType == 8):
                self.parseTargetAssociations(dataIn[:dataLength])
            dataIn = dataIn[dataLength:]
        return dataIn


    def parseOutOfBoxMsg(self, dataIn, numTLVs, tlvHeaderLength, numDetectedObj):
            newDataIn = dataIn
            for i in range(numTLVs):
                try:
                    tlvType, tlvLength = self.tlvHeaderDecode(newDataIn[:tlvHeaderLength])
                    if tlvType>10:
                        raise Exception('TLV index not valid') 
                except Exception as e:
                    print(e)
                    self.fail = 1
                    return

                newDataIn = newDataIn[tlvHeaderLength:]
                if (tlvType == 1):
                    self.parseSDK3xPoints(newDataIn[:tlvLength], numDetectedObj)
                elif (tlvType == 7):
                    self.parseSDK3xSideInfo(newDataIn[:tlvLength], numDetectedObj)
                elif (tlvType == 4):
                    self.parseSDK3xRadarCube(newDataIn[:tlvLength])
                elif (tlvType == 5):
                    self.parseSDK3xDopplerHeatmap(newDataIn[:tlvLength])

                newDataIn = newDataIn[tlvLength:]
            return newDataIn

    def tlvHeaderDecode(self, data):
        tlvType, tlvLength = struct.unpack('2I', data)
        return tlvType, tlvLength

    def parseSDK3xPoints(self, dataIn, numObj):
        pointStruct = '4f'
        pointLength = struct.calcsize(pointStruct)
        try:
            for i in range(numObj):
                self.pcBufPing[0,i], self.pcBufPing[1,i], self.pcBufPing[2,i], self.pcBufPing[3,i] = struct.unpack(pointStruct, dataIn[:pointLength])
                dataIn = dataIn[pointLength:]
            self.pcBufPing = self.pcBufPing[:,:numObj]
        except Exception as e:
            print(e)
            self.fail = 1

    def parseCapon3DPolar(self, data, tlvLength):
        pUnitStruct = '5f'  #elev, azim, doppler, range, snr
        pUnitSize = struct.calcsize(pUnitStruct)
        pUnit = struct.unpack(pUnitStruct, data[:pUnitSize])
        data = data[pUnitSize:]
        objStruct = '2bh2H'
        objSize = struct.calcsize(objStruct)
        self.numDetectedObj = int((tlvLength-pUnitSize)/objSize)
        #print("Cloud Size {0}   ".format(self.numDetectedObj), end='\r')
        for i in range(self.numDetectedObj):
            try:
                elev, az, doppler, ran, snr = struct.unpack(objStruct, data[:objSize])
                data = data[objSize:]
                self.pcPolar[0,i] = ran*pUnit[3]           #range
                if (az >= 128):
                    print ('Az greater than 127')
                    az -= 256
                if (elev >= 128):
                    print ('Elev greater than 127')
                    elev -= 256
                if (doppler >= 32768):
                    print ('Doppler greater than 32768')
                    doppler -= 65536
                self.pcPolar[1,i] = az*pUnit[1]  #azimuth
                self.pcPolar[2,i] = elev*pUnit[0] #elevation
                self.pcPolar[3,i] = doppler*pUnit[2]       #doppler
                self.pcPolar[4,i] = snr*pUnit[4]           #snr
                
            except:
                self.numDetectedObj = i
                print('Point Cloud TLV Parser Failed')
                break
        self.polar2Cart3D()

    def polar2Cart3D(self):
        self.pcBufPing = np.empty((5,self.numDetectedObj))
        for n in range(0, self.numDetectedObj):
            self.pcBufPing[2,n] = self.pcPolar[0,n]*math.sin(self.pcPolar[2,n]) #z
            self.pcBufPing[0,n] = self.pcPolar[0,n]*math.cos(self.pcPolar[2,n])*math.sin(self.pcPolar[1,n]) #x
            self.pcBufPing[1,n] = self.pcPolar[0,n]*math.cos(self.pcPolar[2,n])*math.cos(self.pcPolar[1,n]) #y
        self.pcBufPing[3,:] = self.pcPolar[3,0:self.numDetectedObj] #doppler
        self.pcBufPing[4,:] = self.pcPolar[4,0:self.numDetectedObj] #snr


    def parseDetectedTracksSDK3x(self, data, tlvLength):
        targetStruct = 'I27f'
        targetSize = struct.calcsize(targetStruct)
        self.numDetectedTarget = int(tlvLength/targetSize)
        targets = np.empty((16,self.numDetectedTarget))
        rotTarget = [0,0,0]
        try:
            for i in range(self.numDetectedTarget):
                targetData = struct.unpack(targetStruct,data[:targetSize])
                #tid, x, y
                if (self.CaponEC):
                    targets[0:13,i]=targetData[0:13]
                else:
                    #tid, pos x, pos y
                    targets[0:3,i]=targetData[0:3]
                    # pos z
                    targets[3,i] = targetData[3]
                    #vel x, vel y
                    targets[4:6,i] = targetData[4:6]
                    #vel z
                    targets[6,i] = targetData[6]
                    # acc x, acc y
                    targets[7:9,i] = targetData[7:9]
                    # acc z
                    targets[9,i] = targetData[9]
                    #ec[16]
                    #targets[10:14,i]=targetData[10:14]
                    targets[10:13,i]=targetData[10:13]#Chris 2020-12-18
                    #g
                    #targets[14,i]=targetData[14]
                    targets[14,i]=targetData[26]
                    #confidenceLevel
                    #targets[15,i]=targetData[15]
                    targets[15,i]=targetData[27]
                                                         
                data = data[targetSize:]
        except:
            print('Target TLV parse failed')
        self.targetBufPing = targets

    def parseTargetAssociations(self, data):
        targetStruct = 'B'
        targetSize = struct.calcsize(targetStruct)
        numIndexes = int(len(data)/targetSize)
        self.indexes = []
        self.unique = []
        try:
            for i in range(numIndexes):
                ind = struct.unpack(targetStruct, data[:targetSize])
                self.indexes.append(ind[0])
                data = data[targetSize:]
            if (self.getUnique):
                uTemp = self.indexes[math.ceil(numIndexes/2):]
                self.indexes = self.indexes[:math.ceil(numIndexes/2)]
                for i in range(math.ceil(numIndexes/8)):
                    for j in range(8):
                        self.unique.append(self.getBit(uTemp[i], j))
        except:
            print('TLV Index Parse Fail')

    def getBit(byte, bitNum):
        mask = 1 << bitNum
        if (byte&mask):
            return 1
        else:
            return 0

    def parseSDK3xSideInfo(self, dataIn, numObj):
        sideInfoStruct = '2h'
        sideInfoLength = struct.calcsize(sideInfoStruct)
        try:
            for i in range(numObj):
                self.pcBufPing[4,i], unused = struct.unpack(sideInfoStruct, dataIn[:sideInfoLength])
                dataIn = dataIn[sideInfoLength:]
        except Exception as e:
            print(e)
            self.fail = 1

    def parseSDK3xDopplerHeatmap(self, dataIn):
        q = list(dataIn)
        numDopplerBins = self.num_doppler_bins
        numRangeBins = self.num_range_bins
        qidx = 0
        self.rangeDoppler = []

        numBytes = numDopplerBins * numRangeBins * 2
        try:

            chunk = q
            chunkE = list(chunk[0::2])
            chunkO = [element * 256 for element in list(chunk[1::2])]
            self.rangeDoppler = list(map(add, chunkE, chunkO))
        except Exception as e:
            print(e)
            self.fail = 1
    

    def parseSDK3xRadarCube(self, dataIn):
        NUM_ANGLE_BINS = 64
        q = list(dataIn)
        self.rangeFFT = []
        qrows = self.num_azimuth_antennas
        qcols = self.num_range_bins
        qidx = 0
        QQ = []
        try:

            for tmpc in range(qcols):
                complex_values = [complex(0,0)] * NUM_ANGLE_BINS
                real = [0] * qrows
                imag = [0] * qrows
                for tmpr in range(qrows):
                    real[tmpr] = q[qidx + 1] * 256 + q[qidx]
                    if (real[tmpr] > 32767):
                        real[tmpr] = real[tmpr] - 65536
                    imag[tmpr] = q[qidx + 3] * 256 + q[qidx + 2]
                    if (imag[tmpr] > 32767):
                        imag[tmpr] = imag[tmpr] - 65536
                    qidx = qidx + 4
                    complex_values[tmpr] = complex(real[tmpr],imag[tmpr])
                fftValues = []
                for j,k in zip(real,imag):
                    fftValues.extend((j,k))
                self.rangeFFT.append(fftValues)
                    

                fft_result = fft.transform(complex_values, False)
                real_result = [0] * NUM_ANGLE_BINS
                for ri in range(NUM_ANGLE_BINS):
                    cx = fft_result[ri]
                    real_result[ri] = math.sqrt(cx.real * cx.real + cx.imag * cx.imag)

                QQ.append(real_result[33:] + real_result[0:32])
            fliplrQQ = []

            for tmpr in range(len(QQ)):
                fliplrQQ.append(QQ[tmpr][::-1])

            self.radarCube = fliplrQQ
        except Exception as e:
            print(e)
            self.fail = 1