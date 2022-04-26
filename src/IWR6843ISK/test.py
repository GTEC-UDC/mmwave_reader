import numpy as np

NUM_ANGLE_BINS = 64
numRangeBins = 256
digOutSampleRate = 5209
freqSlopeConst_actual = 70
numAntennas = 8

rangeIdxToMeters = 3e8 * digOutSampleRate * 1e3 / (2 * abs(freqSlopeConst_actual)* 1e12 * numRangeBins)
angles = np.arange(-NUM_ANGLE_BINS/2, NUM_ANGLE_BINS/2)*(2/NUM_ANGLE_BINS)
theta = np.arcsin(np.deg2rad(angles))
range = np.arange(0,numRangeBins) * rangeIdxToMeters


posX = np.multiply(range.reshape(1,np.size(range)),np.sin(theta).reshape(np.size(theta),1)).T
posY = np.multiply(range.reshape(1,np.size(range)),np.cos(theta).reshape(np.size(theta),1)).T

x = posX.ravel()
y = posY.ravel()
pass