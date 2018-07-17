from tqdm import tqdm
import numpy as np
import time as t
import itertools
import argparse
import sys
import os

file = '2011_09_26_0001_0000000000.npy'

pointCloud = np.load(file)

for i, j in itertools.product(range(pointCloud.shape[0]), range(pointCloud.shape[1])):
	x = float(pointCloud[i, j, 0])
	y = float(pointCloud[i, j, 1])
	z = float(pointCloud[i, j, 2])
	t.sleep(0.005)
	zenith = np.arcsin( z / np.sqrt(x*x + y*y + z*z)) * 180 / np.pi 
	azim = np.arcsin ( y / np.sqrt(x*x + y*y )) * 180 / np.pi

	print i, j, zenith, '\t', azim

