"""
author: Ingrid Navarro 
date: July 11th, 2018

Converting from .npy to .txt 
	python convert.py --inpath ../data/squeeze/ \
	                  --outpath ../data/ground/ \
					  --conv txt

Converting from .txt to .npy
	python convert.py --inpath ../data/ground/lidar_ng/ \
	   				  --outpath ../data/squeeze/ \
					  --outdir lidar_64 \
					  --conv npy

Downsampling pointclouds
   python convert.py --inpath ../data/squeeze/lidar_ng/ \
   				     --outpath ../data/squeeze/ \
   				     --conv downs
Merging classes 
	python convert.py --inpath ../data/squeeze/ \
					  --outpath ../data/squeeze/ \
					  --convs mrg 
"""
from tqdm import tqdm
from statistics import mode
import numpy as np
import time as t
import itertools
import argparse
import sys
import os

def makeDir(path, dirN, gnd):
	"""
	Create output directory. Provide path, directory name, and ID to indicate if it was 
	labeled with / without ground. 
		e.g. g_v --> ground, ng_v --> non-ground. 
	The method will create a new version of the directory if it already exists.  
	"""
	ver = 1
	newDir = gnd + "_" + dirN + "_v" + str(ver)
	fullPath = path + newDir
	while os.path.isdir(fullPath):
		print "Directory {} exists.".format(fullPath)
		ver += 1
		newDir = gnd + "_" + dirN + "_v" + str(ver)
		fullPath = path + newDir
	print "Creating directory {}".format(fullPath)
	os.makedirs(fullPath)
	return fullPath

def npyConvert(ipath, opath, azimuth, zenith):
	"""
	Convert point cloud files in .txt format to .npy format to be processed by the SqueezeSeg
	network. Provide input path where the .txt files are and full path of directory created to 
	save the .npy files. The method will create and .npy files with shape (64, 512, 6) as 
	indicated in the SqueezeSeg approach. 
	"""
	numFiles = len(os.listdir(ipath))
	params = 6  # x, y, z, i, r, l
	npyArr = np.zeros((zenith, azimuth, params))

	print "\nProcessing  {:} .txt files from: {} ".format(numFiles, ipath)
	with tqdm(total=numFiles) as progressBar:
		for file in os.listdir(ipath):
			with open(ipath + file, 'r') as f:
			 	txtArr = np.loadtxt(ipath + file)
			 	# Convert .txt into a (64, 512, 6) np.array
			 	currentMin = 0
				currentMax = azimuth
			 	for i in range(zenith):
			 		npyArr[i, :, :] = txtArr[currentMin:currentMax, 0:6]
			 		currentMin = currentMax
			 		currentMax += azimuth
			# Save to .npy file 
			npyFilename = '/' + file.split('.')[0] + '.npy'
			np.save(opath + npyFilename, npyArr)
			progressBar.update(1)

def txtConvert(ipath, opath):
	"""
	Convert point cloud files in .npy format to .txt format to be processed and annotated by the 
	Ground Filtration Algorithm. Provide input path where the .npy files are and full path of 
	directory created to save the .txt files. 
	"""
	numFiles = len(os.listdir(ipath))	
	print "\nProcessing {:} .npy files from: {}".format(numFiles, ipath)
	with tqdm(total=numFiles) as progressBar:
		for file in os.listdir(ipath):
			# Read current point cloud and create file 
			pointCloudPath = ipath + file
			pointCloud = np.load(pointCloudPath).astype(np.float32)
			txtFilename = '/' + file.split('.')[0] + '.txt'
			outfile = open(opath + txtFilename, 'a')

			# Each line has a format [X Y Z I R L]
			num = 0
			for i, j in itertools.product(range(pointCloud.shape[0]), range(pointCloud.shape[1])):
				xyzirl = pointCloud[i, j, :] 
				xyzirl_str = ' '.join(str(k) for k in xyzirl) + ' ' + str(num) + '\n'
				num += 1
				outfile.write(xyzirl_str)
			outfile.close()
			progressBar.update(1)

def mergeClass(ipath, opath, cls1, cls2):
	"""
	Merge classes on the point cloud.
	"""
	numFiles = len(os.listdir(ipath))	
	print "\nProcessing {:} .npy files from: {}".format(numFiles, ipath)
	with tqdm(total=numFiles) as progressBar:
		for file in os.listdir(ipath):
			# Read current point cloud and create file 
			pointCloudPath = ipath + file
			pointCloud = np.loadtxt(pointCloudPath)
		
			txtFilename = "/" + file.split('.')[0] + '.txt'
			outfile = open(opath + txtFilename, 'a')

			# Each line has a format [X Y Z I R L]
			num = 0
			for i in itertools.product(range(pointCloud.shape[0])):
				xyzirl = pointCloud[i] 
			 	if int(xyzirl[5]) == cls2:
			 		xyzirl[5] = cls1

				xyzirl_str = ' '.join(str(k) for k in xyzirl) + ' ' + str(num) + '\n'
			 	num += 1
			 	outfile.write(xyzirl_str)
			outfile.close()
			progressBar.update(1)



def downSample(ipath, opath):
	"""
	Convert the .npy point cloud dataset of the VLP64 into .npy versions of a VLP32 and VLP16. 
	Provide input path of the name of the folder to downsample and the base name for the output 
	path where the new datasets will be saved. 
	"""
	# Down sampled datasets 
	outDirs    = ["vlp32",  "vlp16d", "vlp16m", "vlp16u" ]
	filenameID = ["z32_", "z16d_", "z16m_", "z16u_"]

	# Make the output directories
	outDirs = [ makeDir(opath, outDirs[p], "g") for p in range(len(outDirs)) ]

	# Start the downsampling 
	numFiles = len(os.listdir(ipath))
	print "\nDownsampling {} point clouds from: {}".format(numFiles, ipath)
	with tqdm(total=numFiles) as progressBar:
		for file in os.listdir(ipath):	
			baseFilename = file.split('.')[0]
			basePath = ipath + file

			pointCloud64 = np.load(basePath).astype(np.float32)
			xyzirl = [[] for _ in range(4)] 

			# The zenith level indicates the lidar ring 
			for zenithLevel in range(pointCloud64.shape[0]): 

				if zenithLevel % 2 == 0: # Downsampled to a 32 beam lidar 
					xyzirl[0].append(pointCloud64[zenithLevel, :, :])

				if zenithLevel % 4 == 1: # Downsampled to a 16 beam lidar
					xyzirl[1].append(pointCloud64[zenithLevel-1, :, :]) # 0 to 59 (every 4) 
					xyzirl[2].append(pointCloud64[zenithLevel  , :, :]) # 1 to 60 (every 4)
					xyzirl[3].append(pointCloud64[zenithLevel+1, :, :]) # 2 to 61 (every 4)

			# Transform lists to .npy files 
			for pcl in range(len(outDirs)):
			 	fullFilename = outDirs[pcl] + '/' + filenameID[pcl] + baseFilename
			 	downsampledPointCloud = np.array(xyzirl[pcl])  
			 	np.save(fullFilename, downsampledPointCloud)

			progressBar.update(1)

def upSample(ipath, opath):
	"""
	Convert the .npy point cloud dataset of the VLP16 into .npy version of a VLP32. 
	Provide input path of the name of the folder to downsample and the base name for the output 
	path where the new datasets will be saved. 
	"""
	# Start the downsampling 
	numFiles = len(os.listdir(ipath))
	print "\nUpsampling {} point clouds from: {}".format(numFiles, ipath)
	with tqdm(total=numFiles) as progressBar:
		for file in os.listdir(ipath):	
			baseFilename = file.split('.')[0]
			basePath = ipath + file
			pointCloud32 = np.zeros((32, 512, 6))
			
			pointCloud16 = np.load(basePath).astype(np.float32)
			k = 0
			d = 0.5
			for i in range(pointCloud16.shape[0]):
				interm = np.zeros((512, 6))
				xyzirl1 = pointCloud16[i, :, :]
				if i == pointCloud16.shape[0]-1:
				 	xyzirl2 = np.zeros((512, 6))
				else:
				 	xyzirl2 = pointCloud16[i+1, :, :]
				
				xyzirl2[xyzirl2 == 0] = np.nan
				xyzirl1[xyzirl1 == 0] = np.nan
				D = np.sqrt((xyzirl2[:,2]-xyzirl1[:,2])**2 +(xyzirl2[:,1]-xyzirl1[:,1])**2) # sqrt((z2-z1)**2 + (y2-y1)**2)
				
				# Get X				
				interm[:,0] = (xyzirl2[:,0]+xyzirl1[:,0]) / 2 # (x1 + x2) / 2
				# Get Y 
				interm[:,1] = xyzirl1[:,1] + d*(xyzirl2[:,1]-xyzirl1[:,1]) / D  # y1 + d * (y2-y1) / D
				# Get Z 
				interm[:,2] = xyzirl1[:,2] + d*(xyzirl2[:,2]-xyzirl1[:,2]) / D # z1 + d * (z2-z1) / D
				# Get I 
				interm[:,3] = (xyzirl2[:,3]+xyzirl1[:,3]) / 2 # (i1 + i2) / 2
				# Get R
				interm[:,4] = (xyzirl2[:,4]+xyzirl1[:,4]) / 2 # (r1 + r2) / 2				
				xyzirl1[np.isnan(xyzirl1)] = 0
				xyzirl2[np.isnan(xyzirl2)] = 0
				interm[np.isnan(interm)] = 0

				# Get Label 
				label = np.column_stack((xyzirl1[:,5], xyzirl2[:,5]))
				interm[:,5] = label.max(1)

				pointCloud32[k,  :,:] = xyzirl1
				pointCloud32[k+1,:,:] = interm
				k += 2
				if k == 32:
				 	k = 0
				npyFilename = '/' + file.split('.')[0] + '.npy'
				np.save(opath + npyFilename, pointCloud32)

			progressBar.update(1)

def main():
	# Argument parsing 
	ap = argparse.ArgumentParser()
	ap.add_argument("--inpath",  required=True,   help="Path to .npy files")
	ap.add_argument("--outpath", required=True,   help="Path to .txt files")
	ap.add_argument("--conv",    required=True,   help="'npy' or 'txt': type of conversion")
	ap.add_argument("--outdir",  default="vlp64", help="Name of output directory.")
	ap.add_argument("--azimuth", default=512,     help="Azimuth used in processed point clouds")
	ap.add_argument("--zenith",  default=64,      help="Number of point cloud rings")
	ap.add_argument("--cls1",    default=2,       help="Class to keep")
	ap.add_argument("--cls2",    default=3,       help="Class to merge")
	ap.add_argument("--label",   default='g',     help="ID to identify type of dataset")
	args = vars(ap.parse_args())

	inPath   = args["inpath"]
	outPath  = args["outpath"]
	outDir   = args["outdir"]
	azimuth  = args["azimuth"]
	zenith   = args["zenith"]
	convType = args["conv"]
	clsKeep  = args["cls1"]
	clsMerge = args["cls2"]
	label    = args["label"]

	# Process data
	start = t.time() # Program start
	if convType == "npy": 
		npyConvert(inPath, makeDir(outPath, outDir, label), azimuth, zenith)
	elif convType == "txt":
		txtConvert(inPath, makeDir(outPath, outDir, label))
	elif convType == "down":
		downSample(inPath, outPath)
	elif convType == "up":
		upSample(inPath, makeDir(outPath, outDir, label))
	elif convType == "merge":
		mergeClass(inPath, makeDir(outPath, outDir, label), clsKeep, clsMerge)
	else:
		print "Error: invalid argument"
		exit()
	print "Execution Time: ", t.time() - start, "s"

if __name__ == '__main__':
	main()
