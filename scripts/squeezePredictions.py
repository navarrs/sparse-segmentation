from tqdm import tqdm
import numpy as np
import time as t
import itertools
import argparse
import os

start = t.time() # Program start

# Argument parsing 
ap = argparse.ArgumentParser()
ap.add_argument("--input_path",
				default="../results/squeeze/test1/",
		        help="Path to .npy predictions")
ap.add_argument("--output_path", 
				default="../results/squeeze/",
				help="Path to .txt files")
ap.add_argument("--num_cls", 
				default=4, 
				help="Specify number of classes.")
args = vars(ap.parse_args())

inputPath  = args["input_path"]
outputPath = args["output_path"]
numClasses = args["num_cls"]
colorMap = [' 125 125 125',   # unknown
            ' 220 0 220',     # car 
            ' 0 255 0',       # pedestrian 
            ' 0 0 255',       # cyclist 
            ' 255 0 0']       # ground	

# Create directory where to store .txt predicitons
version = 1
dirs = inputPath.split('/')
numDirs = len(dirs) 
newDirName = dirs[numDirs-2] + "_txt_v" + str(version)
predictionsDir = outputPath + newDirName

# Create directory where to save the .txt point clouds
while os.path.isdir(predictionsDir):
	print "Directory ", predictionsDir, "already exists. "
	version += 1
	newDirName = dirs[numDirs-2] + "_txt_v" + str(version)
	predictionsDir = outputPath + newDirName
print "Creating directory ", predictionsDir
os.makedirs(predictionsDir)	

# Convert all .npy to .txt
numberOfFiles = len(os.listdir(inputPath))
with tqdm(total=numberOfFiles) as progressBar:
	for filename in os.listdir(inputPath):
		pointCloudPath = inputPath + filename 
		data = np.load(pointCloudPath)

		outfile = open(predictionsDir + '/' + filename.split('.')[0] + ".txt", "a")
		for i, j in itertools.product(range(data.shape[0]), range(data.shape[1])):
		 	cls =  int(data[i, j, 5])
		 	arr = data[i, j, :]
		 	arr_str = ' '.join(str(x) for x in arr) + colorMap[cls] + '\n'  
		 	outfile.write(arr_str)
		
		outfile.close()
		progressBar.update(1)

print "Execution Time: ", t.time() - start, "s"