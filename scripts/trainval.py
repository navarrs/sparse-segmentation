import random
import argparse
import os

def makeDir(path, dirN):
	"""
	Create output directory. Provide path, directory name, 
	and ID to indicate if it was label with / withou ground. 
		e.g. g_v --> ground, ng_v --> non-ground. 
	
	The method will create a new version of the directory if 
	it already exists.  
	"""
	ver = 1
	newDir = str(ver) + "_" + dirN
	fullPath = path + newDir
	while os.path.isdir(fullPath):
		print "Directory {} exists.".format(fullPath)
		ver += 1
		newDir = str(ver) + "_" + dirN 
		fullPath = path + newDir
	print "Creating directory {}".format(fullPath)
	os.makedirs(fullPath)
	return fullPath

if __name__ == '__main__':

	# Argument parsing 
	ap = argparse.ArgumentParser()
	ap.add_argument("--inpath", required=True, 
			        help="Path to .npy files")
	ap.add_argument("--outdir", default="list", 
					help="Name of output directory.")
	ap.add_argument("--train", default=80, 
					help="Percentage for training set")
	args = vars(ap.parse_args())

	inPath = args["inpath"]
	outDir = args["outdir"]
	train  = args["train"]

	for directory in os.listdir(inPath):
		currentDir = inPath + directory +"/"
		outPath = makeDir(currentDir, outDir)

		txtAll = open(outPath + "/" + "all.txt", 'a')
		txtTrain = open(outPath + "/" + "train.txt", 'a')
		txtVal = open(outPath + "/" + "val.txt", 'a')
		
		files = []
		for file in os.listdir(currentDir):
			if file.endswith(".npy"):
				filename = file.split('.')[0]
				files.append(filename)
				txtAll.write(filename + '\n')
		shuffled = random.sample(files, len(files))
		perc = int(train) * len(shuffled) / 100 
		txtTrain.write("\n".join(shuffled[:perc]))
		txtVal.write("\n".join(shuffled[perc:]))

		txtAll.close()
		txtTrain.close()
		txtVal.close()

