# Semantic Segmentation of Sparse Point Clouds using SqueezeSeg - RISS 2018

- Point Cloud ground annotator using a Ground extraction algorithm based on: Fast segmentation of 3D point clouds: a paradigm on LIDAR data (Dimitris Zermas)
- Data downsampler to simulate VLP16 LIDAR scanners from VLP64 LIDAR scans obtained from the Squeezeseg: Convolutional neural nets with recurrent crf for real-time road-object segmentation from 3d lidar point cloud (https://github.com/BichenWuUCB/SqueezeSeg) 

### Prerequisites

```
For the ground extraction algorithm (developed in C++)
- Eigen3
- Boost
For the downsampler (developed in Python 2.7)
- Python 2.7
```

### Installing

```
cd build
cmake CMakeLists.txt
make 

```

### How to use

1. txtConvert.py 
I use this code to convert the VLP64 point cloud files in .npy format .txt format.
 
2. mainGroundSqueeze.cpp 
I use .txt files obtained from the previous step to automatically annotate ground points using my ground annotation tool in c++. 
TODO: modify this code to do real-time segmentation. 
TODO: implement a python script with this algorithm to avoid having to convert form .npy to .txt and back. 

3. npyConvert.py
I use this code to convert the annotated files (.txt) from the previous step into .npy files. 

4. downSamplerPCL.py 
I use this code to downsample the pointclouds in .npy format from the previous step to simulate VLP32 and VLP16 lidar scans. 

# Still working on this
6. createSets.py
Create training and validations sets for each point cloud list. 

7. SqueezeSeg
Train SqueezeSeg Network with the downsampled files. 

8. demo_txt.py (from squeezeseg) and squeezePredictions.py 
Make predictions
