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
mkdir & cd build
cmake ..
make 
```

### How to use the ground labeler
To test if it works, run the following command:

```
./extractGround --inpath ../data/samples/ --outpath ../data/samples_out --seg 2 --lpr 20
```
It should create a folder called samples_out_1 in ./data/ where it will save the resulting point clouds. 

Modifying other parameters (example):
```
./extractGround --inpath ../data/samples/ --outpath ../data/samples_out --seg 2 --lpr 20 --iter 2 --thseed 1.0 --thdist 0.4
```
If not modified, it will use the default values. 

### Some results
Tested on a pointcloud with 64 scanlines (obtained from the KITTI dataset): 
<p align="center">
    <img src="./readme/rings64.png" width="600" />
  </p>
Tested on a pointclouds with 16 scanlines 
<p align="center">
    <img src="./readme/rings16.png" width="600" />
  </p>
<p align="center">
    <img src="./readme/rings16_2.png" width="600" />
  </p>
