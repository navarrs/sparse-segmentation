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

