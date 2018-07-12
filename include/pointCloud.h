#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "includes.h"

Eigen::MatrixXf convertPointToMatXf(point_XYZIRL point);
int getPointCloud(std::string pathToFile, std::vector<point_XYZIRL>& pointCloud);
void copyPointCloud(const std::vector<point_XYZIRL> & pointCloud1, std::vector<point_XYZIRL>& pointCloud2);
void printPointCloud(const std::vector<point_XYZIRL>& pointCloud, int num);
void sortPointCloud(std::vector<point_XYZIRL>& pointCloud, bool filter);

#endif
