#ifndef GROUNDEXTRACTOR_H
#define GROUNDEXTRACTOR_H

#include "includes.h"

Eigen::MatrixXf estimatePlaneNormal(const std::vector<point_XYZIRL>& seeds, const Eigen::MatrixXf& means);
Eigen::MatrixXf getSeedMeans(const std::vector<point_XYZIRL>& seeds);
Eigen::MatrixXf getSeedMedians(const std::vector<point_XYZIRL>& seeds);
float getDistance(Eigen::MatrixXf vec3x1, Eigen::MatrixXf vec1x3);
void extractInitialSeedPoints(const std::vector<point_XYZIRL>& pointCloud, std::vector<point_XYZIRL>& seedPoints, int numLPR, float seedThresh, bool method);

#endif
