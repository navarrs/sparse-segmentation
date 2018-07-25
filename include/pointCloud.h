#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "includes.h"

/* 
 *	Stores point cloud from text file into a point cloud vector.
 *  
 *  @params 
 *  	path to file (string)
 * 		reference to pointcloud (vector<point_XYZIRL>) 
 *  @return 0 if successful, 1 if not
 */
int getPointCloud(std::string pathToFile, std::vector<point_XYZIRL>& pointCloud);

/* 
 *	Prints point cloud coordinates
 *  
 *  @params 
 * 		reference to pointcloud (vector<point_XYZIRL>)
 *      number of values to print (int) 
 *  @return void
 */
void printPointCloud(const std::vector<point_XYZIRL>& pointCloud, int num);

/* 
 *	Sorts point cloud based on chosen axis
 *  
 *  @params 
 * 		reference to pointcloud (vector<point_XYZIRL>)
 *      reference to filter values - outliers (vector<point_XYZIRL>)
 *      apply filtering (bool)
 *      sorting axis (string)
 *  @return void
 */
void sortPointCloud(std::vector<point_XYZIRL>& pointCloud, std::vector<point_XYZIRL>& filtered, bool filter, std::string axis);

/* 
 *	Convers LIDAR points from vector to MatrixXf
 *  
 *  @params 
 * 		point from point cloud (point_XYZIRL)
 *  @return point from point cloud (MatrixXf)
 */
Eigen::MatrixXf convertPointToMatXf(point_XYZIRL point);


#endif
