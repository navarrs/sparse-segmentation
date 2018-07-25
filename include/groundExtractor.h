#ifndef GROUNDEXTRACTOR_H
#define GROUNDEXTRACTOR_H

#include "includes.h"

/* 
 *	Computes the initial seeds used to estimate the ground plane. It first computes 
 *	an average representative of the ground (LPR) from an arbitrary number of points.
 *  Then it uses this value as a threshold to determine if a point will be used as a
 *  seed point.  
 *
 *  @params 
 *  	reference to pointcloud (vector<point_XYZIRL>)  
 * 		reference to the seeds (vector<point_XYZIRL>)
 *      number of points needed to estimate LPR (int)
 *      seed threshold for LPR (float)
 *      method to use: means / medians (bool)
 *  @return 0 if successful, 1 if not
 */
void extractInitialSeedPoints(const std::vector<point_XYZIRL>& pointCloud, std::vector<point_XYZIRL>& seedPoints, int numLPR, float seedThresh, bool method);

/* 
 *	Computes the median value of each coordinate axis of the seeds. The resulting
 *	values are considered to be good representatives of the plane values. Also, using
 *  the medians allows to get rid of the outliers, but the method is slower.  
 *
 *  @params   
 * 		reference to the seeds (vector<point_XYZIRL>)
 *  @return vector (MatrixXf) with median values of x,y,z of the seeds
 */
Eigen::MatrixXf getSeedMedians(const std::vector<point_XYZIRL>& seeds);

/* 
 *	Computes the mean value of each coordinate axis of the seeds. The resulting
 *	values are considered to be good representatives of the plane values. Also, when
 *  using the means, outliers might significantly affect the result, but the method
 *  is faster than using medians.  
 *
 *  @params   
 * 		reference to the seeds (vector<point_XYZIRL>)
 *  @return vector (MatrixXf) with mean values of x,y,z of the seeds
 */
Eigen::MatrixXf getSeedMeans(const std::vector<point_XYZIRL>& seeds);

/* 
 *	Computes the normal representative of the plane model using SVD. Given the seed
 *  means, compute the covariance matrix of the seeds to analize the sparsity of the 
 *  values. The vector with the least sparsity will be used to represent the plane. This
 *  vector corresponds to the last vector of the U matrix from the SVD method. 
 *
 *  @params   
 * 		reference to the seeds (vector<point_XYZIRL>)
 *      mean values of the seeds (MatrixXf)
 *  @return normal of the model (MatrixXf) 
 */
Eigen::MatrixXf estimatePlaneNormal(const std::vector<point_XYZIRL>& seeds, const Eigen::MatrixXf& means);

/* 
 *	Computes the distance between the plane normal and a point in the point cloud. 
 *
 *  @params   
 * 		normal (MatrixXf)
 *      point (MatrixXf)
 *  @return distance (float)
 */
float getDistance(Eigen::MatrixXf vec3x1, Eigen::MatrixXf vec1x3);


#endif
