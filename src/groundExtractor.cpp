#include "groundExtractor.h"

/* -------------------------------------------------------------------------------------------------------------
	Function: First, compute the LPR value. It represents the average lowest height value from a set of points. 
			  Use this value + the seed threshold as the limit to consider a point as initial seed. 
	Parameters: 
	   - Original point cloud (vector <point_XYZIRL>)
	   - Seed point cloud (vector<point_XYZIRL>)
	   - Number of LPR points to calculate average LPR (int)
	   - Seed treshold
 ------------------------------------------------------------------------------------------------------------- */
void extractInitialSeedPoints(const std::vector<point_XYZIRL>& pointCloud, std::vector<point_XYZIRL>& seedPoints, int numLPR, float seedThresh) {
	// Compute average LPR
	float sum = 0.0;
	int count = 0;
	for (int i = 0; i < numLPR && i < pointCloud.size(); i++) {
		sum += pointCloud[i].z;
		count++;
	}

	// Extract seeds based on the LPR
	float avgLPR = count != 0 ? sum / count : 0.0; 
	seedPoints.clear();
	for (int i = 0; i < pointCloud.size(); i++) {
		if (pointCloud[i].z < avgLPR + seedThresh) {
			seedPoints.push_back(pointCloud[i]);
		}
	}
}

/* -------------------------------------------------------------------------------------------------------------
	Function: Compute mean value of each axis of the seeds. The values of these means are assumed to be good 
			  representative of the dispersion of the plane.
	Parameters: 
	   - Seed points (vector <point_XYZIRL>)
  ------------------------------------------------------------------------------------------------------------- */
Eigen::MatrixXf getSeedMeans(const std::vector<point_XYZIRL>& seeds) {
	Eigen::MatrixXf xyzMeans(3, 1);
	xyzMeans(0, 0) = xyzMeans(1, 0) = xyzMeans(2, 0) = 0.0;
	int numSeeds = seeds.size();
	for (int i = 0; i < numSeeds; i++) {
		xyzMeans(0, 0) += seeds[i].x; // x accum
		xyzMeans(1, 0) += seeds[i].y; // y accum
		xyzMeans(2, 0) += seeds[i].z; // z accum 
	}
	xyzMeans(0, 0) /= numSeeds; // x mean 
	xyzMeans(1, 0) /= numSeeds; // y mean
	xyzMeans(2, 0) /= numSeeds; // z mean
	return xyzMeans;	  
}

/* -------------------------------------------------------------------------------------------------------------
	Function: First, compute mean value of each axis from the current seeds. Based on the mean values, compute
	          the normal that represents the current plane model.To compute the normal (N) calculate the 
	          covariance matrix of the seed points. Then, obtain the vector with the least sparsity by computing
	          the U matrix using SVD. The vector with the least variance is the last column of this matrix. It 
	          is a good estimation of the plane's normal. 
	Parameters: 
	   - Seed points (vector <point_XYZIRL>)
  ------------------------------------------------------------------------------------------------------------- */
Eigen::MatrixXf estimatePlaneNormal(const std::vector<point_XYZIRL>& seeds, const Eigen::MatrixXf& xyzMeans) {
	
	float xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0;
	float xMean = xyzMeans(0, 0);
	float yMean = xyzMeans(1, 0);
	float zMean = xyzMeans(2, 0);
	int numSeeds = seeds.size();

	// Compute covariance matrix 
	for (int i = 0; i < numSeeds; i++) {
		float xTemp = seeds[i].x - xMean;
		float yTemp = seeds[i].y - yMean;
		float zTemp = seeds[i].z - zMean;
		xx += xTemp * xTemp;
		yy += yTemp * yTemp;
		zz += zTemp * zTemp;
		xy += xTemp * yTemp;
		xz += xTemp * zTemp;
		yz += yTemp * zTemp;
	} 
	Eigen::MatrixXf covarianceMat(3, 3); 
	covarianceMat << xx, xy, xz, 
	                 xy, yy, yz, 
	                 xz, yz, zz;
	covarianceMat /= numSeeds;
	
	// Compute the normal of the plane
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(covarianceMat, 0x04); // 0x04 --> Computes U matrix 
	return svd.matrixU().col(2); // normal
}

/* -------------------------------------------------------------------------------------------------------------
	Function: Compute the distance between the plane normal and a point in the point cloud
	Parameters: 
	   - Plane normal (MatrixXf)
	   - Point (MatrixXf)
  ------------------------------------------------------------------------------------------------------------- */
float getDistance(Eigen::MatrixXf vec3x1, Eigen::MatrixXf vec1x3) {
     return (vec3x1 * vec1x3)(0, 0);
}      