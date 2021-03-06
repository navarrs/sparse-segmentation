#include "pointCloud.h"

// Override operator>> for types point_XYZIRL
std::istream& operator>>(std::istream& is, point_XYZIRL& p) {
	return is >> p.x >> p.y >> p.z >> p.i >> p.r >> p.l >> p.n;
}

// Store point cloud from text file into point cloud vector 
int getPointCloud(std::string pathToFile, std::vector<point_XYZIRL>& pointCloud) {
	std::ifstream velodyneFile(pathToFile.c_str());	
	if (!velodyneFile.fail()) {
		point_XYZIRL point;
		while (velodyneFile >> point) { // Overload operator>>
			pointCloud.push_back(point); 
		}
		return 1;	
	} else return 0;
}

// Print point cloud coordinates
void printPointCloud(const std::vector<point_XYZIRL>& pointCloud, int num) {
	// Print coordinates of each point
	for (int i = 0; i < num; i++) {
		std::cout << "Point [" << i << "] with coordinates (x, y, z, i, r, l): (" 
		          << pointCloud[i].x << ", "
		          << pointCloud[i].y << ", "
		          << pointCloud[i].z << ", "
		          << pointCloud[i].i << ", " 
		          << pointCloud[i].r << ","
		          << pointCloud[i].l << ") " << std::endl;
	}
}

// Helper methods to sort the vectors 
bool compareN(point_XYZIRL& p1, point_XYZIRL& p2) {
	return p1.n < p2.n;	
} 
bool compareZ(point_XYZIRL& p1, point_XYZIRL& p2) {
	return p1.z < p2.z;
}
bool compareX(point_XYZIRL& p1, point_XYZIRL& p2) {
	return p1.x < p2.x;
}

// Sort point cloud based on chosen axis
void sortPointCloud(std::vector<point_XYZIRL>& pointCloud, std::vector<point_XYZIRL>& filtered, bool filter, std::string axis) {

	if (axis == "z") sort(pointCloud.begin(), pointCloud.end(), compareZ);	
	else if (axis == "x") sort(pointCloud.begin(), pointCloud.end(), compareX);	
	else sort(pointCloud.begin(), pointCloud.end(), compareN);
	
	// Filter noise due to mirror reflection
	if (filter) {
		std::vector<point_XYZIRL>::iterator iter = pointCloud.begin();
		for (int i = 0; i < pointCloud.size(); ++i) {
			if (pointCloud[i].z < THRESH_ERROR) iter++;
			else break;
		}
		filtered.insert(filtered.begin(), pointCloud.begin(), iter);
		pointCloud.erase(pointCloud.begin(), iter);
	}
}

// Convert point from point_XYZIRL to MatrixXf
Eigen::MatrixXf convertPointToMatXf(point_XYZIRL point) {
	Eigen::MatrixXf pointXf(1, 3);
	pointXf(0, 0) = point.x;
	pointXf(0, 1) = point.y;
	pointXf(0, 2) = point.z;
	return pointXf;
} 
