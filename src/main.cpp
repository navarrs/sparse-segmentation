/*	
	Sun 01 July 2018 19:08:52 PM EDT 
	Ground Labeling Algorithm - GLA
	By Ingrid Navarro 
*/
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#include <stdio.h>
#include <sys/stat.h>
#include <time.h>
#include <dirent.h>
#include <math.h>

#include "groundExtractor.h"
#include "includes.h"
#include "pointCloud.h"

namespace po = boost::program_options;
using Eigen::JacobiSVD;
using Eigen::VectorXf;
using Eigen::MatrixXf;

DIR * dir;
struct dirent *ent;
struct stat sb;
int getFiles(std::string path, std::vector<std::string>& files);
void saveToFile(const std::vector<point_XYZIRL>& pointCloud, std::string dirName, std::string name, bool type);

int main(int argc, char* argv[]) {
	// Parse arguments
	po::options_description description("Usage");
	description.add_options()
		("help", "Program usage.")
		("inpath",  po::value<std::string>()->default_value("data/ground/lidar_ng"), "Input pointclouds path")
		("outpath", po::value<std::string>()->default_value("data/ground/"), "Output pointclouds file")
		("lpr",     po::value<int>()->default_value(20), "Number of seeds needed to get the LPR")
		("seg",     po::value<int>()->default_value(1), "Number of segments along the x-axis")
		("iter",    po::value<int>()->default_value(3), "Number of times needed to estimate the ground plane")
		("thseed",  po::value<float>()->default_value(1.2), "Seeds threshold value")
		("thdist",  po::value<float>()->default_value(0.3), "Distance threshold value")
		("method",  po::value<bool>()->default_value(true), "Use means or medians");
	po::variables_map opts;
	po::store(po::command_line_parser(argc, argv).options(description).run(), opts);
	try { 
		po::notify(opts);	
	} catch (std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return 1;
	}
	if (opts.count("help")) { 
		std::cout << description; 
		return 1; 
	}
	std::string inputPath  = opts["inpath"].as<std::string>();
	std::string outputPath = opts["outpath"].as<std::string>();
	int numLPR        = opts["lpr"].as<int>();
	int numSegments   = opts["seg"].as<int>();
	float numIters    = opts["iter"].as<int>();
	float seedThresh  = opts["thseed"].as<float>();
	float distThresh  = opts["thdist"].as<float>();
	bool method       = opts["method"].as<bool>();

	// --------------------------------------------------------------------------------------------
	// Start algorithm
	std::cout 
		<< " --------------------------------------- " << std::endl	
    	<< "|      Ground Extraction Algorithm      |" << std::endl
    	<< " --------------------------------------- " << std::endl
	    << " ------------ CONFIGURATION " << std::endl
	    << " --- Reading point cloud files from: " << inputPath << std::endl
	    << " --- Saving annotated files in: " << outputPath << std::endl
	    << " --- Num of iterations: " << numIters << std::endl
	    << " --- Num of segments along the x-axis: " << numSegments << std::endl
	    << " --- Num to calculate LPR: " << numLPR << std::endl
	    << " --- Seeds threshold: " << seedThresh << std::endl
	    << " --- Distance threshold: " << distThresh << std::endl << std::endl
	    << " ---------------- START " << std::endl; 
	
	// Get all files to be process from the specified directory 
	std::vector<std::string> files; 
	if (getFiles(inputPath, files)) return 1;
	
	// Create output directory
	int version = 0;
	std::string newDir;
	do { 
		newDir = outputPath + boost::lexical_cast<std::string>(++version);
	} while (stat(newDir.c_str(), &sb) == 0); // create new version if file exists
	std::cout << "Creating directory " << newDir << std::endl;
	mkdir(newDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); 

	// Annotate ground points
	clock_t startTime = clock(); 
	for (int i = 0; i < files.size(); i++) {	

		std::vector<point_XYZIRL> pointCloud;
		std::vector<point_XYZIRL> labeledPointCloud;
		std::vector<point_XYZIRL> filteredPoints;
		std::string filename = files[i];
		std::string tempPath = inputPath + filename;

	    // Read point cloud 
	 	if (!getPointCloud(tempPath, pointCloud)) {
	 		std::cout << "ERROR: could not locate file." << std::endl;
	 		return 0;
	 	}

		// Sort point cloud on x-xis.
		sortPointCloud(pointCloud, filteredPoints, false, "x");

		// Create subpointclouds and run algorithm on every subpointcloud
		size_t chunk = ceil((double)pointCloud.size() / numSegments);
		std::vector<point_XYZIRL>::iterator start = pointCloud.begin();
		int pcl = 0;
		int count = 0;
		for(size_t it = 0; it < pointCloud.size(); it += chunk) {
    		
    		// Create subpointcloud
    		std::vector<point_XYZIRL> sortedPointCloudOnZ(start+it, start+std::min<size_t>(it+chunk, pointCloud.size()));
    		filteredPoints.clear();
    		sortPointCloud(sortedPointCloudOnZ, filteredPoints, true, "z");

    		// Extract initial seeds 
			std::vector<point_XYZIRL> seeds;
			extractInitialSeedPoints(sortedPointCloudOnZ, seeds, numLPR, seedThresh, method);
			
			// Estimate plane 
			if (seeds.size()) {
			 	for (int iter = 0; iter < numIters; iter++) {	
					//	The linear model to solve is: ax + by +cz + d = 0
			        //   		where; N = [a b c]     X = [x y z], 
					//		           d = -(N.transpose * X)
					MatrixXf xyzM; // Means or medians
					MatrixXf normal;
					if (method) {
						xyzM  = getSeedMeans(seeds);
					} else {
						xyzM = getSeedMedians(seeds);
					}
			 		normal = estimatePlaneNormal(seeds, xyzM);
				    float negDist = -(normal.transpose() * xyzM)(0, 0); // d = -(n.T * X)
				    float currDistThresh = distThresh - negDist;            // Max ground distance of current model
			        
			        // Calculate the distance for each point and compare it with current threshold to 
			        // determine if it is a ground point or not. 
			        seeds.clear();           			
			        if (iter < numIters-1) {  // Continue estimating plane
				        for (int i = 0; i < sortedPointCloudOnZ.size(); i++) {
				    		MatrixXf point = convertPointToMatXf(sortedPointCloudOnZ[i]);		
				   			if (getDistance(point, normal) < currDistThresh) {
				   				seeds.push_back(sortedPointCloudOnZ[i]);
				   			}
						}
			        } else { // Label final point cloud segment
			        	for (int i = 0; i < sortedPointCloudOnZ.size(); i++) {
				    		MatrixXf point = convertPointToMatXf(sortedPointCloudOnZ[i]);		
				   			if (getDistance(point, normal) < currDistThresh && sortedPointCloudOnZ[i].l == 0) {
				   				sortedPointCloudOnZ[i].l = GROUND_LABEL; 	
				   				count++;
				   			}
						}	
						// Add filtered points to the point cloud
						labeledPointCloud.insert(labeledPointCloud.end(), sortedPointCloudOnZ.begin(), sortedPointCloudOnZ.end());  
						labeledPointCloud.insert(labeledPointCloud.end(), filteredPoints.begin(), filteredPoints.end());
			        } 
				}
			} else std::cout << "No seeds extracted." << std::endl;
		}
		std::cout << "Processed files: " << i + 1 << "/" << files.size() << " with " << count << " of " << labeledPointCloud.size() <<  " ground points" << std::endl;
		sortPointCloud(labeledPointCloud, filteredPoints, false, "n"); // Sort based on the ring
		saveToFile(labeledPointCloud, newDir, filename, true); 
	}
	clock_t finishTime = clock();
	std::cout 
		 << " ---------------- DONE. " << std::endl
		 << "Total execution time: " 
		 << (finishTime - startTime) / double(CLOCKS_PER_SEC) << "s" << std::endl; 
	return 0;
}

/* -------------------------------------------------------------------------------------------------------------
	Function: Save point cloud to file 
	Parameters:
	   - Point from point cloud (point_XYZIRL)
	   - directory (string) & name (string)
	   - type of pointcloud (ground or not ground)
   ------------------------------------------------------------------------------------------------------------- */
void saveToFile(const std::vector<point_XYZIRL>& pointCloud, std::string dirName, std::string name, bool type) {
	int version = 0;
	std::ofstream textfile;
	std::string filename = dirName + "/" + name;
	 	
	textfile.open(filename.c_str(), std::fstream::app);
	for (int i = 0; i < pointCloud.size(); i++) {
		textfile << pointCloud[i].x << " " << pointCloud[i].y << " " 
	   		     << pointCloud[i].z << " " << pointCloud[i].i << " " 
	  		     << pointCloud[i].r << " " << pointCloud[i].l << std::endl;
	}
	textfile.close();
}

/* -------------------------------------------------------------------------------------------------------------
	Function: Get all the files from specified directory
	Parameters:
	   - Input path (string)
	   - Vector to store filenames (vector<string>)
   ------------------------------------------------------------------------------------------------------------- */
int getFiles(std::string path, std::vector<std::string>& files) {
	if ((dir = opendir(path.c_str())) == NULL) {
		std::cout << "Error(" << errno << ") opening " << dir << std::endl;
		return errno;
	}
	
	// Get files
	char *p1;
	char *p2;
	while((ent = readdir(dir)) != NULL) {
		std::string file = ent->d_name;
		p1 = strtok(ent->d_name, ".");
		p2 = strtok(NULL, ".");
		if (p2 != NULL) {
			if (strcmp(p2, "txt") == 0) {
				files.push_back(std::string(file));
			}
		}
	}
	closedir(dir);
	return 0;
}