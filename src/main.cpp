/*
 *	Last modified: Wed 25 July 2018 9:24:52 AM EDT 
 *  @brief: GLA - Ground Labeling Algorithm
 *		   A tool to annotate ground points from a given point cloud. 
 *         Based on: Fast segmentation of 3D point clouds: A paradigm on LIDAR data (Dimitris Zermas)
 *  @file: main.cpp
 *  @author: Ingrid Navarro
 *  @version: 1.2
 */
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

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
namespace fs = boost::filesystem;
using namespace std;
using Eigen::JacobiSVD;
using Eigen::VectorXf;
using Eigen::MatrixXf;

DIR * dir;
struct dirent *ent;
struct stat sb;

/* 
 *	Gets all point cloud files from specified directory
 *  
 *  @params 
 *  	point clouds input path (string)
 * 		vector to store filenames (vector<string>) 
 *  @return 0 if successfull, error if not. 
 */
int getFiles(string path, vector<string>& files);

/* 
 *	Saves point cloud to text file after computing GLA
 *  
 *  @params 
 *  	point cloud (vector<Point_XYZIRL>)
 * 		file path (string)	 
 *  @return void
 */
void saveToFile(const vector<point_XYZIRL>& pointCloud, string filepath);

// GLA - Ground Labeling Algorithm 
int main(int argc, char* argv[]) {
	
	// Parse arguments
	po::options_description description("Usage");
	description.add_options()
		("help", "Program usage.")
		("inpath",  po::value<string>()->default_value("../data/sample/textfiles1/"),"Path to input point clouds.")
		("outpath", po::value<string>()->default_value("../data/sample/"), "Path to save output point clouds.")
		("lpr",     po::value<int>()->default_value(20),                   "Num. of seeds needed to get the LPR.")
		("seg",     po::value<int>()->default_value(1),                    "Num. of segments along the x-axis.")
		("iter",    po::value<int>()->default_value(3), 				   "Num. of plane estimations per segment.")
		("thseed",  po::value<float>()->default_value(1.2),                "Max. value to determine a seed.")
		("thdist",  po::value<float>()->default_value(0.3), 			   "Max. value to determine ground distance.")
		("method",  po::value<bool>()->default_value(true), 			   "Use means or medians to extract seeds.");
	po::variables_map opts;
	po::store(po::command_line_parser(argc, argv).options(description).run(), opts);
	try { 
		po::notify(opts);	
	} catch (exception& e) {
		cerr << "Error: " << e.what() << endl;
		return 1;
	}
	if (opts.count("help")) { 
		cout << description; 
		return 1; 
	}
	string inputPath  = opts["inpath"].as<string>();
	string outputPath = opts["outpath"].as<string>();
	int numLPR        = opts["lpr"].as<int>();
	int numSegments   = opts["seg"].as<int>();
	float numIters    = opts["iter"].as<int>();
	float seedThresh  = opts["thseed"].as<float>();
	float distThresh  = opts["thdist"].as<float>();
	bool method       = opts["method"].as<bool>();

	// Start algorithm
	cout << " --------------------------------------- " << endl	
    	 << "|      Ground Extraction Algorithm      |" << endl
    	 << " --------------------------------------- " << endl
	     << "[ CONFIGURATION ] " << endl
	     << "  >> Reading point cloud files from: " << inputPath << endl
	     << "  >> Saving annotated files in: " << outputPath << endl
	     << "  >> Num of iterations: " << numIters << endl
	     << "  >> Num of segments along the x-axis: " << numSegments << endl
	     << "  >> Num to calculate LPR: " << numLPR << endl
	     << "  >> Seeds threshold: " << seedThresh << endl
	     << "  >> Distance threshold: " << distThresh << endl << endl
	     << "[ START ] " << endl; 
	
	// Get all files to be process from the specified directory 
	vector<string> files; 
	if (getFiles(inputPath, files)) {
		return 1;	
	} else {
		cout << "  >> Processing " << files.size() << " files" << endl;
	}
	
	// Create output directory
	int version = 0;
	string newDir;
	fs::path p(inputPath);
	string name = p.parent_path().filename().string(); // Get name of the point cloud directory
	do { 
		newDir = outputPath + "g_" + name + "_" + boost::lexical_cast<string>(++version);
	} while (stat(newDir.c_str(), &sb) == 0); // Create new version if directory exists
	cout << "  >> Creating directory " << newDir << endl;
	mkdir(newDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); 

	// Annotate ground points
	clock_t startTime = clock(); 
	for (int i = 0; i < files.size(); i++) {	

		vector<point_XYZIRL> pointCloud;
		vector<point_XYZIRL> labeledPointCloud;
		vector<point_XYZIRL> filteredPoints;
		string filename = files[i];
		string tempPath = inputPath + filename;

	    // Read point cloud 
	 	if (!getPointCloud(tempPath, pointCloud)) {
	 		cout << "ERROR: could not locate file." << endl;
	 		return 0;
	 	}

		// Sort point cloud on x-xis.
		sortPointCloud(pointCloud, filteredPoints, false, "x");

		// Create subpointclouds and run algorithm on every subpointcloud
		size_t chunk = ceil((double)pointCloud.size() / numSegments);
		vector<point_XYZIRL>::iterator start = pointCloud.begin();
		int count = 0;
		for(size_t it = 0; it < pointCloud.size(); it += chunk) {
    		
    		// Create subpointcloud
    		vector<point_XYZIRL> sortedPointCloudOnZ(start+it, start+std::min<size_t>(it+chunk, pointCloud.size()));
    		filteredPoints.clear();
    		sortPointCloud(sortedPointCloudOnZ, filteredPoints, true, "z");

    		// Extract initial seeds 
			vector<point_XYZIRL> seeds;
			extractInitialSeedPoints(sortedPointCloudOnZ, seeds, numLPR, seedThresh, method);
			
			// Estimate plane 
			if (seeds.size()) {
			 	for (int iter = 0; iter < numIters; iter++) {	
					//	The linear model to solve is: ax + by +cz + d = 0
			        //   		where; N = [a b c]     X = [x y z], 
					//		           d = -(N.transpose * X)
					MatrixXf xyzM; 
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
			} else cout << "No seeds extracted." << endl;
		}
		clock_t computeTime = clock();
		cout << "  >> File[" << i + 1 << "/" << files.size() << "] - "
             << "Ground points found: " << count << " / " << labeledPointCloud.size() << "."
             << "Time: " << (computeTime - startTime) / double(CLOCKS_PER_SEC) << "s" << endl;
		sortPointCloud(labeledPointCloud, filteredPoints, false, "n"); // Sort based on the ring
		string filepath = newDir + "/" + filename;
		saveToFile(labeledPointCloud, filepath); 
	}
	clock_t finishTime = clock();
	cout << endl;
	cout << "[ DONE ]" << endl
		 << "  >> Total execution time: " 
		 << (finishTime - startTime) / double(CLOCKS_PER_SEC) << "s" << endl; 
	return 0;
}

// Get filenames of point clouds. 
int getFiles(string path, vector<string>& files) {
	// Validate directory 
	if ((dir = opendir(path.c_str())) == NULL) {
		cout << "Error (" << errno << "): could not open " << dir << endl;
		return errno;
	}
	
	// Get files
	char *p1;
	char *p2;
	while((ent = readdir(dir)) != NULL) {
		string file = ent->d_name;
		p1 = strtok(ent->d_name, ".");
		p2 = strtok(NULL, ".");
		if (p2 != NULL) {
			if (strcmp(p2, "txt") == 0) {
				files.push_back(string(file));
			}
		}
	}
	closedir(dir);
	return 0;
}

// Saves final point cloud to text file 
void saveToFile(const vector<point_XYZIRL>& pointCloud, string filepath) {
	int version = 0;
	ofstream textfile;
	 	
	textfile.open(filepath.c_str(), std::fstream::app);
	for (int i = 0; i < pointCloud.size(); i++) {
		textfile << pointCloud[i].x << " " 
				 << pointCloud[i].y << " " 
	   		     << pointCloud[i].z << " " 
				 << pointCloud[i].i << " " 
	  		     << pointCloud[i].r << " " 
				 << pointCloud[i].l << endl;
	}
	textfile.close();
}


