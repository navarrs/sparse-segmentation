/*	
	Sun 01 July 2018 19:08:52 PM EDT 
	Method for Ground Filtration from 3D point clouds. 
	Ingrid Navarro 
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

using namespace std;
namespace po = boost::program_options;
using Eigen::JacobiSVD;
using Eigen::VectorXf;
using Eigen::MatrixXf;

DIR * dir;
struct dirent *ent;
struct stat sb;
int getFiles(string path, vector<string>& files);
void saveToFile(const vector<point_XYZIRL>& pointCloud, string dirName, string name, bool type);

int main(int argc, char* argv[]) {
	// Parse arguments
	po::options_description description("Usage");
	description.add_options()
		("help", "Program usage.")
		("inpath",  po::value<string>()->default_value("data/ground/lidar_ng"), "Input pointclouds path")
		("outpath", po::value<string>()->default_value("data/ground/"), "Output pointclouds file")
		("lpr",     po::value<int>()->default_value(20), "Number of seeds needed to get the LPR")
		("seg",     po::value<int>()->default_value(1), "Number of segments along the x-axis")
		("iter",    po::value<int>()->default_value(3), "Number of times needed to estimate the ground plane")
		("thseed",  po::value<float>()->default_value(1.2), "Seeds threshold value")
		("thdist",  po::value<float>()->default_value(0.3), "Distance threshold value");
	po::variables_map opts;
	po::store(po::command_line_parser(argc, argv).options(description).run(), opts);
	try { po::notify(opts);	}
	catch (exception& e) {
		cerr << "Error: " << e.what() << endl;
		return 1;
	}
	if (opts.count("help")) { cout << description; return 1; }
	string inputPath  = opts["inpath"].as<string>();
	string outputPath = opts["outpath"].as<string>();
	int numLPR        = opts["lpr"].as<int>();
	int numSegments   = opts["seg"].as<int>();
	float numIters    = opts["iter"].as<int>();
	float seedThresh  = opts["thseed"].as<float>();
	float distThresh  = opts["thdist"].as<float>();

	// --------------------------------------------------------------------------------------------
	// Start algorithm
	cout << " --------------------------------------- " << endl	
    	 << "|      Ground Extraction Algorithm      |" << endl
    	 << " --------------------------------------- " << endl
	     << " ------------ CONFIGURATION " << endl
	     << " --- Reading point cloud files from: " << inputPath << endl
	     << " --- Saving annotated files in: " << outputPath << endl
	     << " --- Num of iterations: " << numIters << endl
	     << " --- Num of segments along the x-axis: " << numSegments << endl
	     << " --- Num to calculate LPR: " << numLPR << endl
	     << " --- Seeds threshold: " << seedThresh << endl
	     << " --- Distance threshold: " << distThresh << endl << endl
	     << " ---------------- START " << endl; 
	
	// Get all files to be process from the specified directory 
	vector<string> files; 
	if (getFiles(inputPath, files)) return 1;
	
	// Create output directory
	int version = 0;
	string newDir;
	do { 
		newDir = outputPath + boost::lexical_cast<std::string>(++version);
	} while (stat(newDir.c_str(), &sb) == 0); // create new version if file exists
	cout << "Creating directory " << newDir << endl;
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
		int pcl = 0;
		int count = 0;
		for(size_t it = 0; it < pointCloud.size(); it += chunk) {
    		
    		// Create subpointcloud
    		vector<point_XYZIRL> sortedPointCloudOnZ(start+it, start+std::min<size_t>(it+chunk, pointCloud.size()));
    		filteredPoints.clear();
    		sortPointCloud(sortedPointCloudOnZ, filteredPoints, true, "z");

    		// Extract initial seeds 
			vector<point_XYZIRL> seeds;
			extractInitialSeedPoints(sortedPointCloudOnZ, seeds, numLPR, seedThresh);
			
			// Start plane estimation 
			if (seeds.size()) {
			 	for (int iter = 0; iter < numIters; iter++) {					
					//	The linear model to solve is: ax + by +cz + d = 0
			        //   		where; N = [a b c]     X = [x y z], 
					//		           d = -(N.transpose * X)
			 		MatrixXf xyzMeans  = getSeedMeans(seeds);
			     	MatrixXf normal = estimatePlaneNormal(seeds, xyzMeans);
				    float negDist = -(normal.transpose() * xyzMeans)(0, 0); // d = -(n.T * X)
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
		cout << "Processed files: " << i + 1 << "/" << files.size() << " with " << count << " of " << labeledPointCloud.size() <<  " ground points" << endl;
		sortPointCloud(labeledPointCloud, filteredPoints, false, "n"); // Sort based on the ring
		saveToFile(labeledPointCloud, newDir, filename, true); 
	}
	clock_t finishTime = clock();
	cout << " ---------------- DONE. " << endl
		 << "Total execution time: " 
		 << (finishTime - startTime) / double(CLOCKS_PER_SEC) << "s" << endl; 
	return 0;
}

/* -------------------------------------------------------------------------------------------------------------
	Function: Save point cloud to file 
	Parameters:
	   - Point from point cloud (point_XYZIRL)
	   - directory (string) & name (string)
	   - type of pointcloud (ground or not ground)
   ------------------------------------------------------------------------------------------------------------- */
void saveToFile(const vector<point_XYZIRL>& pointCloud, string dirName, string name, bool type) {
	int version = 0;
	ofstream textfile;
	string filename = dirName + "/" + name;
	 	
	textfile.open(filename.c_str(), std::fstream::app);
	for (int i = 0; i < pointCloud.size(); i++) {
		textfile << pointCloud[i].x << " " << pointCloud[i].y << " " 
	   		     << pointCloud[i].z << " " << pointCloud[i].i << " " 
	  		     << pointCloud[i].r << " " << pointCloud[i].l << endl;
	}
	textfile.close();
}

/* -------------------------------------------------------------------------------------------------------------
	Function: Get all the files from specified directory
	Parameters:
	   - Input path (string)
	   - Vector to store filenames (vector<string>)
   ------------------------------------------------------------------------------------------------------------- */
int getFiles(string path, vector<string>& files) {
	if ((dir = opendir(path.c_str())) == NULL) {
		cout << "Error(" << errno << ") opening " << dir << endl;
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