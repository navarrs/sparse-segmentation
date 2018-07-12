/*	
	Sun 01 July 2018 19:08:52 PM EDT 
	Method for Ground Filtration from 3D point clouds. 
	Ingrid Navarro 

	Based on:
	    - A fast segmentation method for sparse point clouds (Mengjie Li)
	    - Fast segmentation of 3D point clouds: a paradigm on LIDAR data (Dimitris Zermas)
	    - Real-time ground filtration methid for loader crane monitoring system (Karol Miadlicki)

	Example commands:	
	Used with VLP64:
    	./groundExtractor --path samples/KITTI/2011_09_26/velodyne_points/data/  --file 0000000000  --num_lpr 50  --num_iter 3 --th_seed 1.2  --th_dist 0.3
    Used with VLP16:
    	./groundExtractor --path samples/VLP16  --file 000000  --num_lpr 20  --num_iter 3  --th_seed 0.4  --th_dist 0.2
*/
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#include <stdio.h>
#include <sys/stat.h>
#include <time.h>

#include "includes.h"
#include "groundExtractor.h"
#include "pointCloud.h"

namespace po = boost::program_options;
using Eigen::JacobiSVD;
using Eigen::VectorXf;
using Eigen::MatrixXf;
using namespace std;

// Methods
struct stat sb;
void saveToFile(const vector<point_XYZIRL>& pointCloud, string dirName, string name, bool type);
static void usage(string progName);

int main(int argc, char* argv[]) {
	
	if (argc < 2) { usage(argv[0]); return 1; } // If no argument provided
	
	// Default parameters (if not provided by the user
	string pathToFile = "../data/KITTI/2011_09_26/velodyne_points/data/0000000000.txt";
	string dataset    = "squeeze"; 
	int numLPR        = 20;
	int numIterations = 3;
	int numSegments   = 1;
	float seedThresh  = 1.2; // 0.4
	float distThresh  = 0.3; // 0.2
	float avgLPR      = 0.0;

	// Parse arguments
	po::options_description descritption("Ground Extration Usage");
	descritption.add_options()
		("help",                           "Program usage.")
		("path",      po::value<string>(), "VLP file path")
		("dataset",   po::value<string>(), "Specify source dataset")
		("num_lpr",   po::value<int>(),    "Number of seeds needed to get the LPR")
		("num_iter",  po::value<int>(),    "Number of times needed to estimate the ground plane")
		("num_seg",   po::value<int>(),    "Number of point cloud segments")
		("th_seed",   po::value<float>(),  "Seeds threshold value")
		("th_dist",   po::value<float>(),  "Distance threshold value");
	
	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(descritption).run(), vm);
	po::notify(vm);
	
	if (vm.count("help"))      { usage(argv[0]); return 0; }
	if (vm.count("path"))      pathToFile    = vm["path"].as<string>();
	if (vm.count("dataset"))   dataset       = vm["dataset"].as<string>();
	if (vm.count("num_lpr"))   numLPR        = vm["num_lpr"].as<int>();
	if (vm.count("num_iter"))  numIterations = vm["num_iter"].as<int>();
	if (vm.count("num_seg"))   numSegments   = vm["num_seg"].as<int>();
	if (vm.count("th_seed"))   seedThresh    = vm["th_seed"].as<float>();
	if (vm.count("th_dist"))   distThresh    = vm["th_dist"].as<float>();
	
	size_t index = pathToFile.find_last_of("/");
	string filename = pathToFile.substr(index+1, pathToFile.length());
	index = filename.find_last_of("."); 
	filename  = filename.substr(0, index); 

	// --------------------------------------------------------------------------------------------
	// Start algorithm
	cout << " --------------------------------------- " << endl	
    	 << "|      Ground Extraction Algorithm      |" << endl
    	 << " --------------------------------------- " << endl
	     << " ------------ CONFIGURATION " << endl
	     << " --- Reading point cloud from: " << pathToFile << endl
	     << " --- Num of point cloud segments: " << numSegments << endl
	     << " --- Num of iterations: " << numIterations << endl
	     << " --- Num to calculate LPR: " << numLPR << endl
	     << " --- Seeds threshold: " << seedThresh << endl
	     << " --- Distance threshold: " << distThresh << endl << endl
	     << " ---------------- START " << endl;
	
	// Read point cloud 
	vector<point_XYZIRL> sortedPointCloudOnHeight;
	vector<point_XYZIRL> unsortedPointCloud;
	vector<point_XYZIRL> groundPointCloud;
	vector<point_XYZIRL> noGroundPointCloud;
	
	clock_t startTime = clock(); 

	// TODO: read point cloud in format [x y z i r l]
	// TODO: upload sample files to github
	// For now, it is assumed that the point cloud is in a textfile
	// with the following format: [x y z i]
	if (!getPointCloud(pathToFile, unsortedPointCloud)) {
		cout << endl << "ERROR: could not locate file." << endl;
		return 0;
	}
	copyPointCloud(unsortedPointCloud, sortedPointCloudOnHeight);

	// Sort point cloud based on provided height axis. 
	// Last argument is used to filter noise, in current tests I use 
	// 'false' for VLP16 and 'true' for VLP64
	sortPointCloud(sortedPointCloudOnHeight, true);  
	// printPointCloud(sortedPointCloudOnHeight, 10); // uncomment to debug
	
	// Extract initial seeds based on the seed threshold and an average
	// of the lowest point that represents the ground. 
	extractInitialSeedPoints(sortedPointCloudOnHeight, groundPointCloud, numLPR, seedThresh);
	
	// Start ground estimation 
	if (groundPointCloud.size()) {
	 	for (int iter = 0; iter < numIterations; iter++) {
 			cout << endl 
	 		     << "\tEstimate ground plane ITERATION [" << iter + 1 <<" / " << numIterations << "]" << endl
	 		     << "\t\t--- Current seed points: " << groundPointCloud.size() << endl;
			
			// TODO: finish from here 
			//	The linear model to solve is:
	        //   		ax + by +cz + d = 0
	        //   		N = [a b c]
	        //   		X = [x y z]
			//		    N.transpose * X = -d 
	 		MatrixXf xyzMeans  = getSeedMeans(groundPointCloud);
	     	MatrixXf normal = estimatePlaneNormal(groundPointCloud, xyzMeans);
		    float negDist = -(normal.transpose() * xyzMeans)(0, 0); // This is equivalent to equation n.T * X = -d 
		    float currDistThresh = distThresh - negDist;  // Get maximum distance representative for this plane model
	   		cout << "\t\t--- Current distance threshold: " << currDistThresh << endl;
	           			
	        groundPointCloud.clear();  // Clear points before readjusting with new plane model 
	        noGroundPointCloud.clear();

	        cout << "\tUpdate ground and non-ground points..."; // New ground points will be used as seeds	
			for (int i = 0; i < sortedPointCloudOnHeight.size(); i++) {
	    		MatrixXf point = convertPointToMatXf(sortedPointCloudOnHeight[i]);		
	   			if (getDistance(point, normal) < currDistThresh) { 
	   				groundPointCloud.push_back(sortedPointCloudOnHeight[i]); 	
	   			} else {
	   				noGroundPointCloud.push_back(sortedPointCloudOnHeight[i]);
	   			 }
			}
			cout << "Done." << endl;  
		}

	 	// Store final results
	    string newDir = "results/" + dataset + "/" + filename;  // Directory to create
	 	if (!(stat(newDir.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))) { // If directory doesn't exist, create it
	 		mkdir(newDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); 
	 	}
	 	saveToFile(groundPointCloud, newDir, filename, true);
	 	saveToFile(noGroundPointCloud, newDir, filename, false);
	    
	    clock_t finishTime = clock();
	    cout << " ---------------- DONE. " << endl
	         << "Total execution time: " 
	         << (finishTime - startTime) / double(CLOCKS_PER_SEC) * 1000 << "ms" << endl; 

	} else cout << "No seeds extracted." << endl;
	
	return 0;
}

/* --------------------------------------------------
	Function: Show program's usage
	Parameters:
	   - Program name (string)
	Returns: 
	   - Nothing (void) 
   -------------------------------------------------- */
static void usage(string progName) {
    cout << "Usage: " << progName << " <option(s)>  " << endl
         << "Options:" << endl
         << "\t--help\t\tShow this help message" << endl
         << "\t--num_lpr\t\tIndicate number to estimate lowest point representative (LPR)" << endl
         << "\t--num_iter\t\tIndicate number of iterations to estimate the plane" << endl
         << "\t--num_seg\t\tIndicate number of segmentations along the x-axis" << endl
         << "\t--th_seeds\t\tIndicate threshold value to compute seeds" << endl
         << "\t--th_dist\t\tIndicate threshold value to compute plane distance" << endl
         << "\t--path \t\tSpecify the path of the point cloud." << endl
         << "\t--file \t\tSpecify filename"
         << " ./groundExtractor --path ../samples/KITTI/2011_09_26/velodyne_points/data/ --file 0000000000 --num_lpr 5000 --num_iter 5 --height_ax z --th_seed 0.5 --th_dist 0.6 --num_seg 20 " << endl
         << endl;	
}

/* --------------------------------------------------
	Function: Save point cloud to file 
	Parameters:
	   - Point from point cloud (point_XYZIRL)
	   - directory (string) & name (string)
	   - type of pointcloud (ground or not ground)
	Returns: 
	   - Vectorized point (MatrixXf)
   -------------------------------------------------- */
void saveToFile(const std::vector<point_XYZIRL>& pointCloud, std::string dirName, std::string name, bool type) {
	int version = 0;
	std::ofstream textfile;
	std::string filename;
	do {
		if (type) {
			filename = dirName + "/" + boost::lexical_cast<std::string>(++version) + "_gnd_" + name + ".txt";
		} else {
			filename = dirName + "/" + boost::lexical_cast<std::string>(++version) + "_ngnd_" + name + ".txt";
		}
	} while (stat(filename.c_str(), &sb) == 0); // create new version if file exists
	 	
	textfile.open(filename.c_str(), std::fstream::app);
	for (int i = 0; i < pointCloud.size(); i++) {
		textfile << pointCloud[i].x << " "
	   		     << pointCloud[i].y << " " 
	   		     << pointCloud[i].z << " "
	  		     << pointCloud[i].i << " " 
	  		     << pointCloud[i].r << " "
	  		     << pointCloud[i].l << std::endl;
	}
	textfile.close();
}
