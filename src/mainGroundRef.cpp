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

#include "groundExtractor.h"
#include "includes.h"
#include "pointCloud.h"

#define GROUND 4

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
static void usage(string progName);

int main(int argc, char* argv[]) {
	
	if (argc < 2) { usage(argv[0]); return 1; } // If no argument provided
	
	// Default parameters (if not provided by the user)
	string inputPath  = "data/ground/lidar_ng/";
	string outputPath = "data/ground/";
	int numLPR        = 20;
	int numSegments   = 1;
	int numIterations = 3;
	float seedThresh  = 1.2; // 0.4
	float distThresh  = 0.3; // 0.2
	float avgLPR      = 0.0;

	// Parse arguments
	po::options_description descritption("Ground Extration Usage");
	descritption.add_options()
		("help",                           "Program usage.")
		("in_path",   po::value<string>(), "Input pointclouds path")
		("out_path",  po::value<string>(),  "Output pointclouds file")
		("num_lpr",   po::value<int>(),    "Number of seeds needed to get the LPR")
		("num_seg",   po::value<int>(),    "Number of segments along the x-axis")
		("num_iter",  po::value<int>(),    "Number of times needed to estimate the ground plane")
		("th_seed",   po::value<float>(),  "Seeds threshold value")
		("th_dist",   po::value<float>(),  "Distance threshold value");
	
	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(descritption).run(), vm);
	po::notify(vm);
	
	if (vm.count("help"))      { usage(argv[0]); return 0; }
	if (vm.count("in_path"))   inputPath     = vm["in_path"].as<string>();
	if (vm.count("out_path"))  outputPath    = vm["out_path"].as<string>();
	if (vm.count("num_lpr"))   numLPR        = vm["num_lpr"].as<int>();
	if (vm.count("num_seg"))   numSegments   = vm["num_seg"].as<int>();
	if (vm.count("num_iter"))  numIterations = vm["num_iter"].as<int>();
	if (vm.count("th_seed"))   seedThresh    = vm["th_seed"].as<float>();
	if (vm.count("th_dist"))   distThresh    = vm["th_dist"].as<float>();

	// --------------------------------------------------------------------------------------------
	// Start algorithm
	cout << " --------------------------------------- " << endl	
    	 << "|      Ground Extraction Algorithm      |" << endl
    	 << " --------------------------------------- " << endl
	     << " ------------ CONFIGURATION " << endl
	     << " --- Reading point cloud files from: " << inputPath << endl
	     << " --- Saving annotated files in: " << outputPath << endl
	     << " --- Num of iterations: " << numIterations << endl
	     << " --- Num of segments along the x-axis: " << numSegments << endl
	     << " --- Num to calculate LPR: " << numLPR << endl
	     << " --- Seeds threshold: " << seedThresh << endl
	     << " --- Distance threshold: " << distThresh << endl << endl
	     << " ---------------- START " << endl;
	
	// Get all files to process from specified directory 
	vector<string> files; 
	getFiles(inputPath, files);
	
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

		vector<point_XYZIRL> unsortedPointCloud;
		vector<point_XYZIRL> sortedPointCloudOnX;
		vector<point_XYZIRL> labeledPointCloud;
		vector<point_XYZIRL> filteredPoints;
		labeledPointCloud.clear();
		
		string filename = files[i];
		string tempPath = inputPath + filename;

	    // Read point cloud 
	 	if (!getPointCloud(tempPath, unsortedPointCloud)) {
	 		cout << endl << "ERROR: could not locate file." << endl;
	 		return 0;
	 	}

		// Sort point cloud based on x axis. Second argument is used to filter 
		// noisy values (this was done empirically :D ) 
		// TODO: deal with filtering and chunks
		copyPointCloud(unsortedPointCloud, sortedPointCloudOnX);
		sortPointCloud(sortedPointCloudOnX, filteredPoints, false, "x");

		// Create subpointclouds
		size_t chunk = ceil((double)sortedPointCloudOnX.size() / numSegments);
		vector<point_XYZIRL>::iterator start = sortedPointCloudOnX.begin();
		int pcl = 0;
		int count = 0;
		for(size_t it = 0; it < sortedPointCloudOnX.size(); it += chunk) {
    		
    		vector<point_XYZIRL> sortedPointCloudOnZ(start+it, start+std::min<size_t>(it+chunk, sortedPointCloudOnX.size()));
    		filteredPoints.clear();
    		sortPointCloud(sortedPointCloudOnZ, filteredPoints, true, "z");

    		// Extract initial seeds based on the seed threshold and an average of the
			// lowest point that represents the ground. 
			vector<point_XYZIRL> seeds;
			extractInitialSeedPoints(sortedPointCloudOnZ, seeds, numLPR, seedThresh);
			// printPointCloud(seeds, 10);
			// Start plane estimation 
			if (seeds.size()) {
			 	for (int iter = 0; iter < numIterations; iter++) {
		    		// cout << endl 
			 		//      << "\tEstimate ground plane ITERATION [" << iter + 1 <<" / " << numIterations << "]" << endl
			 		//      << "\t\t--- Current seed points: " << seeds.size() << endl;
					
					//	The linear model to solve is: ax + by +cz + d = 0
			        //   		where; N = [a b c]     X = [x y z], 
					//		           d = -(N.transpose * X)
			 		MatrixXf xyzMeans  = getSeedMeans(seeds);
			     	MatrixXf normal = estimatePlaneNormal(seeds, xyzMeans);
				    float negDist = -(normal.transpose() * xyzMeans)(0, 0); // d = -(n.T * X)
				    float currDistThresh = distThresh - negDist;            // Max ground distance of current model
			   		// cout << "\t\t--- Current distance threshold: " << currDistThresh << endl;
			        
			        // Calculate the distance for each point and compare it with current threshold to 
			        // determine if it is a ground point or not. 
			        seeds.clear();           			
			        // cout << "\tLabel ground points..."; // New ground points will be used as seeds
			        
			        // TODO: fix this mess 
			        if (iter < numIterations-1) {  // We are still estimating planes so we filter the noise
				        for (int i = 0; i < sortedPointCloudOnZ.size(); i++) {
				    		MatrixXf point = convertPointToMatXf(sortedPointCloudOnZ[i]);		
				   			if (getDistance(point, normal) < currDistThresh) {
				   				seeds.push_back(sortedPointCloudOnZ[i]);
				   			}
						}
			        } else { // Here we will consider the noise as a point ground 
			        	for (int i = 0; i < sortedPointCloudOnZ.size(); i++) {
				    		MatrixXf point = convertPointToMatXf(sortedPointCloudOnZ[i]);		
				   			if (getDistance(point, normal) < currDistThresh && sortedPointCloudOnZ[i].l == 0) {
				   				sortedPointCloudOnZ[i].l = GROUND; // Ground class  	
				   				count++;
				   			}
				   			labeledPointCloud.push_back(sortedPointCloudOnZ[i]);
						}
			        }
					// cout << "Done." << endl;  
				}
			} else cout << "No seeds extracted." << endl;
		}
		cout << "Processed files: " << i + 1 << "/" << files.size() 
		     << " with " << count << " ground points. "<< endl;
		for (int f = 0; f < filteredPoints.size(); f++) {
					labeledPointCloud.push_back(filteredPoints[f]);
		}
		saveToFile(labeledPointCloud, newDir, filename, true); 
	}
	clock_t finishTime = clock();
	cout << " ---------------- DONE. " << endl
		 << "Total execution time: " 
         << (finishTime - startTime) / double(CLOCKS_PER_SEC) << "s" << endl; 
	return 0;
}

/* --------------------------------------------------
	Function: Show program's usage
	Parameters:
	   - Program name (string)
   -------------------------------------------------- */
static void usage(string progName) {
    cout << "Usage: " << progName << " <option(s)>  " << endl
         << "Options:" << endl
         << "\t--help\t\tShow this help message" << endl
         << "\t--num_lpr\t\tIndicate number to estimate lowest point representative (LPR)" << endl
         << "\t--num_iter\t\tIndicate number of iterations to estimate the plane" << endl
         << "\t--th_seeds\t\tIndicate threshold value to compute seeds" << endl
         << "\t--th_dist\t\tIndicate threshold value to compute plane distance" << endl
         << "\t--in_path \t\tSpecify the path where the input files are." << endl
         << "\t--out_path\t\tSpecift the path where to save the output files."
         << " ./groundExtractor --path ../samples/KITTI/2011_09_26/velodyne_points/data/" << endl
         << endl;	
}

/* --------------------------------------------------
	Function: Save point cloud to file 
	Parameters:
	   - Point from point cloud (point_XYZIRL)
	   - directory (string) & name (string)
	   - type of pointcloud (ground or not ground)
   -------------------------------------------------- */
void saveToFile(const vector<point_XYZIRL>& pointCloud, string dirName, string name, bool type) {
	int version = 0;
	ofstream textfile;
	string filename = dirName + "/" + name;
	 	
	textfile.open(filename.c_str(), std::fstream::app);
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

/* -----------------------------------------------------
	Function: Get all the files from specified directory
	Parameters:
	   - Input path (string)
	   - Vector to store filenames (vector<string>)
   ----------------------------------------------------- */
int getFiles(string path, vector<string>& files) {

	if ((dir = opendir(path.c_str())) == NULL) {
		cout << "Error(" << errno << ") opening " << dir << endl;
		return errno;
	}
	
	// Get files
	char *p1;
	char *p2;
	while((ent = readdir(dir)) != NULL) {
		string f = ent->d_name;
		p1 = strtok(ent->d_name, ".");
		p2 = strtok(NULL, ".");
		if (p2 != NULL) {
			if (strcmp(p2, "txt") == 0) {
				files.push_back(string(f));
			}
		}
	}
	closedir(dir);
	return 0;
}