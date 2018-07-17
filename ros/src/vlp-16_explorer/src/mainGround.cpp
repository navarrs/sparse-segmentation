// ***********************************************************************
/*!
 *  \file    vlp-16_explorer_node.cpp
 *  \brief   Implementation of hooks to Velodyne data for mover detection, 
 *           and estimation and removal of ground elevation. Simple cannonical
 *           code for exploratory purposes...
 *  \date    June 18, 2018
 *  \author  Luis E. Navarro-Serment
 *
 *  This node reads Velodyne data; converts it to a local frame; 
 *
 *  Ver. 1.0
 */
// ***********************************************************************

#include <csignal>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <nav_msgs/GridCells.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <boost/lexical_cast.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/point_cloud_conversion.h>

tf::TransformListener *tfListener;

// Parameter storage
std::string velodyneTopic;
std::string moverSensorFrame;
std::string moverSensorMarkerFrame;

pthread_mutex_t  lidarMutex;
sensor_msgs::PointCloud2::ConstPtr lastMsg;
bool newData(false), isBusy(false);

// -----------------------------------------------------------
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {

  if (msg != NULL ) {   
	  
	pthread_mutex_lock( &lidarMutex );  
    if( !isBusy ) {
      lastMsg = msg;
      newData = true;
      
    }
    pthread_mutex_unlock( &lidarMutex );  
  }
}

// ----------------------------------------------------------------
int main(int argc, char **argv) {
  std::cout << "\e[31;1m[INFO] VLP-16 Explorer Node is about to be instantiated\n \e[0m";
  
  ros::init(argc, argv, "vlp16_explorer_node"); 
  ros::Time::init();        
  std::cout << "\e[31;1m[INFO] VLP-16 Explorer Node has been initialized\n \e[0m";
  ros::NodeHandle n;


  ros::Subscriber forwardVelodyne;
  pthread_mutex_init( &lidarMutex, NULL );

  tfListener = new tf::TransformListener;
  ros::Rate rate(50.0);

  std::cout << "\e[32;1m[INFO] vlp-16_velodyne_explorer Node: TransformListener is up\n \e[0m";
  
  if( n.hasParam("/vlp16_explorer/vlp16_topic") )
	n.getParam("/vlp16_explorer/vlp16_topic", velodyneTopic);
  else {
	std::cout << "[ERROR: /vlp16_explorer/vlp-16_topic parameter is not defined]\n";
	return -1;
  }
  
  forwardVelodyne = n.subscribe(velodyneTopic, 1, lidarCallback );
  
  std::cout << "\e[34;1m[INFO] VLP16 Topic is " << velodyneTopic << "\n \e[0m";
  int counter = 0;
  while( ros::ok() ) {
	   
    // Check if there is new data to process
    pthread_mutex_lock( &lidarMutex );  
    bool processNewData = newData;
    pthread_mutex_unlock( &lidarMutex );      
    
    if( processNewData ) {
      std::ofstream textfile;
      std::string filename; 
      if (counter < 10) filename =  "VLP16/00000" + boost::lexical_cast<std::string>(counter) + ".txt";
      else if (counter < 100) filename =  "VLP16/0000" + boost::lexical_cast<std::string>(counter) + ".txt";
      else filename = "VLP16/000" + boost::lexical_cast<std::string>(counter) + ".txt";

  		textfile.open(filename.c_str(), std::fstream::app);
      counter++;


  		pthread_mutex_lock( &lidarMutex );  
  		isBusy = true;
  		pthread_mutex_unlock( &lidarMutex );  
  		
  		ROS_INFO("[VLP-16 Sensor:: processing new data]");
  		pcl::PointCloud<pcl::PointXYZ>::Ptr tempVelodyneRaw(new pcl::PointCloud<pcl::PointXYZ>);
  		pcl::PCLPointCloud2                 pclVelodyneRaw;
        	pcl_conversions::toPCL( *lastMsg, pclVelodyneRaw );
        	pcl::fromPCLPointCloud2( pclVelodyneRaw, *tempVelodyneRaw );
        	
          for(unsigned int i=0; i < tempVelodyneRaw->size(); i++) { 
        	    float rawX = tempVelodyneRaw->points[i].x;
        	    float rawY = tempVelodyneRaw->points[i].y;
        	    float rawZ = tempVelodyneRaw->points[i].z;
              textfile << rawX << " "
                       << rawY << " " 
                       << rawZ << " "
                       << 0 << std::endl;
  		}
  		std::cout << "Processed " << tempVelodyneRaw->size() << " points.\n";
  		std::cout << "counter: " << counter << std::endl;
      textfile.close();

  		pthread_mutex_lock( &lidarMutex );  
  		isBusy  = false;
  		newData = false;
  		pthread_mutex_unlock( &lidarMutex );  
	} else {
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }
	
  }
  
  // ros::spin();
                                  
  return 0;
}



