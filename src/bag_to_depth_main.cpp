/*!
*****************************************************************
*
* \note
* Project name: bag to depth
* \note
* ROS package name: bag_to_depth
*
* \author
* Author: Jaouad Hajjami - jaouadhajj@gmail.com
*
* \date Date of creation: 25.06.2016
*
* \brief
* Get true depth information from bagfile
*****************************************************************/

// ROS
#include <ros/package.h>

// OpenCv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

// Bag files
#include <rosbag/bag.h>

//bag_depth header
#include "bag_to_depth/bag_depth.h"

int main( int argc, char** argv )
{
  // Initialize node
  //ros::init(argc, argv, "rosbag_file_reader");
  //ros::NodeHandle nh;

  // load file rosbag file
  std::string bag_in_name;
  bag_in_name = ros::package::getPath("bag_to_depth") + "/bagfile/" + argv[1] + ".bag";

  BagToDepth bag_to_depth(bag_in_name);


  bag_to_depth.msgFrameToMatTrueDepth();
  cv::Mat depthclean=bag_to_depth.getMatTrueDepth();
  cv::Mat depthvisualization=bag_to_depth.getMatDepthVisualization();

  //depthclean: Process depth images with true depth
  //Access depth pixel: depthclean.at<float>(y,x);

  cv::imshow("depth_Visualization", depthvisualization);
  cv::waitKey(0);

  return 0;
}





















//
