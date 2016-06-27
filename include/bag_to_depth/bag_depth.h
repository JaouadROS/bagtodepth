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

// OpenCv
#include "cv.h"

// Bag files
#include <rosbag/bag.h>

class BagToDepth
{
  std::string bag_name;
  rosbag::Bag bag;
  cv::Mat depth_clean;
  cv::Mat depth_visualization;
  cv::Mat rgb_image;

public:
  BagToDepth(std::string bag_name);
  void msgFrameToMatTrueDepth();
  //void msgFrameToMatRgb();
  cv::Mat getMatTrueDepth();
  cv::Mat getMatDepthVisualization();
};
