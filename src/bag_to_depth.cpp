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
#include <ros/ros.h>

// OpenCv
#include "cv.h"
#include "highgui.h"

// For transforming ROS/OpenCV images
#include <cv_bridge/cv_bridge.h>

// Bag files
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Messages
#include "sensor_msgs/Image.h"

// Boost
#include <boost/foreach.hpp>

//bag_depth header
#include "bag_to_depth/bag_depth.h"

BagToDepth::BagToDepth(std::string bag_name)
{
  // Check input
  if (bag_name == "") {
    ROS_ERROR("No input bag file defined!");
  } else {
    // Load input bag file
    bag.open(bag_name, rosbag::bagmode::Read);
    ROS_INFO("Opened %s", bag_name.c_str());
  }
}

void BagToDepth::msgFrameToMatTrueDepth()
{
  // Get topics
  rosbag::View view(bag);
  unsigned int  bagsize=view.size(),
  i_rosmsg=0,
  i_count=1;

  std::cout<<"bagsize: "<<bagsize<<std::endl;

  std::cout<<"Ros message / Frame number: "<<std::endl;
  std::cin>>i_rosmsg;

  while(i_rosmsg>bagsize) {
    ROS_ERROR("The size of the rosbag file is %d", bagsize);
    std::cin>>i_rosmsg;
  }

  ROS_INFO("frame%05i.png ", i_rosmsg);

  // Loop over messages
  sensor_msgs::ImageConstPtr image_msg;
  BOOST_FOREACH(rosbag::MessageInstance const m, view) {
    // Get specific msg (i_rosmsg) from bagfile messages
    image_msg = m.instantiate<sensor_msgs::Image>();
    if (i_rosmsg==i_count)
    break;
    i_count++;
  }

  //cv_bridge::CvImagePtr depth_image_ptr;
  if (image_msg->encoding == "32FC1") { // depth image
    cv_bridge::CvImagePtr depth_image_ptr;
    try {
      depth_image_ptr = cv_bridge::toCvCopy(image_msg);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    depth_clean= cv::Mat(depth_image_ptr->image.rows, depth_image_ptr->image.cols, CV_32FC1);//True depth
    depth_visualization= cv::Mat(depth_image_ptr->image.rows, depth_image_ptr->image.cols, CV_8UC1);//For visualization
    for(size_t i = 0; i < depth_image_ptr->image.rows; i++)
    {
      float* Di = depth_image_ptr->image.ptr<float>(i);
      float* Ii = depth_clean.ptr<float>(i);

      char* Ivi = depth_visualization.ptr<char>(i);
      for(size_t j = 0; j < depth_image_ptr->image.cols; j++)
      {
        if(Di[j] > 0.0f)
        {
          Ii[j] = Di[j];
          Ivi[j] = (char) (255*((Di[j])/(5.5))); // some suitable values.. => For visualization
        }
        else
        {
          Ii[j] = 0.0f;
          Ivi[j] = 0;
        }
      }
    }
  }
}

/*
void BagToDepth::msgFrameToMatTrueDepth()
{
  // Get topics
  rosbag::View view(bag);
  unsigned int bagsize=view.size(),
  i_rosmsg=0,
  i_count=1;

  std::cout<<"bagsize: "<<bagsize<<std::endl;

  std::cout<<"Frame number: "<<std::endl;
  std::cin>>i_rosmsg;

  while(i_rosmsg>bagsize) {
    ROS_INFO("The size of the rosbag file is %d", bagsize);
    std::cin>>i_rosmsg;
  }

  ROS_INFO("frame%05i.png ", i_rosmsg);

  sensor_msgs::ImageConstPtr image_msg;

  // Loop over messages
  //sensor_msgs::ImageConstPtr image_msg;
  BOOST_FOREACH(rosbag::MessageInstance const m, view) {
    // Get specific msg (i_rosmsg) from bagfile messages
    image_msg = m.instantiate<sensor_msgs::Image>();
    if (i_rosmsg==i_count)
    break;
    i_count++;
  }

  if (image_msg->encoding == "rgb8" || image_msg->encoding == "bgr8" ) { // color image
    // Convert color OpenCV image
    cv_bridge::CvImageConstPtr color_image_ptr;
    try {
      color_image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
}
*/

cv::Mat BagToDepth::getMatTrueDepth()
{
  return depth_clean;
}

cv::Mat BagToDepth::getMatDepthVisualization()
{
  return depth_visualization;
}
