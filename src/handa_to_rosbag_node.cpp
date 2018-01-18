#include <glog/logging.h>
#include <ros/ros.h>

#include "handa_to_rosbag/handa_to_rosbag.h"

// Standard C++ entry point
int main(int argc, char** argv) {
  // Starting the logging
  google::InitGoogleLogging(argv[0]);
  // Announce this program to the ROS master
  ros::init(argc, argv, "handa_to_rosbag");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Creating the object to do the work
  handa_to_rosbag::HandaToRosbag handa_to_rosbag(nh, nh_private);

  // Running the converter
  handa_to_rosbag.run();

  // Spinning
  ros::spin();
  // Exit tranquilly
  return 0;
}
