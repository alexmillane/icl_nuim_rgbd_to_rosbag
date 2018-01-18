#include "handa_to_rosbag/conversions.h"

#include <cv_bridge/cv_bridge.h>

#include <glog/logging.h>

namespace handa_to_rosbag {

void imageToRos(const cv::Mat& image, sensor_msgs::Image* image_msg) {
  // Creating the bridge for convertion
  cv_bridge::CvImage image_cv_bridge;
  // Copying in the image data
  image_cv_bridge.image = image;
  // Encodings
  if (image.type() == CV_8U) {
    image_cv_bridge.encoding = "mono8";
  } else if (image.type() == CV_8UC3) {
    image_cv_bridge.encoding = "bgr8";
  } else {
    CHECK(false) << "unsupported image type";
  }
  // Creating the message
  image_cv_bridge.toImageMsg(*image_msg);
}

void depthToRos(const cv::Mat& depth, sensor_msgs::Image* depth_msg){
  // Creating the bridge for convertion
  cv_bridge::CvImage image_cv_bridge;
  // Copying in the image data
  image_cv_bridge.image = depth;
  // Encoding must be mono 32bit float according to
  // http://www.ros.org/reps/rep-0118.html
  CHECK_EQ(depth.type(), CV_32FC1) << "depth image must be mono 32bit float";
  image_cv_bridge.encoding = "32FC1";
  // Creating the message
  image_cv_bridge.toImageMsg(*depth_msg);
}

}  // namespace handa_to_rosbag