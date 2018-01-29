#ifndef HANDA_TO_ROSBAG_CONVERSIONS_H_
#define HANDA_TO_ROSBAG_CONVERSIONS_H_

#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tfMessage.h>

#include <opencv2/highgui/highgui.hpp>

#include "handa_to_rosbag/common.h"

namespace handa_to_rosbag {

// Image types to ROS messages
void imageToRos(const cv::Mat& image, sensor_msgs::Image* image_msg);
void depthToRos(const cv::Mat& depth, sensor_msgs::Image* depth_msg);

// Transform conversions.
void transformToTf(const Transformation& transform,
                   tf::Transform* tf_transform);
void transformToRos(const Transformation& transform,
                    geometry_msgs::TransformStamped* transform_msg);

// Color conversion
//inline float bgrToMono(const uint8_t b, const uint8_t g, const uint8_t r);
inline float bgrToMono(const uint8_t b, const uint8_t g, const uint8_t r) {
  // Using the mono conversion formula here:
  // https://docs.opencv.org/3.1.0/de/d25/imgproc_color_conversions.html
  // and scaling from 0.0 to 1.0
  return (0.299 * static_cast<float>(r) + 0.587 * static_cast<float>(g) +
          0.114 * static_cast<float>(b)) / 254.0;
}

}  // namespace handa_to_rosbag

#endif  // HANDA_TO_ROSBAG_CONVERSIONS_H_
