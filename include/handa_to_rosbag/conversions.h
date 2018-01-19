#ifndef HANDA_TO_ROSBAG_CONVERSIONS_H_
#define HANDA_TO_ROSBAG_CONVERSIONS_H_

#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>

/*#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#include "kitti_to_rosbag/kitti_common.h"
*/
namespace handa_to_rosbag {

// Image types to ROS messages
void imageToRos(const cv::Mat& image, sensor_msgs::Image* image_msg);
void depthToRos(const cv::Mat& depth, sensor_msgs::Image* depth_msg);

// Color conversion
//inline float bgrToMono(const uint8_t b, const uint8_t g, const uint8_t r);
inline float bgrToMono(const uint8_t b, const uint8_t g, const uint8_t r) {
  // Using the mono conversion formula here:
  // https://docs.opencv.org/3.1.0/de/d25/imgproc_color_conversions.html
  // and scaling from 0.0 to 1.0
  return (0.299 * static_cast<float>(r) + 0.587 * static_cast<float>(g) +
          0.114 * static_cast<float>(b)) / 254.0;
}

/*void calibrationToRos(uint64_t cam_id, const CameraCalibration& cam,
                      sensor_msgs::CameraInfo* cam_msg);
void stereoCalibrationToRos(const CameraCalibration& left_cam,
                            const CameraCalibration& right_cam,
                            sensor_msgs::CameraInfo* left_cam_msg,
                            sensor_msgs::CameraInfo* right_cam_msg);
*/

/*void poseToRos(const Transformation& transform,
               geometry_msgs::PoseStamped* pose_msg);
void transformToTf(const Transformation& transform,
                   tf::Transform* tf_transform);
void transformToRos(const Transformation& transform,
                    geometry_msgs::TransformStamped* transform_msg);
void timestampToRos(uint64_t timestamp_ns, ros::Time* time);

std::string getCameraFrameId(int cam_id);
*/
}  // namespace handa_to_rosbag

#endif  // HANDA_TO_ROSBAG_CONVERSIONS_H_
