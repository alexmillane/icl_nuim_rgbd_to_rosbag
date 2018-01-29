#ifndef HANDA_TO_ROSBAG_HANDA_TO_ROSBAG_H_
#define HANDA_TO_ROSBAG_HANDA_TO_ROSBAG_H_

#include <string>
//#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <opencv2/highgui/highgui.hpp>

#include <pcl_ros/point_cloud.h>

#include "handa_to_rosbag/common.h"
#include "handa_to_rosbag/conversions.h"

/*#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>
#include <kindr/minimal/quat-transformation.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <tf/transform_broadcaster.h>
*/
namespace handa_to_rosbag {

// Default values for parameters
/*static const bool kDefaultVerbose = true;
static const std::string kDefaultLocalFrameName = "body";
static const std::string kDefaultGlobalFrameName = "world";
*/
// Convenience typedef
//typedef kindr::minimal::QuatTransformation Transformation;

/*struct TransformationStamped {
  ros::Time stamp;
  Transformation transformation;
};
*/

struct CameraCalibration {
  float fx;
  float fy;
  float cx;
  float cy;
};

constexpr double kDefaultMessageFrequency = 30; //Hz
const std::string kDefaultImageTopicName = "image";
const std::string kDefaultDepthTopicName = "depth";
const std::string kDefaultPointcloudTopicName = "pointcloud";
const std::string kDefaultTransformTopicName = "transform";
constexpr bool kDefaultDistancesInCm = false;
constexpr bool kDefaultUsePovData = true;

// Class handling global alignment calculation and publishing
class HandaToRosbag {
 public:
  // Constructor
  HandaToRosbag(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Runs the conversion
  void run();

 private:
  // Loading parameters from ros
  void getParametersFromRos(const ros::NodeHandle& nh_private);

  // Gets the camera parameters setting the relavent members
  bool loadCameraParameters();

  // Loads an image at an index, returning false if none is available
  bool loadImage(const int image_idx, cv::Mat* image_ptr) const;
  bool loadDepth(const int image_idx, cv::Mat* depth_ptr) const;
  bool loadDepthPov(const std::string& depth_path, cv::Mat* depth_ptr) const;
  bool loadDepthTum(const std::string& depth_path, cv::Mat* depth_ptr) const;
  bool loadPose(const int image_idx, Transformation* T_W_C_ptr) const;

  // Generates a Handa image path from the index
  inline std::string indexToImagePathPov(const int idx) const;
  inline std::string indexToImagePathTum(const int idx) const;
  inline std::string indexToDepthPathPov(const int idx) const;
  inline std::string indexToDepthPathTum(const int idx) const;
  inline std::string indexToPosePath(const int idx) const;
  inline std::string indexToStringPov(const int idx) const;
  inline std::string indexToStringTum(const int idx) const;

  // Time stamp generation
  ros::Time indexToTimestamp(const int idx) const;

  // Shows an image
  void displayImage(const cv::Mat& image) const;
  void displayDepth(const cv::Mat& depth) const;

  // Saves the relavent params of an image
  void saveImageParams(const cv::Mat& image);

  // Converts a matrix in POVRay native format to depth values (as described at
  // https://www.doc.ic.ac.uk/~ahanda/VaFRIC/codes.html)
  void povRayImageToDepthImage(cv::Mat* povray_depth_image) const;
  inline float povRayPixelToDepthPixel(const float povray_depth,
                                       const size_t row_idx,
                                       const size_t col_idx) const;

  // Converts a depth image to a pointcloud (various cloud options)
  void depthImageToPointcloud(
      const cv::Mat& depth,
      pcl::PointCloud<pcl::PointXYZ>* pointcloud_ptr) const;
  void depthImageToPointcloud(
    const cv::Mat& depth, const cv::Mat image,
    pcl::PointCloud<pcl::PointXYZRGB>* pointcloud_ptr) const;
  void depthImageToPointcloud(
    const cv::Mat& depth, const cv::Mat image,
    pcl::PointCloud<pcl::PointXYZI>* pointcloud_ptr) const;

  // Removes the offset such that the first pose is at position zero;
  void removePoseOffset(Transformation* T_W_C_ptr);

  // Converts a POVRay position vector to a metric one (basically divide by 100)
  void povRayPoseToMetricPose(Transformation* T_W_C_ptr) const;

  // Filepaths
  bool use_pov_data_; // True for POV data, false for TUM data
  std::string data_root_;
  std::string tum_data_root_;
  //std::string pose_path_; // For the freiburg pose format.
  std::string output_path_;

  // The output bag
  rosbag::Bag bag_;

  // Topic names
  std::string image_topic_name_;
  std::string depth_topic_name_;
  std::string pointcloud_topic_name_;
  std::string transform_topic_name_;

  // The size of the loaded images (valid after the first image is loaded)
  bool image_params_valid_;
  size_t image_size_rows_;
  size_t image_size_cols_;

  // Parameters
  const double message_frequency_;

  // Camera calibration
  CameraCalibration camera_calibration_;

  // Members for zeroing the first camera position
  bool first_pose_flag_;
  Transformation first_pose_;

  // This member indicates if loaded positions are in cm
  // This is true for the office datasets.
  bool distances_in_cm_;

  /*  // Subscribes and Advertises to the appropriate ROS topics
    void subscribeToTopics();
    void advertiseTopics();
    void getParametersFromRos();

    // Publishes the resulting transform
    void publishTFTransform(const ros::TimerEvent& event);

    // Callbacks
    // Subscribing to the transforms
    void transformCallback(const geometry_msgs::TransformStampedConstPtr& msg);

    // Node handles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Subscribers
    ros::Subscriber transform_sub_;

    // TF publishing timer and broadcaster object.
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Timer tf_timer_;

    // Stores the current subscribed transform
    TransformationStamped transform_;

    // Parameters
    bool verbose_;
    std::string local_frame_name_;
    std::string global_frame_name_;
  */

};

}  // namespace handa_to_rosbag

#endif /* HANDA_TO_ROSBAG_HANDA_TO_ROSBAG_H_ */
