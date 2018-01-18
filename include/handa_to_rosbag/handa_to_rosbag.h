#ifndef HANDA_TO_ROSBAG_HANDA_TO_ROSBAG_H_
#define HANDA_TO_ROSBAG_HANDA_TO_ROSBAG_H_

#include <string>
//#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <opencv2/highgui/highgui.hpp>

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

constexpr double kDefaultMessageFrequency = 30; //Hz
const std::string kDefaultImageTopicName = "image";
const std::string kDefaultDepthTopicName = "depth";

// Class handling global alignment calculation and publishing
class HandaToRosbag {
 public:
  // Constructor
  HandaToRosbag(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Runs the conversion
  void run();

 private:

  // Loads an image at an index, returning false if none is available
  bool loadImage(const int image_idx, cv::Mat* image_ptr) const;
  bool loadDepth(const int image_idx, cv::Mat* depth_ptr) const;

  // Generates a Handa image path from the index
  std::string indexToImagePath(const int idx) const;
  std::string indexToDepthPath(const int idx) const;
  inline std::string indexToString(const int idx) const;

  // Time stamp generation
  ros::Time indexToTimestamp(const int idx) const;

  // Shows an image
  void displayImage(const cv::Mat& image) const;
  void displayDepth(const cv::Mat& depth) const;

  // Saves the relavent params of an image
  void saveImageParams(const cv::Mat& image);

  // Converts a matrix in POVRay native format to depth values (as described at
  // https://www.doc.ic.ac.uk/~ahanda/VaFRIC/codes.html)
  // TODO

  // Filepaths
  std::string data_root_;
  std::string output_path_;

  // The output bag
  rosbag::Bag bag_;

  // Topic names
  std::string image_topic_name_;
  std::string depth_topic_name_;

  // The size of the loaded images (valid after the first image is loaded)
  bool image_params_valid_;
  size_t image_size_rows_;
  size_t image_size_cols_;

  // Parameters
  const double message_frequency_;

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
