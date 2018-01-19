#include "handa_to_rosbag/handa_to_rosbag.h"

#include <fstream>
#include <iomanip>
#include <sstream>

#include <sensor_msgs/Image.h>

#include <glog/logging.h>

#include "handa_to_rosbag/conversions.h"

namespace handa_to_rosbag {

HandaToRosbag::HandaToRosbag(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private)
    : image_params_valid_(false),
      message_frequency_(kDefaultMessageFrequency),
      image_topic_name_(kDefaultImageTopicName),
      depth_topic_name_(kDefaultDepthTopicName) {
  // Getting data and params
  // subscribeToTopics();
  // advertiseTopics();
  // getParametersFromRos();

  // HARDCODED PATHS
  data_root_ = std::string(
      "/home/millanea/trunk/datasets/icl_nuim_rgbd_benchmark/"
      "office_room_traj2_loop/raw");

  output_path_ = std::string(
      "/home/millanea/trunk/datasets/icl_nuim_rgbd_benchmark/"
      "office_room_traj2_loop/rosbag/data.bag");
}

void HandaToRosbag::run() {
  // Convert all entries
  // for (size_t image_idx = 0; image_idx < 1000; image_idx++) {

  // Opening the bag
  bag_.open(output_path_, rosbag::bagmode::Write);

  // Looping and loading images
  int image_idx = 0;
  bool first_image = true;
  cv::Mat image;
  cv::Mat depth;
  while (loadImage(image_idx, &image) && ros::ok()) {
    // Saving the image params if required
    if (!image_params_valid_) {
      saveImageParams(image);
    }

    // DEBUG
    std::cout << "image_idx: " << std::endl << image_idx << std::endl;
    ++image_idx;

    // Getting the timestamp
    const ros::Time timestamp = indexToTimestamp(image_idx);

    // Loading the depth image
    loadDepth(image_idx, &depth);

    // NEED TO CONVERT FROM THE POVRAY DEPTH VALUES TO METRIC VALUES
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    povRayImageToDepthImage(&depth);

    // Writing the image to the bag
    sensor_msgs::Image image_msg;
    imageToRos(image, &image_msg);
    bag_.write(image_topic_name_, timestamp, image_msg);

    // Writing the depth image to the bag
    sensor_msgs::Image depth_msg;
    depthToRos(depth, &depth_msg);
    bag_.write(depth_topic_name_, timestamp, depth_msg);

    // DEBUG
    // displayImage(image);
    // displayDepth(depth);
  }
}

bool HandaToRosbag::loadImage(const int image_idx, cv::Mat* image_ptr) const {
  // The path to the image
  std::string image_path(indexToImagePath(image_idx));
  // Loading the image
  cv::Mat image_temp;
  image_temp = cv::imread(
      image_path, CV_LOAD_IMAGE_UNCHANGED);  // CV_8UC3, CV_LOAD_IMAGE_UNCHANGED
  // Checking the image encoding
  // std::cout << "image_ptr->type():" << image_ptr->type() << std::endl;
  // Testing success
  if (!image_temp.data) {
    std::cout << "Could not load image data.\n";
    return false;
  }
  // Converting to a ROS compatible type
  image_temp.convertTo(*image_ptr, CV_8UC3, 1.0 / 255.0);
  // Success
  return true;
}

bool HandaToRosbag::loadDepth(const int image_idx, cv::Mat* depth_ptr) const {
  CHECK(image_params_valid_)
      << "Need valid image params before loading depth image";
  // The path to the image
  std::string depth_path(indexToDepthPath(image_idx));
  // Creating the (empty) image
  *depth_ptr = cv::Mat(image_size_rows_, image_size_cols_, CV_32FC1);
  // Loading the depth values into the matrix
  float m;
  std::ifstream fileStream(depth_path);
  int idx = 0;
  while (fileStream >> m) {
    const int row = idx / image_size_cols_;
    const int col = idx % image_size_cols_;
    depth_ptr->at<float>(row, col) = m;
    idx++;
  }
  // NEED TO DO TESTING FOR RUNNING OUT OF VALUES EARLY.
  return true;
}

inline std::string HandaToRosbag::indexToImagePath(const int idx) const {
  return (data_root_ + "/" + "scene_" + indexToString(idx) + ".png");
}

inline std::string HandaToRosbag::indexToDepthPath(const int idx) const {
  return (data_root_ + "/" + "scene_" + indexToString(idx) + ".depth");
}

inline std::string HandaToRosbag::indexToString(const int idx) const {
  // Getting a three digit index
  std::stringstream ss;
  ss << std::setw(3) << std::setfill('0') << idx;
  return ss.str();
}

inline ros::Time HandaToRosbag::indexToTimestamp(const int idx) const {
  double timestamp = static_cast<double>(idx) / message_frequency_;
  return ros::Time(timestamp);
}

void HandaToRosbag::displayImage(const cv::Mat& image) const {
  // Create a window for display.
  cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
  // Show our image inside it.
  cv::imshow("Display window", image);
  // Wait for a keystroke in the window
  cv::waitKey(0);
}

void HandaToRosbag::displayDepth(const cv::Mat& depth) const {
  // Normalizing
  cv::Mat normalized_depth;
  cv::normalize(depth, normalized_depth, 1.0, 0.0, cv::NORM_MINMAX);
  // Displaying the normalized image
  displayImage(normalized_depth);
}

void HandaToRosbag::saveImageParams(const cv::Mat& image) {
  image_size_rows_ = image.rows;
  image_size_cols_ = image.cols;
  image_params_valid_ = true;
}

void HandaToRosbag::povRayImageToDepthImage(cv::Mat* povray_depth_image) const {
  // Looping over pixels in the image and converting
  for (size_t row_idx = 0; row_idx < povray_depth_image->rows; row_idx++) {
    for (size_t col_idx = 0; col_idx < povray_depth_image->rows; col_idx++) {
      povRayPixelToDepthPixel(povray_depth_image->at<float>(row_idx, col_idx),
                              row_idx, col_idx);
    }
  }
}

float HandaToRosbag::povRayPixelToDepthPixel(const float povray_depth,
                                             const size_t row_idx,
                                             const size_t col_idx) const {
  // NEED TO LOAD THE FOCUS!!!!!
  constexpr float f = 1.0;
  // Conversion as per: https://www.doc.ic.ac.uk/~ahanda/VaFRIC/codes.html)
  return f * std::sqrt(std::pow(povray_depth, 2) /
                       (std::pow(col_idx, 2) + std::pow(f, 2) +
                        std::pow(row_idx, 2)));
}

/*void HandaToRosbag::subscribeToTopics() {
  // Subscribing to the required data topics
  transform_sub_ = nh_.subscribe("transform_to_be_converted", 1,
                                 &HandaToRosbag::transformCallback, this);
}

void HandaToRosbag::advertiseTopics() {
  // Advertising topics
  tf_timer_ = nh_.createTimer(ros::Duration(0.01),
                              &HandaToRosbag::publishTFTransform, this);
}

void HandaToRosbag::getParametersFromRos() {
  // Retrieving the parameters
  nh_private_.getParam("verbose", verbose_);
  nh_private_.getParam("local_frame_name", local_frame_name_);
  nh_private_.getParam("global_frame_name", global_frame_name_);
}

void HandaToRosbag::transformCallback(
    const geometry_msgs::TransformStampedConstPtr& msg) {
  // Converting and storing the transform
  transform_.stamp = msg->header.stamp;
  tf::transformMsgToKindr(msg->transform, &transform_.transformation);
}

void HandaToRosbag::publishTFTransform(const ros::TimerEvent& event) {
  tf::Transform tf_transform;
  tf::transformKindrToTF(transform_.transformation, &tf_transform);
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      tf_transform, ros::Time::now(), global_frame_name_, local_frame_name_));
}
*/
}  // namespace handa_to_rosbag