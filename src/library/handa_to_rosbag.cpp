#include "handa_to_rosbag/handa_to_rosbag.h"

#include <fstream>
#include <iomanip>
#include <sstream>

#include <sensor_msgs/Image.h>

#include <glog/logging.h>

#include "handa_to_rosbag/pose_loader.h"

namespace handa_to_rosbag {

HandaToRosbag::HandaToRosbag(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private)
    : image_params_valid_(false),
      message_frequency_(kDefaultMessageFrequency),
      image_topic_name_(kDefaultImageTopicName),
      depth_topic_name_(kDefaultDepthTopicName),
      pointcloud_topic_name_(kDefaultPointcloudTopicName),
      transform_topic_name_(kDefaultTransformTopicName),
      first_pose_flag_(true) {
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

  // Loading the camera parameters
  CHECK(loadCameraParameters()) << "Couldn't load the camera parameters";

  // Looping and loading images
  int image_idx = 0;
  bool first_image = true;
  cv::Mat image;
  cv::Mat depth;
  while (ros::ok()) {
    // DEBUG
    std::cout << "image_idx: " << std::endl << image_idx << std::endl;

    // Getting the timestamp
    const ros::Time timestamp = indexToTimestamp((image_idx + 1));

    // Loading the RGB image
    loadImage(image_idx, &image);
    // Writing the image to the bag
    sensor_msgs::Image image_msg;
    imageToRos(image, &image_msg);
    bag_.write(image_topic_name_, timestamp, image_msg);
    // Saving the image params if required
    if (!image_params_valid_) {
      saveImageParams(image);
    }

    // Loading the depth image and converting to metric depth values
    loadDepth(image_idx, &depth);
    povRayImageToDepthImage(&depth);
    // Writing the depth image to the bag
    sensor_msgs::Image depth_msg;
    depthToRos(depth, &depth_msg);
    bag_.write(depth_topic_name_, timestamp, depth_msg);

    // Generating the pointcloud
    // Note pcl stamp is in MICROSECONDS, not nanoseconds.
    // pcl::PointCloud<pcl::PointXYZ> pointcloud;
    // depthImageToPointcloud(depth, &pointcloud);
    // pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    // depthImageToPointcloud(depth, image, &pointcloud);
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    depthImageToPointcloud(depth, image, &pointcloud);
    pointcloud.header.frame_id = "cam";
    pointcloud.header.stamp = static_cast<uint64_t>(timestamp.toNSec() / 1000);
    bag_.write(pointcloud_topic_name_, timestamp, pointcloud);

    // Gettting the camera pose in the world (POVRay) frame
    Transformation T_W_C;
    loadPose(image_idx, &T_W_C);
    povRayPoseToMetricPose(&T_W_C);
    removePoseOffset(&T_W_C);
    // Writing to TransformStamped to bag
    geometry_msgs::TransformStamped T_W_C_msg;
    transformToRos(T_W_C, &T_W_C_msg);
    T_W_C_msg.header.stamp = timestamp;
    T_W_C_msg.header.frame_id = "world";
    T_W_C_msg.child_frame_id = "cam";
    bag_.write(transform_topic_name_, timestamp, T_W_C_msg);
    // Writing to Tf to bag
    tf::tfMessage T_W_C_tf_msg;
    T_W_C_tf_msg.transforms.push_back(T_W_C_msg);
    bag_.write("/tf", timestamp, T_W_C_tf_msg);

    // DEBUG
    //std::cout << "T_W_C: " << std::endl << T_W_C.asVector() << std::endl;

    // DEBUG
    // displayImage(image);
    // displayDepth(depth);

    ++image_idx;
  }
}

bool HandaToRosbag::loadCameraParameters() {
  // Hardcoded camera intrinsics matrix as per:
  // https://www.doc.ic.ac.uk/~ahanda/VaFRIC/codes.html
  //
  // float K[3][3] = { 481.20, 0,       319.50,
  //                   0,      -480.00, 239.50,
  //                   0,      0,       1.00   };
  camera_calibration_.fx = 481.20;
  camera_calibration_.fy = -480.00;
  camera_calibration_.cx = 319.50;
  camera_calibration_.cy = 239.50;
  // Hardcoded cam params are always successful
  return true;
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

bool HandaToRosbag::loadPose(const int image_idx,
                             Transformation* T_W_C_ptr) const {
  CHECK_NOTNULL(T_W_C_ptr);
  // The path to the pose file
  std::string pose_path = indexToPosePath(image_idx);
  // Loading the transformation using the loader
  loadHandaPose(pose_path, T_W_C_ptr);
  // NEED TO DO SOME TESTING IF THIS WORKED
  return true;
}

inline std::string HandaToRosbag::indexToImagePath(const int idx) const {
  return (data_root_ + "/" + "scene_" + indexToString(idx) + ".png");
}

inline std::string HandaToRosbag::indexToDepthPath(const int idx) const {
  return (data_root_ + "/" + "scene_" + indexToString(idx) + ".depth");
}

inline std::string HandaToRosbag::indexToPosePath(const int idx) const {
  return (data_root_ + "/" + "scene_" + indexToString(idx) + ".txt");
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
    for (size_t col_idx = 0; col_idx < povray_depth_image->cols; col_idx++) {
      // Getting POVRay depth
      const float pov_depth = povray_depth_image->at<float>(row_idx, col_idx);
      // Converting
      const float metric_depth =
          povRayPixelToDepthPixel(pov_depth, row_idx, col_idx);
      // Overwritting
      povray_depth_image->at<float>(row_idx, col_idx) = metric_depth;
      // DEBUG
      // std::cout << pov_depth << " -> " << metric_depth << std::endl;
    }
  }
}

float HandaToRosbag::povRayPixelToDepthPixel(const float povray_depth,
                                             const size_t row_idx,
                                             const size_t col_idx) const {
  // Conversion as per the converter found at
  // https://www.doc.ic.ac.uk/~ahanda/VaFRIC/codes.html)
  // float u_u0_by_fx = (u - u0) / focal_x;
  // float v_v0_by_fy = (v - v0) / focal_y;
  // depth_array[u + v * img_width] =
  //    depth_array[u + v * img_width] /
  //    sqrt(u_u0_by_fx * u_u0_by_fx + v_v0_by_fy * v_v0_by_fy + 1);
  const float row_cy_by_fy =
      (static_cast<float>(row_idx) - camera_calibration_.cy) /
      camera_calibration_.fy;
  const float col_cx_by_fx =
      (static_cast<float>(col_idx) - camera_calibration_.cx) /
      camera_calibration_.fx;
  const float planar_depth =
      povray_depth /
      std::sqrt(col_cx_by_fx * col_cx_by_fx + row_cy_by_fy * row_cy_by_fy + 1);
  // Converting from units which for some reason are in cm to m.
  // I could not spot this anywhere in the docs... No idea why its in cm.
  constexpr float kCmToM = 1.0 / 100.0;
  return planar_depth * kCmToM;
}

void HandaToRosbag::depthImageToPointcloud(
    const cv::Mat& depth,
    pcl::PointCloud<pcl::PointXYZ>* pointcloud_ptr) const {
  CHECK_NOTNULL(pointcloud_ptr);
  // Looping and generating 3D points
  for (size_t row_idx = 0; row_idx < depth.rows; row_idx++) {
    for (size_t col_idx = 0; col_idx < depth.cols; col_idx++) {
      // Creating the point
      pcl::PointXYZ point;
      point.z = depth.at<float>(row_idx, col_idx);
      point.x = (static_cast<float>(col_idx) - camera_calibration_.cx) *
                point.z / camera_calibration_.fx;
      point.y = (static_cast<float>(row_idx) - camera_calibration_.cy) *
                point.z / camera_calibration_.fy;
      // Adding to the could
      pointcloud_ptr->push_back(point);
    }
  }
}

void HandaToRosbag::depthImageToPointcloud(
    const cv::Mat& depth, const cv::Mat image,
    pcl::PointCloud<pcl::PointXYZRGB>* pointcloud_ptr) const {
  CHECK_NOTNULL(pointcloud_ptr);
  // Looping and generating 3D points
  for (size_t row_idx = 0; row_idx < depth.rows; row_idx++) {
    for (size_t col_idx = 0; col_idx < depth.cols; col_idx++) {
      // Creating the point
      pcl::PointXYZRGB point;
      point.z = depth.at<float>(row_idx, col_idx);
      point.x = (static_cast<float>(col_idx) - camera_calibration_.cx) *
                point.z / camera_calibration_.fx;
      point.y = (static_cast<float>(row_idx) - camera_calibration_.cy) *
                point.z / camera_calibration_.fy;
      // Adding the color
      point.b = image.at<cv::Vec3b>(row_idx, col_idx)[0];
      point.g = image.at<cv::Vec3b>(row_idx, col_idx)[1];
      point.r = image.at<cv::Vec3b>(row_idx, col_idx)[2];
      // Adding to the could
      pointcloud_ptr->push_back(point);
    }
  }
}

void HandaToRosbag::depthImageToPointcloud(
    const cv::Mat& depth, const cv::Mat image,
    pcl::PointCloud<pcl::PointXYZI>* pointcloud_ptr) const {
  CHECK_NOTNULL(pointcloud_ptr);
  // Looping and generating 3D points
  for (size_t row_idx = 0; row_idx < depth.rows; row_idx++) {
    for (size_t col_idx = 0; col_idx < depth.cols; col_idx++) {
      // Creating the point
      pcl::PointXYZI point;
      point.z = depth.at<float>(row_idx, col_idx);
      point.x = (static_cast<float>(col_idx) - camera_calibration_.cx) *
                point.z / camera_calibration_.fx;
      point.y = (static_cast<float>(row_idx) - camera_calibration_.cy) *
                point.z / camera_calibration_.fy;
      // Adding the color
      const uint8_t b = image.at<cv::Vec3b>(row_idx, col_idx)[0];
      const uint8_t g = image.at<cv::Vec3b>(row_idx, col_idx)[1];
      const uint8_t r = image.at<cv::Vec3b>(row_idx, col_idx)[2];
      point.intensity = bgrToMono(b, g, r);
      // Adding to the could
      pointcloud_ptr->push_back(point);
    }
  }
}

void HandaToRosbag::removePoseOffset(Transformation* T_W_C_ptr) {
  CHECK_NOTNULL(T_W_C_ptr);
  // Saving the first camera pose if required
  if (first_pose_flag_ == true) {
    first_pose_ = *T_W_C_ptr;
    first_pose_flag_ = false;
  }
  // Removing the first camera position offset
  //T_W_C_ptr->getPosition() =
  //    first_pose_.getPosition() - T_W_C_ptr->getPosition();
  *T_W_C_ptr = first_pose_.inverse() * (*T_W_C_ptr);
}

void HandaToRosbag::povRayPoseToMetricPose(Transformation* T_W_C_ptr) const {
  CHECK_NOTNULL(T_W_C_ptr);
  // Units are in cm for some fucked up reason
  constexpr float kCmToM = 1.0 / 100.0;
  T_W_C_ptr->getPosition() = T_W_C_ptr->getPosition() * kCmToM;
}

}  // namespace handa_to_rosbag