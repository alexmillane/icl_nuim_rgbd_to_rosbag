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

  // Loading the camera parameters
  CHECK(loadCameraParameters()) << "Couldn't load the camera parameters";

  // Inspecting the other format depth images
  std::string image_debug_path =
      "/home/millanea/trunk/datasets/icl_nuim_rgbd_benchmark/"
      "office_room_traj2_loop/tum/depth/0.png";
  cv::Mat depth_debug_temp;
  depth_debug_temp =
      cv::imread(image_debug_path,
                 CV_LOAD_IMAGE_UNCHANGED);  // CV_8UC3, CV_LOAD_IMAGE_UNCHANGED
  std::cout << "depth_debug_temp.type(): " << depth_debug_temp.type()
            << std::endl;
  std::cout << "depth_debug_temp.rows: " << depth_debug_temp.rows << std::endl;
  std::cout << "depth_debug_temp.cols: " << depth_debug_temp.cols << std::endl;
  cv::Mat depth_debug(depth_debug_temp.rows, depth_debug_temp.cols, CV_32FC1);
  for (size_t row_idx = 0; row_idx < depth_debug_temp.rows; row_idx++) {
    for (size_t col_idx = 0; col_idx < depth_debug_temp.cols; col_idx++) {
      uint16_t pixel = depth_debug_temp.at<uint16_t>(row_idx, col_idx);
      float depth = static_cast<float>(pixel) / 5000.0;
      depth_debug.at<float>(row_idx, col_idx) = depth;
    }
  }
  //for (auto depth_it = depth_debug.begin<float>();
  //     depth_it != depth_debug.end<float>(); depth_it++) {
  //  std::cout << "depth: " << *depth_it << std::endl;
  //}

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
      // Converting to metric depth values
      povRayImageToDepthImage(&depth);

      // DEBUG
      // Inspecting the depth values
      //for (auto pixel_it = depth.begin<float>(); pixel_it !=
      // depth.end<float>();
      //     pixel_it++) {
      //  std::cout << "pixel: " << *pixel_it << std::endl;
      //}
/*      double min;
      double max;
      cv::minMaxLoc(depth, &min, &max);
      std::cout << "min: " << min << std::endl;
      std::cout << "max: " << max << std::endl;
*/

      for (size_t row_idx = 0; row_idx < depth.rows; row_idx++) {
        for (size_t col_idx = 0; col_idx < depth.cols; col_idx++) {
          float depth_val = depth.at<float>(row_idx, col_idx);
          float depth_debug_val = depth_debug.at<float>(row_idx, col_idx);
          std::cout << depth_val << " vs. " << depth_debug_val << std::endl;
        }
      }

      // Generating the pointcloud
      pcl::PointCloud<pcl::PointXYZ> pointcloud;
      //depthImageToPointcloud(depth, &pointcloud);

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
  const float planar_depth = povray_depth / std::sqrt(col_cx_by_fx * col_cx_by_fx +
                                  row_cy_by_fy * row_cy_by_fy + 1);
  // Converting from units which for some reason are in cm to m.
  // I could not spot this anywhere in the docs... No idea why its in cm.
  constexpr float kCmToM = 1.0 / 100.0;
  return planar_depth * kCmToM;
}

void HandaToRosbag::depthImageToPointcloud(
    const cv::Mat& depth,
    pcl::PointCloud<pcl::PointXYZ>* pointcloud_ptr) const {
  CHECK_NOTNULL(pointcloud_ptr);

  // Looping over depth values and converting to points
  /*  for (auto pixel_it = depth.begin<float>(); pixel_it != depth.end<float>();
         pixel_it++) {
  */

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
      // DEBUG
      std::cout << "point: " << point << std::endl;
    }
  }
}

}  // namespace handa_to_rosbag