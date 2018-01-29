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
      first_pose_flag_(true),
      distances_in_cm_(kDefaultDistancesInCm),
      use_pov_data_(kDefaultUsePovData) {
  // Getting data and params
  getParametersFromRos(nh_private);

  // HARDCODED PATHS
  // data_root_ = std::string(
  //    "/home/millanea/trunk/datasets/icl_nuim_rgbd_benchmark/living_room/"
  //    "living_room_traj0_loop/raw");

  // output_path_ = std::string(
  //    "/home/millanea/trunk/datasets/icl_nuim_rgbd_benchmark/living_room/"
  //    "living_room_traj0_loop/rosbag/data.bag");

  // pose_path_ = std::string(
  //    "/home/millanea/trunk/datasets/icl_nuim_rgbd_benchmark/living_room/"
  //    "livingRoom0.gt.freiburg");
}

void HandaToRosbag::getParametersFromRos(const ros::NodeHandle& nh_private) {
  // Loading whether to use POV or TUM data
  nh_private.param("use_pov_data", use_pov_data_, use_pov_data_);
  // Path to raw POV Data
  // NOTE(alexmillane): This is slightly confusing at the moment because even if
  // TUM format is selected we need this path for the poses.
  CHECK(nh_private.getParam("data_root", data_root_))
      << "Must provide the path to the raw data (POV format) in the "
         "parameter \"data_root\"";
  // Getting the input and output paths
  if (use_pov_data_) {
    std::cout << "POV format data selected" << std::endl;
  } else {
    std::cout << "TUM format data selected" << std::endl;
    CHECK(nh_private.getParam("tum_data_root", tum_data_root_))
        << "Must provide the path to the raw data (TUM format) in the "
           "parameter \"tum_data_root\"";
  }
  CHECK(nh_private.getParam("output_path", output_path_))
      << "Must provide the path to the output folder in the parameter "
         "\"output_path\"";
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
  while (ros::ok()) {
    // DEBUG
    std::cout << "image_idx: " << std::endl << image_idx << std::endl;

    // Getting the timestamp
    const ros::Time timestamp = indexToTimestamp((image_idx + 1));

    // Loading the RGB image (exiting loop if out of images)
    cv::Mat image;
    if (!loadImage(image_idx, &image)) {
      break;
    }
    // Writing the image to the bag
    sensor_msgs::Image image_msg;
    imageToRos(image, &image_msg);
    bag_.write(image_topic_name_, timestamp, image_msg);
    // Saving the image params if required
    if (!image_params_valid_) {
      saveImageParams(image);
    }
    // Loading the depth image and converting to metric depth values
    cv::Mat depth;
    loadDepth(image_idx, &depth);

    // Inspecting the depth images
    // for (auto pixel_it = depth.begin<float>(); pixel_it !=
    // depth.end<float>();
    //     pixel_it++) {
    //  std::cout << "pixel: " << *pixel_it << std::endl;
    //}
    // double min;
    // double max;
    // cv::minMaxLoc(depth, &min, &max);
    // std::cout << "[ " << min << " to " << max << " ]" << std::endl;

    // Writing the depth image to the bag
    sensor_msgs::Image depth_msg;
    depthToRos(depth, &depth_msg);
    bag_.write(depth_topic_name_, timestamp, depth_msg);

    // Generating the pointcloud
    // Note pcl stamp is in MICROSECONDS, not nanoseconds.
    // pcl::PointCloud<pcl::PointXYZ> pointcloud;
    // depthImageToPointcloud(depth, &pointcloud);
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    depthImageToPointcloud(depth, image, &pointcloud);
    // pcl::PointCloud<pcl::PointXYZI> pointcloud;
    // depthImageToPointcloud(depth, image, &pointcloud);
    pointcloud.header.frame_id = "cam";
    pointcloud.header.stamp = static_cast<uint64_t>(timestamp.toNSec() / 1000);
    bag_.write(pointcloud_topic_name_, timestamp, pointcloud);
    // Gettting the camera pose in the world (POVRay) frame
    Transformation T_W_C;
    // Loading from native POVRay format
    loadPose(image_idx, &T_W_C);
    povRayPoseToMetricPose(&T_W_C);
    // Removing the initial camera offset if requested
    constexpr bool remove_pose_offet = false;
    if (remove_pose_offet) {
      removePoseOffset(&T_W_C);
    }
    // Writing to TransformStamped to bag
    geometry_msgs::TransformStamped T_W_C_msg;
    transformToRos(T_W_C, &T_W_C_msg);
    T_W_C_msg.header.stamp = timestamp;
    T_W_C_msg.header.frame_id = "world";  //"world";
    T_W_C_msg.child_frame_id = "cam";     //"cam";
    bag_.write(transform_topic_name_, timestamp, T_W_C_msg);
    // Writing to Tf to bag
    tf::tfMessage T_W_C_tf_msg;
    T_W_C_tf_msg.transforms.push_back(T_W_C_msg);
    bag_.write("/tf", timestamp, T_W_C_tf_msg);

    // DEBUG
    // std::cout << "T_W_C: " << std::endl << T_W_C.asVector() << std::endl;

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
  // Not that I have not used the negative focal length because I transform
  // everything into a right handed coordinate system and openCV definitions.
  camera_calibration_.fx = 481.20;
  camera_calibration_.fy = 480.00;
  camera_calibration_.cx = 319.50;
  camera_calibration_.cy = 239.50;
  // Hardcoded cam params are always successful
  return true;
}

bool HandaToRosbag::loadImage(const int image_idx, cv::Mat* image_ptr) const {
  // The path to the image
  std::string image_path;
  if (use_pov_data_) {
    image_path = indexToImagePathPov(image_idx);
  } else {
    image_path = indexToImagePathTum(image_idx);
  }
  // DEBUG
  std::cout << "image_path: " << image_path << std::endl;
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
  if (use_pov_data_) {
    image_temp.convertTo(*image_ptr, CV_8UC3, 1.0 / 255.0);
  } else {
    image_temp.convertTo(*image_ptr, CV_8UC3);
  }
  // Success
  return true;
}

bool HandaToRosbag::loadDepth(const int image_idx, cv::Mat* depth_ptr) const {
  CHECK(image_params_valid_)
      << "Need valid image params before loading depth image";
  // The path to the image
  std::string depth_path;
  if (use_pov_data_) {
    depth_path = indexToDepthPathPov(image_idx);
  } else {
    depth_path = indexToDepthPathTum(image_idx);
  }
  // DEBUG
  std::cout << "depth_path: " << depth_path << std::endl;
  // Loading the data
  if (use_pov_data_) {
    loadDepthPov(depth_path, depth_ptr);
  } else {
    loadDepthTum(depth_path, depth_ptr);
  }
  // NEED TO DO TESTING FOR RUNNING OUT OF VALUES EARLY.
  return true;
}

bool HandaToRosbag::loadDepthPov(const std::string& depth_path,
                                 cv::Mat* depth_ptr) const {
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
  // Converting the POV format to the planar depth
  povRayImageToDepthImage(depth_ptr);
}

bool HandaToRosbag::loadDepthTum(const std::string& depth_path,
                                 cv::Mat* depth_ptr) const {
  // Loading the image
  cv::Mat depth_temp;
  depth_temp = cv::imread(
      depth_path, CV_LOAD_IMAGE_UNCHANGED);  // CV_8UC3, CV_LOAD_IMAGE_UNCHANGED
  // Checking the image encoding
  // std::cout << "image_ptr->type():" << depth_temp.type() << std::endl;
  // Testing success
  if (!depth_temp.data) {
    std::cout << "Could not load image data.\n";
    return false;
  }

/*  // Inspecting for zeros
  int num_zeros = 0;
  for (size_t row_idx = 0; row_idx < depth_temp.rows; row_idx++) {
    for (size_t col_idx = 0; col_idx < depth_temp.cols; col_idx++) {
      uint16_t pixel_val = depth_temp.at<uint16_t>(row_idx, col_idx);
      if (pixel_val == 0) {
        num_zeros++;
      }
    }
  }
  std::cout << "num_zeros: " << num_zeros << std::endl;
*/
  // Converting to a ROS compatible type
  depth_temp.convertTo(*depth_ptr, CV_32FC1, 1.0 / 5000.0);

/*  // re Inspecting for zeros
  num_zeros = 0;
  for (size_t row_idx = 0; row_idx < depth_ptr->rows; row_idx++) {
    for (size_t col_idx = 0; col_idx < depth_ptr->cols; col_idx++) {
      float pixel_val = depth_ptr->at<float>(row_idx, col_idx);
      if (pixel_val == 0) {
        num_zeros++;
      }
    }
  }
  std::cout << "num_zeros: " << num_zeros << std::endl;
*/
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

inline std::string HandaToRosbag::indexToImagePathPov(const int idx) const {
  return (data_root_ + "/" + "scene_00_" + indexToStringPov(idx) + ".png");
}

inline std::string HandaToRosbag::indexToImagePathTum(const int idx) const {
  return (tum_data_root_ + "/rgb/" + indexToStringTum(idx) + ".png");
}

inline std::string HandaToRosbag::indexToDepthPathPov(const int idx) const {
  return (data_root_ + "/" + "scene_00_" + indexToStringPov(idx) + ".depth");
}

inline std::string HandaToRosbag::indexToDepthPathTum(const int idx) const {
  return (tum_data_root_ + "/depth/" + indexToStringTum(idx) + ".png");
}

inline std::string HandaToRosbag::indexToPosePath(const int idx) const {
  return (data_root_ + "/" + "scene_00_" + indexToStringPov(idx) + ".txt");
}

inline std::string HandaToRosbag::indexToStringPov(const int idx) const {
  // Getting a three digit index
  constexpr int num_chars = 4;
  std::stringstream ss;
  ss << std::setw(num_chars) << std::setfill('0') << idx;
  return ss.str();
}

inline std::string HandaToRosbag::indexToStringTum(const int idx) const {
  return std::to_string(idx);
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
  float planar_depth =
      povray_depth /
      std::sqrt(col_cx_by_fx * col_cx_by_fx + row_cy_by_fy * row_cy_by_fy + 1);
  // Converting from units which for some reason are in cm to m.
  // I could not spot this anywhere in the docs... No idea why its in cm.
  // NOTE(alexmillane): This conversion is only needed in the office dataset
  if (distances_in_cm_) {
    constexpr float kCmToM = 1.0 / 100.0;
    planar_depth = planar_depth * kCmToM;
  }
  return planar_depth;
}

void HandaToRosbag::depthImageToPointcloud(
    const cv::Mat& depth,
    pcl::PointCloud<pcl::PointXYZ>* pointcloud_ptr) const {
  CHECK_NOTNULL(pointcloud_ptr);
  // Looping and generating 3D points
  for (size_t row_idx = 0; row_idx < depth.rows; row_idx++) {
    for (size_t col_idx = 0; col_idx < depth.cols; col_idx++) {
      // Getting the depth
      const float depth_val = depth.at<float>(row_idx, col_idx);
      // Testing for missing depth
      if (depth_val > std::numeric_limits<float>::min()) {
        // Creating the point
        pcl::PointXYZ point;
        point.z = depth_val;
        point.x = (static_cast<float>(col_idx) - camera_calibration_.cx) *
                  point.z / camera_calibration_.fx;
        point.y = (static_cast<float>(row_idx) - camera_calibration_.cy) *
                  point.z / camera_calibration_.fy;
        // Adding to the could
        pointcloud_ptr->push_back(point);
      }
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
      // Getting the depth
      const float depth_val = depth.at<float>(row_idx, col_idx);
      // Testing for missing depth
      if (depth_val > std::numeric_limits<float>::min()) {
        // Creating the point
        pcl::PointXYZRGB point;
        point.z = depth_val;
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
}

void HandaToRosbag::depthImageToPointcloud(
    const cv::Mat& depth, const cv::Mat image,
    pcl::PointCloud<pcl::PointXYZI>* pointcloud_ptr) const {
  CHECK_NOTNULL(pointcloud_ptr);
  // Looping and generating 3D points
  for (size_t row_idx = 0; row_idx < depth.rows; row_idx++) {
    for (size_t col_idx = 0; col_idx < depth.cols; col_idx++) {
      // Getting the depth
      const float depth_val = depth.at<float>(row_idx, col_idx);
      // Testing for missing depth
      if (depth_val > std::numeric_limits<float>::min()) {
        // Creating the point
        pcl::PointXYZI point;
        point.z = depth_val;
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
}

void HandaToRosbag::removePoseOffset(Transformation* T_W_C_ptr) {
  CHECK_NOTNULL(T_W_C_ptr);
  // Saving the first camera pose if required
  if (first_pose_flag_ == true) {
    first_pose_ = *T_W_C_ptr;
    first_pose_flag_ = false;
  }
  // Removing the first camera position offset
  // T_W_C_ptr->getPosition() =
  //    first_pose_.getPosition() - T_W_C_ptr->getPosition();
  *T_W_C_ptr = first_pose_.inverse() * (*T_W_C_ptr);
}

void HandaToRosbag::povRayPoseToMetricPose(Transformation* T_W_C_ptr) const {
  CHECK_NOTNULL(T_W_C_ptr);
  // Units are in cm for some fucked up reason
  if (distances_in_cm_) {
    constexpr float kCmToM = 1.0 / 100.0;
    T_W_C_ptr->getPosition() = T_W_C_ptr->getPosition() * kCmToM;
  }
}

}  // namespace handa_to_rosbag